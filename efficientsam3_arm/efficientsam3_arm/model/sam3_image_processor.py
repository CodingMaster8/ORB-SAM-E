# Copyright (c) Meta Platforms, Inc. and affiliates. All Rights Reserved
from typing import Dict, List, Optional

import numpy as np
import PIL
import torch
import torch.nn.functional as F

from efficientsam3_arm.model import box_ops_arm

from efficientsam3_arm.model.data_misc import FindStage, interpolate
from torchvision.transforms import v2


class Sam3Processor:
    """ """

    def __init__(self, model, resolution=1008, device=None, confidence_threshold=0.5,
                 preserve_aspect_ratio=True):
        if device is None:
            if torch.backends.mps.is_available():
                device = "mps"
            elif torch.cuda.is_available():
                device = "cuda"
            else:
                device = "cpu"
        self.model = model
        self.resolution = resolution
        self.device = device
        self.preserve_aspect_ratio = preserve_aspect_ratio
        self.transform = v2.Compose(
            [
                v2.ToDtype(torch.uint8, scale=True),
                v2.Resize(size=(resolution, resolution)),
                v2.ToDtype(torch.float32, scale=True),
                v2.Normalize(mean=[0.5, 0.5, 0.5], std=[0.5, 0.5, 0.5]),
            ]
        )
        self.confidence_threshold = confidence_threshold

        self.find_stage = FindStage(
            img_ids=torch.tensor([0], device=device, dtype=torch.long),
            text_ids=torch.tensor([0], device=device, dtype=torch.long),
            input_boxes=None,
            input_boxes_mask=None,
            input_boxes_label=None,
            input_points=None,
            input_points_mask=None,
        )

    def _preprocess_with_padding(self, image_tensor):
        """Resize preserving aspect ratio and pad to square resolution.

        Instead of squishing the image to a square, this resizes the longest
        side to ``self.resolution`` while keeping the aspect ratio, then pads
        the shorter side with the mean pixel value (0 in normalized space) so
        that the final tensor is ``(resolution, resolution)``.

        Args:
            image_tensor: ``(C, H, W)`` tensor (uint8 expected).

        Returns:
            Tuple of ``(preprocessed C×H×W float32 tensor, padding info dict)``.
        """
        # Ensure uint8 for high-quality resize
        if image_tensor.dtype != torch.uint8:
            if image_tensor.is_floating_point():
                image_tensor = (image_tensor * 255).clamp(0, 255).to(torch.uint8)
            else:
                image_tensor = image_tensor.to(torch.uint8)

        _, h, w = image_tensor.shape

        # Scale so the longest side equals resolution
        scale = self.resolution / max(h, w)
        new_h = min(int(round(h * scale)), self.resolution)
        new_w = min(int(round(w * scale)), self.resolution)

        # Resize preserving aspect ratio
        image_tensor = v2.functional.resize(image_tensor, [new_h, new_w], antialias=True)

        # Convert to float32 [0, 1] then normalize to [-1, 1]
        image_tensor = image_tensor.to(torch.float32) / 255.0
        image_tensor = v2.functional.normalize(
            image_tensor, mean=[0.5, 0.5, 0.5], std=[0.5, 0.5, 0.5]
        )

        # Compute center padding
        pad_h = self.resolution - new_h
        pad_w = self.resolution - new_w
        pad_top = pad_h // 2
        pad_bottom = pad_h - pad_top
        pad_left = pad_w // 2
        pad_right = pad_w - pad_left

        # Pad with 0.0 (mean in normalized space → neutral gray)
        image_tensor = F.pad(
            image_tensor,
            (pad_left, pad_right, pad_top, pad_bottom),
            mode="constant",
            value=0.0,
        )

        pad_info = {
            "pad_top": pad_top,
            "pad_left": pad_left,
            "resized_h": new_h,
            "resized_w": new_w,
        }
        return image_tensor, pad_info

    @torch.inference_mode()
    def set_image(self, image, state=None):
        """Sets the image on which we want to do predictions."""
        if state is None:
            state = {}

        if isinstance(image, PIL.Image.Image):
            width, height = image.size
        elif isinstance(image, (torch.Tensor, np.ndarray)):
            height, width = image.shape[-2:]
        else:
            raise ValueError("Image must be a PIL image or a tensor")

        image = v2.functional.to_image(image).to(self.device)

        if self.preserve_aspect_ratio:
            image, pad_info = self._preprocess_with_padding(image)
            image = image.unsqueeze(0)
            state["pad_info"] = pad_info
        else:
            image = self.transform(image).unsqueeze(0)
            state["pad_info"] = None

        state["original_height"] = height
        state["original_width"] = width
        state["backbone_out"] = self.model.backbone.forward_image(image)
        inst_interactivity_en = self.model.inst_interactive_predictor is not None
        if inst_interactivity_en and "sam2_backbone_out" in state["backbone_out"]:
            sam2_backbone_out = state["backbone_out"]["sam2_backbone_out"]
            sam2_backbone_out["backbone_fpn"][0] = (
                self.model.inst_interactive_predictor.model.sam_mask_decoder.conv_s0(
                    sam2_backbone_out["backbone_fpn"][0]
                )
            )
            sam2_backbone_out["backbone_fpn"][1] = (
                self.model.inst_interactive_predictor.model.sam_mask_decoder.conv_s1(
                    sam2_backbone_out["backbone_fpn"][1]
                )
            )
        return state

    @torch.inference_mode()
    def set_image_batch(self, images: List[np.ndarray], state=None):
        """Sets the image batch on which we want to do predictions."""
        if state is None:
            state = {}

        if not isinstance(images, list):
            raise ValueError("Images must be a list of PIL images or tensors")
        assert len(images) > 0, "Images list must not be empty"
        assert isinstance(
            images[0], PIL.Image.Image
        ), "Images must be a list of PIL images"

        state["original_heights"] = [image.height for image in images]
        state["original_widths"] = [image.width for image in images]

        if self.preserve_aspect_ratio:
            transformed = []
            pad_infos = []
            for image in images:
                img_tensor = v2.functional.to_image(image).to(self.device)
                img_tensor, pad_info = self._preprocess_with_padding(img_tensor)
                transformed.append(img_tensor)
                pad_infos.append(pad_info)
            images = torch.stack(transformed, dim=0)
            state["pad_infos"] = pad_infos
        else:
            images = [
                self.transform(v2.functional.to_image(image).to(self.device))
                for image in images
            ]
            images = torch.stack(images, dim=0)
            state["pad_infos"] = None

        state["backbone_out"] = self.model.backbone.forward_image(images)
        inst_interactivity_en = self.model.inst_interactive_predictor is not None
        if inst_interactivity_en and "sam2_backbone_out" in state["backbone_out"]:
            sam2_backbone_out = state["backbone_out"]["sam2_backbone_out"]
            sam2_backbone_out["backbone_fpn"][0] = (
                self.model.inst_interactive_predictor.model.sam_mask_decoder.conv_s0(
                    sam2_backbone_out["backbone_fpn"][0]
                )
            )
            sam2_backbone_out["backbone_fpn"][1] = (
                self.model.inst_interactive_predictor.model.sam_mask_decoder.conv_s1(
                    sam2_backbone_out["backbone_fpn"][1]
                )
            )
        return state

    @torch.inference_mode()
    def set_text_prompt(self, prompt: str, state: Dict):
        """Sets the text prompt and run the inference"""

        if "backbone_out" not in state:
            raise ValueError("You must call set_image before set_text_prompt")

        text_outputs = self.model.backbone.forward_text([prompt], device=self.device)
        # will erase the previous text prompt if any
        state["backbone_out"].update(text_outputs)
        if "geometric_prompt" not in state:
            state["geometric_prompt"] = self.model._get_dummy_prompt()

        return self._forward_grounding(state)

    @torch.inference_mode()
    def add_geometric_prompt(self, box: List, label: bool, state: Dict):
        """Adds a box prompt and run the inference.
        The image needs to be set, but not necessarily the text prompt.
        The box is assumed to be in [center_x, center_y, width, height] format and normalized in [0, 1] range.
        The label is True for a positive box, False for a negative box.
        """
        if "backbone_out" not in state:
            raise ValueError("You must call set_image before set_text_prompt")

        if "language_features" not in state["backbone_out"]:
            # Looks like we don't have a text prompt yet. This is allowed, but we need to set the text prompt to "visual" for the model to rely only on the geometric prompt
            dummy_text_outputs = self.model.backbone.forward_text(
                ["visual"], device=self.device
            )
            state["backbone_out"].update(dummy_text_outputs)

        if "geometric_prompt" not in state:
            state["geometric_prompt"] = self.model._get_dummy_prompt()

        # adding a batch and sequence dimension
        boxes = torch.tensor(box, device=self.device, dtype=torch.float32).view(1, 1, 4)

        # Convert from original [0,1] to padded [0,1] coordinate space
        pad_info = state.get("pad_info")
        if pad_info is not None:
            content_w_frac = pad_info["resized_w"] / self.resolution
            content_h_frac = pad_info["resized_h"] / self.resolution
            pad_left_frac = pad_info["pad_left"] / self.resolution
            pad_top_frac = pad_info["pad_top"] / self.resolution
            # box is [cx, cy, w, h] in [0,1] relative to original image
            boxes[..., 0] = boxes[..., 0] * content_w_frac + pad_left_frac  # cx
            boxes[..., 1] = boxes[..., 1] * content_h_frac + pad_top_frac   # cy
            boxes[..., 2] = boxes[..., 2] * content_w_frac                  # w
            boxes[..., 3] = boxes[..., 3] * content_h_frac                  # h

        labels = torch.tensor([label], device=self.device, dtype=torch.bool).view(1, 1)
        state["geometric_prompt"].append_boxes(boxes, labels)

        return self._forward_grounding(state)

    def reset_all_prompts(self, state: Dict):
        """Removes all the prompts and results"""
        if "backbone_out" in state:
            backbone_keys_to_del = [
                "language_features",
                "language_mask",
                "language_embeds",
            ]
            for key in backbone_keys_to_del:
                if key in state["backbone_out"]:
                    del state["backbone_out"][key]

        keys_to_del = ["geometric_prompt", "boxes", "masks", "masks_logits", "scores"]
        for key in keys_to_del:
            if key in state:
                del state[key]

    @torch.inference_mode()
    def set_confidence_threshold(self, threshold: float, state=None):
        """Sets the confidence threshold for the masks"""
        self.confidence_threshold = threshold
        if state is not None and "boxes" in state:
            # we need to filter the boxes again
            # In principle we could do this more efficiently since we would only need
            # to rerun the heads. But this is simpler and not too inefficient
            return self._forward_grounding(state)
        return state

    @torch.inference_mode()
    def _forward_grounding(self, state: Dict):
        outputs = self.model.forward_grounding(
            backbone_out=state["backbone_out"],
            find_input=self.find_stage,
            geometric_prompt=state["geometric_prompt"],
            find_target=None,
        )

        out_bbox = outputs["pred_boxes"]
        out_logits = outputs["pred_logits"]
        out_masks = outputs["pred_masks"]
        out_probs = out_logits.sigmoid()
        presence_score = outputs["presence_logit_dec"].sigmoid().unsqueeze(1)
        out_probs = (out_probs * presence_score).squeeze(-1)

        keep = out_probs > self.confidence_threshold
        out_probs = out_probs[keep]
        out_masks = out_masks[keep]
        out_bbox = out_bbox[keep]

        # convert to [x0, y0, x1, y1] format
        boxes = box_ops_arm.box_cxcywh_to_xyxy(out_bbox)

        img_h = state["original_height"]
        img_w = state["original_width"]

        pad_info = state.get("pad_info")
        if pad_info is not None:
            # Boxes are in [0,1] relative to the padded resolution×resolution image.
            # Convert to [0,1] relative to the actual image content.
            pad_left_frac = pad_info["pad_left"] / self.resolution
            pad_top_frac = pad_info["pad_top"] / self.resolution
            content_w_frac = pad_info["resized_w"] / self.resolution
            content_h_frac = pad_info["resized_h"] / self.resolution

            boxes[:, 0] = (boxes[:, 0] - pad_left_frac) / content_w_frac
            boxes[:, 1] = (boxes[:, 1] - pad_top_frac) / content_h_frac
            boxes[:, 2] = (boxes[:, 2] - pad_left_frac) / content_w_frac
            boxes[:, 3] = (boxes[:, 3] - pad_top_frac) / content_h_frac
            boxes = boxes.clamp(0, 1)

        # Scale to original pixel coordinates
        scale_fct = torch.tensor([img_w, img_h, img_w, img_h]).to(self.device)
        boxes = boxes * scale_fct[None, :]

        if pad_info is not None:
            # Masks: interpolate to padded resolution, crop padding, resize to original
            out_masks = interpolate(
                out_masks.unsqueeze(1),
                (self.resolution, self.resolution),
                mode="bilinear",
                align_corners=False,
            ).sigmoid()
            pt = pad_info["pad_top"]
            pl = pad_info["pad_left"]
            rh = pad_info["resized_h"]
            rw = pad_info["resized_w"]
            out_masks = out_masks[:, :, pt : pt + rh, pl : pl + rw]
            out_masks = interpolate(
                out_masks,
                (img_h, img_w),
                mode="bilinear",
                align_corners=False,
            )
        else:
            out_masks = interpolate(
                out_masks.unsqueeze(1),
                (img_h, img_w),
                mode="bilinear",
                align_corners=False,
            ).sigmoid()

        state["masks_logits"] = out_masks
        state["masks"] = out_masks > 0.5
        state["boxes"] = boxes
        state["scores"] = out_probs
        return state
