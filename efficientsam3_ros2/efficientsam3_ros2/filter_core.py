"""
Core dynamic object filtering logic using EfficientSAM3.

This module contains the DynamicObjectFilter class that handles all the
segmentation and masking logic. It is designed to be ROS2-independent
so it can be tested locally on MacOS without ROS2 installed.

Usage (standalone testing):
    from filter_core import DynamicObjectFilter
    
    filter = DynamicObjectFilter(
        model_path="path/to/checkpoint.pt",
        dynamic_classes=["person", "car", "bicycle"]
    )
    
    filtered_image, masks, detections = filter.process_frame(image)
"""

import sys
import os
from pathlib import Path
from typing import List, Optional, Tuple, Dict, Any, Union
from dataclasses import dataclass
from enum import Enum

import numpy as np
import cv2

# Try to import torch - will fail gracefully if not available
try:
    import torch
    TORCH_AVAILABLE = True
except ImportError:
    TORCH_AVAILABLE = False
    print("Warning: PyTorch not available. DynamicObjectFilter will not work.")

# Try to import PIL
try:
    from PIL import Image
    PIL_AVAILABLE = True
except ImportError:
    PIL_AVAILABLE = False


class MaskingStrategy(Enum):
    """Strategy for handling masked (dynamic) regions."""
    BLACKOUT = "blackout"      # Set to black (0, 0, 0)
    GRAYOUT = "grayout"        # Set to gray (128, 128, 128)
    INPAINT = "inpaint"        # Use OpenCV inpainting
    BLUR = "blur"              # Apply strong blur to masked regions


@dataclass
class Detection:
    """Represents a single detected dynamic object."""
    class_name: str
    confidence: float
    bbox: Tuple[float, float, float, float]  # x1, y1, x2, y2
    mask: Optional[np.ndarray] = None


class DynamicObjectFilter:
    """
    Filters dynamic objects from images using EfficientSAM3.
    
    This class loads an EfficientSAM3 model and uses it to detect and mask
    dynamic objects (people, vehicles, animals, etc.) from camera frames.
    The filtered frames can then be used by SLAM systems that assume a
    static environment.
    
    Args:
        model_path: Path to the EfficientSAM3 checkpoint file
        efficientsam3_path: Path to the efficientsam3_arm package (if not in PYTHONPATH)
        dynamic_classes: List of object classes to filter (e.g., ["person", "car"])
        confidence_threshold: Minimum confidence for detection (0.0 to 1.0)
        masking_strategy: How to handle masked regions
        device: Device to run inference on ("cpu", "mps", "cuda", or "auto")
        backbone_type: EfficientSAM3 backbone type (default: "repvit")
        model_name: EfficientSAM3 model size (default: "s" for small)
    """
    
    # Default dynamic object classes commonly found in SLAM environments
    DEFAULT_DYNAMIC_CLASSES = [
        "person",
        "car", 
        "truck",
        "bus",
        "motorcycle",
        "bicycle",
        "dog",
        "cat",
        "bird",
    ]
    
    # Map checkpoint filename patterns to (backbone_type, model_name, text_encoder_type)
    MODEL_FILENAME_MAP = {
        "repvit-m0_9_mobileclip_s1": ("repvit", "m0_9", "MobileCLIP-S1"),
        "repvit-m0_9": ("repvit", "m0_9", "MobileCLIP-S1"),
        "repvit_m0_9": ("repvit", "m0_9", "MobileCLIP-S1"),
        "repvit_s": ("repvit", "m0_9", "MobileCLIP-S1"),
        "repvit_m1_1": ("repvit", "m1_1", "MobileCLIP-S1"),
        "repvit_m2_3": ("repvit", "m2_3", "MobileCLIP-S1"),
        "tinyvit_m": ("tinyvit", "m", "MobileCLIP-S1"),
        "tinyvit_l": ("tinyvit", "l", "MobileCLIP-S1"),
    }
    
    @staticmethod
    def _infer_model_config(model_path: str, backbone_type: str, model_name: str):
        """
        Auto-detect backbone_type, model_name, and text_encoder_type from the checkpoint filename.
        Falls back to provided defaults if detection fails.
        """
        import os
        basename = os.path.basename(model_path).lower()
        # Strip prefix and extension: "efficient_sam3_repvit-m0_9_mobileclip_s1.pth" -> "repvit-m0_9_mobileclip_s1"
        stem = basename.replace("efficient_sam3_", "").replace(".pth", "").replace(".pt", "")
        
        for pattern, (bt, mn, te) in DynamicObjectFilter.MODEL_FILENAME_MAP.items():
            if pattern in stem:
                return bt, mn, te
        
        # Fallback to provided values
        return backbone_type, model_name, "MobileCLIP-S1"

    def __init__(
        self,
        model_path: str,
        efficientsam3_path: Optional[str] = None,
        dynamic_classes: Optional[List[str]] = None,
        confidence_threshold: float = 0.3,
        masking_strategy: MaskingStrategy = MaskingStrategy.GRAYOUT,
        device: str = "auto",
        backbone_type: str = "repvit",
        model_name: str = "s",
    ):
        if not TORCH_AVAILABLE:
            raise RuntimeError("PyTorch is required but not installed")
        
        self.model_path = model_path
        self.dynamic_classes = dynamic_classes or self.DEFAULT_DYNAMIC_CLASSES
        self.confidence_threshold = confidence_threshold
        self.masking_strategy = masking_strategy
        
        # Auto-detect backbone, model name, and text encoder from checkpoint filename
        self.backbone_type, self.model_name, self.text_encoder_type = self._infer_model_config(
            model_path, backbone_type, model_name
        )
        
        # Add efficientsam3_arm to path if specified
        if efficientsam3_path:
            sys.path.insert(0, efficientsam3_path)
        
        # Determine device
        if device == "auto":
            self.device = self._get_optimal_device()
        else:
            self.device = device
        
        # Model and processor (lazy loaded)
        self._model = None
        self._processor = None
        self._model_loaded = False
        
        # Build the text prompt from dynamic classes
        self._text_prompt = self._build_text_prompt()
        
        # Statistics
        self.total_frames_processed = 0
        self.total_detections = 0
    
    def _get_optimal_device(self) -> str:
        """Auto-detect the best available device."""
        if torch.cuda.is_available():
            return "cuda"
        elif hasattr(torch.backends, 'mps') and torch.backends.mps.is_available():
            return "mps"
        return "cpu"
    
    def _build_text_prompt(self) -> str:
        """Build the text prompt for EfficientSAM3 from dynamic classes."""
        # EfficientSAM3 expects prompts like "person. car. dog."
        return ". ".join(self.dynamic_classes) + "."
    
    def _load_model(self) -> None:
        """Lazy load the EfficientSAM3 model."""
        if self._model_loaded:
            return
        
        try:
            # Import EfficientSAM3 ARM modules
            from efficientsam3_arm import build_efficientsam3_image_model
            from efficientsam3_arm.model.sam3_image_processor import Sam3Processor
            
            print(f"Loading EfficientSAM3 model from {self.model_path}...")
            print(f"Device: {self.device}, Backbone: {self.backbone_type}, Size: {self.model_name}, Text Encoder: {self.text_encoder_type}")
            
            # Build model (matching the reference notebook usage)
            self._model = build_efficientsam3_image_model(
                checkpoint_path=self.model_path,
                backbone_type=self.backbone_type,
                model_name=self.model_name,
                text_encoder_type=self.text_encoder_type,
                device=self.device,
                enable_inst_interactivity=False,
            )
            
            # Create processor (matching reference notebook usage)
            self._processor = Sam3Processor(
                model=self._model,
                device=self.device,
                confidence_threshold=self.confidence_threshold,
            )
            
            self._model_loaded = True
            print("Model loaded successfully!")
            
        except ImportError as e:
            raise RuntimeError(
                f"Failed to import EfficientSAM3 modules. "
                f"Make sure efficientsam3_arm is in your PYTHONPATH. Error: {e}"
            )
        except Exception as e:
            raise RuntimeError(f"Failed to load EfficientSAM3 model: {e}")
    
    def process_frame(
        self, 
        image: Union[np.ndarray, "Image.Image"],
        return_visualization: bool = False,
    ) -> Tuple[np.ndarray, Optional[np.ndarray], List[Detection]]:
        """
        Process a single frame to filter out dynamic objects.
        
        Args:
            image: Input image (numpy array BGR or PIL Image RGB)
            return_visualization: If True, also return a visualization image
            
        Returns:
            Tuple of:
                - filtered_image: Image with dynamic objects masked
                - combined_mask: Binary mask of all dynamic objects (or None if no detections)
                - detections: List of Detection objects with details
        """
        # Ensure model is loaded
        self._load_model()
        
        # Convert to PIL if needed (EfficientSAM3 expects PIL or RGB)
        if isinstance(image, np.ndarray):
            # Assume BGR (OpenCV format) - convert to RGB for PIL
            image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            pil_image = Image.fromarray(image_rgb)
            original_bgr = image.copy()
        else:
            pil_image = image
            original_bgr = cv2.cvtColor(np.array(image), cv2.COLOR_RGB2BGR)
        
        # Run inference
        state = self._processor.set_image(pil_image)
        state = self._processor.set_text_prompt(self._text_prompt, state)
        
        # Extract detections
        detections = []
        combined_mask = None
        
        if "masks" in state and state["masks"] is not None and len(state["masks"]) > 0:
            masks = state["masks"].cpu().numpy()
            boxes = state["boxes"].cpu().numpy()
            scores = state["scores"].cpu().numpy()
            
            # Remove channel dimension if present
            if masks.ndim == 4:
                masks = masks.squeeze(1)
            
            h, w = original_bgr.shape[:2]
            combined_mask = np.zeros((h, w), dtype=np.uint8)
            
            for i in range(len(scores)):
                # Create detection object
                detection = Detection(
                    class_name="dynamic_object",  # EfficientSAM3 doesn't provide class names
                    confidence=float(scores[i]),
                    bbox=tuple(boxes[i].tolist()),
                    mask=masks[i] > 0.5,
                )
                detections.append(detection)
                
                # Add to combined mask
                mask_binary = (masks[i] > 0.5).astype(np.uint8)
                # Resize mask to image size if needed
                if mask_binary.shape != (h, w):
                    mask_binary = cv2.resize(mask_binary, (w, h), interpolation=cv2.INTER_NEAREST)
                combined_mask = np.maximum(combined_mask, mask_binary)
            
            self.total_detections += len(detections)
        
        # Apply masking strategy
        if combined_mask is not None and combined_mask.any():
            filtered_image = self._apply_mask(original_bgr, combined_mask)
        else:
            filtered_image = original_bgr.copy()
        
        self.total_frames_processed += 1
        
        return filtered_image, combined_mask, detections
    
    def _apply_mask(self, image: np.ndarray, mask: np.ndarray) -> np.ndarray:
        """
        Apply the masking strategy to filter out dynamic regions.
        
        Args:
            image: Original BGR image
            mask: Binary mask (255 = dynamic region to mask)
            
        Returns:
            Filtered image with dynamic regions handled according to strategy
        """
        result = image.copy()
        mask_bool = mask.astype(bool)
        
        if self.masking_strategy == MaskingStrategy.BLACKOUT:
            result[mask_bool] = [0, 0, 0]
            
        elif self.masking_strategy == MaskingStrategy.GRAYOUT:
            result[mask_bool] = [128, 128, 128]
            
        elif self.masking_strategy == MaskingStrategy.INPAINT:
            # Use OpenCV inpainting to fill masked regions
            mask_uint8 = (mask * 255).astype(np.uint8)
            result = cv2.inpaint(image, mask_uint8, inpaintRadius=3, flags=cv2.INPAINT_TELEA)
            
        elif self.masking_strategy == MaskingStrategy.BLUR:
            # Apply strong Gaussian blur to masked regions
            blurred = cv2.GaussianBlur(image, (99, 99), 0)
            result[mask_bool] = blurred[mask_bool]
        
        return result
    
    def update_dynamic_classes(self, classes: List[str]) -> None:
        """Update the list of dynamic object classes to filter."""
        self.dynamic_classes = classes
        self._text_prompt = self._build_text_prompt()
    
    def update_confidence_threshold(self, threshold: float) -> None:
        """Update the confidence threshold for detections."""
        self.confidence_threshold = threshold
        if self._processor is not None:
            self._processor.confidence_threshold = threshold
    
    def update_masking_strategy(self, strategy: MaskingStrategy) -> None:
        """Update the masking strategy."""
        self.masking_strategy = strategy
    
    def get_stats(self) -> Dict[str, Any]:
        """Get processing statistics."""
        return {
            "total_frames_processed": self.total_frames_processed,
            "total_detections": self.total_detections,
            "avg_detections_per_frame": (
                self.total_detections / self.total_frames_processed 
                if self.total_frames_processed > 0 else 0
            ),
            "device": self.device,
            "model_loaded": self._model_loaded,
            "dynamic_classes": self.dynamic_classes,
            "confidence_threshold": self.confidence_threshold,
            "masking_strategy": self.masking_strategy.value,
        }
    
    def reset_stats(self) -> None:
        """Reset processing statistics."""
        self.total_frames_processed = 0
        self.total_detections = 0


def create_visualization(
    original: np.ndarray,
    filtered: np.ndarray,
    mask: Optional[np.ndarray],
    detections: List[Detection],
) -> np.ndarray:
    """
    Create a side-by-side visualization of original and filtered images.
    
    Args:
        original: Original BGR image
        filtered: Filtered BGR image
        mask: Binary mask (or None)
        detections: List of detections
        
    Returns:
        Visualization image with original, filtered, and mask side by side
    """
    h, w = original.shape[:2]
    
    # Create mask visualization
    if mask is not None:
        mask_vis = cv2.cvtColor((mask * 255).astype(np.uint8), cv2.COLOR_GRAY2BGR)
        # Color the mask
        mask_colored = original.copy()
        mask_colored[mask.astype(bool)] = [0, 0, 255]  # Red for dynamic regions
        mask_vis = cv2.addWeighted(original, 0.5, mask_colored, 0.5, 0)
    else:
        mask_vis = np.zeros_like(original)
    
    # Draw bounding boxes on filtered image
    filtered_with_boxes = filtered.copy()
    for det in detections:
        x1, y1, x2, y2 = [int(v) for v in det.bbox]
        cv2.rectangle(filtered_with_boxes, (x1, y1), (x2, y2), (0, 255, 0), 2)
        label = f"{det.confidence:.2f}"
        cv2.putText(filtered_with_boxes, label, (x1, y1 - 5), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
    
    # Stack horizontally
    vis = np.hstack([original, mask_vis, filtered_with_boxes])
    
    # Add labels
    font = cv2.FONT_HERSHEY_SIMPLEX
    cv2.putText(vis, "Original", (10, 30), font, 1, (255, 255, 255), 2)
    cv2.putText(vis, "Mask", (w + 10, 30), font, 1, (255, 255, 255), 2)
    cv2.putText(vis, "Filtered", (2*w + 10, 30), font, 1, (255, 255, 255), 2)
    
    return vis


# ============================================================================
# Standalone testing functions (can run on MacOS without ROS2)
# ============================================================================

def test_with_image(
    image_path: str,
    model_path: str,
    efficientsam3_path: Optional[str] = None,
    output_path: Optional[str] = None,
    show: bool = True,
) -> None:
    """
    Test the DynamicObjectFilter with a single image.
    
    Args:
        image_path: Path to input image
        model_path: Path to EfficientSAM3 checkpoint
        efficientsam3_path: Path to efficientsam3_arm package
        output_path: Optional path to save output
        show: Whether to display the result
    """
    print(f"Testing with image: {image_path}")
    
    # Load image
    image = cv2.imread(image_path)
    if image is None:
        raise ValueError(f"Failed to load image: {image_path}")
    
    # Create filter
    filter_obj = DynamicObjectFilter(
        model_path=model_path,
        efficientsam3_path=efficientsam3_path,
        masking_strategy=MaskingStrategy.GRAYOUT,
    )
    
    # Process
    filtered, mask, detections = filter_obj.process_frame(image)
    
    print(f"Detections: {len(detections)}")
    for i, det in enumerate(detections):
        print(f"  {i+1}. confidence={det.confidence:.3f}, bbox={det.bbox}")
    
    # Create visualization
    vis = create_visualization(image, filtered, mask, detections)
    
    # Save if requested
    if output_path:
        cv2.imwrite(output_path, vis)
        print(f"Saved result to: {output_path}")
    
    # Show if requested
    if show:
        cv2.imshow("Dynamic Object Filter Test", vis)
        print("Press any key to close...")
        cv2.waitKey(0)
        cv2.destroyAllWindows()
    
    # Print stats
    print("\nStats:", filter_obj.get_stats())


if __name__ == "__main__":
    import argparse
    
    parser = argparse.ArgumentParser(description="Test DynamicObjectFilter")
    parser.add_argument("--image", type=str, required=True, help="Path to input image")
    parser.add_argument("--model", type=str, required=True, help="Path to EfficientSAM3 checkpoint")
    parser.add_argument("--efficientsam3-path", type=str, default=None, 
                        help="Path to efficientsam3_arm package")
    parser.add_argument("--output", type=str, default=None, help="Output path for visualization")
    parser.add_argument("--no-show", action="store_true", help="Don't display result")
    
    args = parser.parse_args()
    
    test_with_image(
        image_path=args.image,
        model_path=args.model,
        efficientsam3_path=args.efficientsam3_path,
        output_path=args.output,
        show=not args.no_show,
    )
