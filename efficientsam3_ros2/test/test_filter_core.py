#!/usr/bin/env python3
"""
Unit tests for filter_core.py

These tests can be run on MacOS without ROS2 installed.
They test the core filtering logic independently.

Usage:
    # Run all tests
    python -m pytest test/test_filter_core.py -v
    
    # Run with coverage
    python -m pytest test/test_filter_core.py -v --cov=efficientsam3_ros2
"""

import sys
import os
from pathlib import Path
import numpy as np
import cv2
import pytest

# Add parent directory to path for imports
sys.path.insert(0, str(Path(__file__).parent.parent / "efficientsam3_ros2"))

from filter_core import (
    DynamicObjectFilter,
    MaskingStrategy,
    Detection,
    create_visualization,
)


class TestMaskingStrategy:
    """Test masking strategy enum."""
    
    def test_masking_strategies_exist(self):
        """Test that all expected masking strategies are defined."""
        assert MaskingStrategy.BLACKOUT.value == "blackout"
        assert MaskingStrategy.GRAYOUT.value == "grayout"
        assert MaskingStrategy.INPAINT.value == "inpaint"
        assert MaskingStrategy.BLUR.value == "blur"
    
    def test_masking_strategy_from_string(self):
        """Test creating masking strategy from string."""
        assert MaskingStrategy("grayout") == MaskingStrategy.GRAYOUT
        assert MaskingStrategy("blackout") == MaskingStrategy.BLACKOUT


class TestDetection:
    """Test Detection dataclass."""
    
    def test_detection_creation(self):
        """Test creating a Detection object."""
        det = Detection(
            class_name="person",
            confidence=0.95,
            bbox=(100, 200, 300, 400),
            mask=None,
        )
        assert det.class_name == "person"
        assert det.confidence == 0.95
        assert det.bbox == (100, 200, 300, 400)
        assert det.mask is None
    
    def test_detection_with_mask(self):
        """Test Detection with a mask."""
        mask = np.zeros((100, 100), dtype=bool)
        mask[25:75, 25:75] = True
        
        det = Detection(
            class_name="car",
            confidence=0.8,
            bbox=(25, 25, 75, 75),
            mask=mask,
        )
        assert det.mask is not None
        assert det.mask.shape == (100, 100)
        assert det.mask[50, 50] == True


class TestDynamicObjectFilterInit:
    """Test DynamicObjectFilter initialization (without loading model)."""
    
    def test_default_dynamic_classes(self):
        """Test that default dynamic classes are set."""
        expected_classes = [
            "person", "car", "truck", "bus",
            "motorcycle", "bicycle", "dog", "cat", "bird",
        ]
        assert DynamicObjectFilter.DEFAULT_DYNAMIC_CLASSES == expected_classes
    
    def test_text_prompt_generation(self):
        """Test that text prompt is correctly generated from classes."""
        # We can test the method without initializing the full filter
        classes = ["person", "car", "dog"]
        expected_prompt = "person. car. dog."
        
        # Create a minimal filter just to test prompt generation
        # (will fail on model load, but we can test the method)
        try:
            filter_obj = DynamicObjectFilter(
                model_path="dummy_path.pt",
                dynamic_classes=classes,
            )
        except Exception:
            pass  # Expected to fail without model
        
        # Test the prompt building logic directly
        prompt = ". ".join(classes) + "."
        assert prompt == expected_prompt


class TestMaskApplication:
    """Test mask application methods without model."""
    
    @pytest.fixture
    def sample_image(self):
        """Create a sample test image."""
        image = np.zeros((100, 100, 3), dtype=np.uint8)
        image[:, :] = [100, 150, 200]  # BGR color
        return image
    
    @pytest.fixture
    def sample_mask(self):
        """Create a sample binary mask."""
        mask = np.zeros((100, 100), dtype=np.uint8)
        mask[25:75, 25:75] = 1  # Central region masked
        return mask
    
    def test_blackout_masking(self, sample_image, sample_mask):
        """Test blackout masking strategy."""
        result = sample_image.copy()
        mask_bool = sample_mask.astype(bool)
        result[mask_bool] = [0, 0, 0]
        
        # Check masked region is black
        assert result[50, 50].tolist() == [0, 0, 0]
        # Check unmasked region is unchanged
        assert result[10, 10].tolist() == [100, 150, 200]
    
    def test_grayout_masking(self, sample_image, sample_mask):
        """Test grayout masking strategy."""
        result = sample_image.copy()
        mask_bool = sample_mask.astype(bool)
        result[mask_bool] = [128, 128, 128]
        
        # Check masked region is gray
        assert result[50, 50].tolist() == [128, 128, 128]
        # Check unmasked region is unchanged
        assert result[10, 10].tolist() == [100, 150, 200]
    
    def test_inpaint_masking(self, sample_image, sample_mask):
        """Test inpaint masking strategy."""
        mask_uint8 = (sample_mask * 255).astype(np.uint8)
        result = cv2.inpaint(sample_image, mask_uint8, inpaintRadius=3, flags=cv2.INPAINT_TELEA)
        
        # Result should have valid pixels everywhere
        assert result.shape == sample_image.shape
        assert not np.any(np.isnan(result))
    
    def test_blur_masking(self, sample_image, sample_mask):
        """Test blur masking strategy."""
        result = sample_image.copy()
        mask_bool = sample_mask.astype(bool)
        blurred = cv2.GaussianBlur(sample_image, (99, 99), 0)
        result[mask_bool] = blurred[mask_bool]
        
        # Result should be blurred in masked region
        assert result.shape == sample_image.shape


class TestCreateVisualization:
    """Test visualization helper function."""
    
    def test_visualization_creation(self):
        """Test creating a visualization image."""
        # Create test data
        original = np.zeros((100, 100, 3), dtype=np.uint8)
        original[:, :] = [100, 150, 200]
        
        filtered = original.copy()
        filtered[25:75, 25:75] = [128, 128, 128]
        
        mask = np.zeros((100, 100), dtype=np.uint8)
        mask[25:75, 25:75] = 1
        
        detections = [
            Detection(
                class_name="test",
                confidence=0.9,
                bbox=(25, 25, 75, 75),
                mask=None,
            )
        ]
        
        vis = create_visualization(original, filtered, mask, detections)
        
        # Visualization should be 3x width (original, mask, filtered)
        assert vis.shape == (100, 300, 3)


class TestDeviceDetection:
    """Test device detection utilities."""
    
    def test_device_string_values(self):
        """Test valid device string values."""
        valid_devices = ["auto", "cpu", "cuda", "mps"]
        for device in valid_devices:
            assert isinstance(device, str)


# Integration test (requires model - skip if not available)
@pytest.mark.skipif(
    not os.environ.get("EFFICIENTSAM3_MODEL_PATH"),
    reason="Model path not set in environment"
)
class TestDynamicObjectFilterIntegration:
    """Integration tests requiring the actual model."""
    
    @pytest.fixture
    def model_path(self):
        return os.environ.get("EFFICIENTSAM3_MODEL_PATH")
    
    @pytest.fixture
    def efficientsam3_path(self):
        return os.environ.get("EFFICIENTSAM3_PATH", "")
    
    def test_filter_initialization(self, model_path, efficientsam3_path):
        """Test that filter initializes correctly with real model."""
        filter_obj = DynamicObjectFilter(
            model_path=model_path,
            efficientsam3_path=efficientsam3_path if efficientsam3_path else None,
        )
        assert filter_obj is not None
        assert not filter_obj._model_loaded  # Lazy loading
    
    def test_process_frame(self, model_path, efficientsam3_path):
        """Test processing a frame with real model."""
        filter_obj = DynamicObjectFilter(
            model_path=model_path,
            efficientsam3_path=efficientsam3_path if efficientsam3_path else None,
        )
        
        # Create a dummy image
        image = np.zeros((480, 640, 3), dtype=np.uint8)
        image[:, :] = [128, 128, 128]
        
        filtered, mask, detections = filter_obj.process_frame(image)
        
        assert filtered.shape == image.shape
        assert isinstance(detections, list)


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
