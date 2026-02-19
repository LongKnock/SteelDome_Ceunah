# mturret_vision/backends/cuda_backend.py
from ultralytics import YOLO
from .base_backend import BaseBackend

class CUDABackend(BaseBackend):
    def __init__(self, model_path, config):
        # Note: Ultralytics handles model loading internally, so we just pass the path
        # It automatically finds the GPU if available.
        self.model = YOLO(model_path.replace('.onnx', '.pt')) 
        
    def detect(self, image):
        # YOLOv8 handles all preprocessing (resize, norm) internally
        results = self.model(image, verbose=False)
        
        # Return results in a standard format
        return results[0].boxes