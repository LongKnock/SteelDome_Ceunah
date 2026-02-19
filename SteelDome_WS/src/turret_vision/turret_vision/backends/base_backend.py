# turret_vision/backends/base_backend.py
import cv2
import numpy as np

class BaseBackend:
    def __init__(self, model_path, config):
        self.model_path = model_path
        self.config = config
        self.classes = self.load_classes(config['class_file'])

    def load_classes(self, path):
        with open(path, 'r') as f:
            return [line.strip() for line in f.readlines()]

    def detect(self, image):
        """
        Input: cv2 image (BGR)
        Output: List of detections via [class_id, confidence, x, y, w, h]
        """
        raise NotImplementedError("You must implement detect() in the subclass!")