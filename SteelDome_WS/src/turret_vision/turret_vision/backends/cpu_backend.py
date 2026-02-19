import onnxruntime as ort
import numpy as np
import cv2
from .base_backend import BaseBackend

class CPUBackend(BaseBackend):
    def __init__(self, model_path, config):
        super().__init__(model_path, config)
        # Load ONNX Model
        self.session = ort.InferenceSession(model_path, providers=['CPUExecutionProvider'])

        self.input_name = self.session.get_inputs()[0].name
        self.input_shape = self.session.get_inputs()[0].shape
        self.img_size = (self.input_shape[2], self.input_shape[3])  # (height, width)

        def preprocess(self, img):
            #  Resize to 640x640
            blob = cv2.resize(img, self.img_size)
            #  BGR to RGB
            blob = cv2.cvtColor(blob, cv2.COLOR_BGR2RGB)
            #  HWC to CHW
            blob = blob.transpose((2, 0, 1))  # HWC to CHW
            #  Normalize to [0,1]
            blob = np.expand_dims(blob, axis=0).astype(np.float32)/255.0  # Add batch dimension
     
            return blob
        
        def postprocess(self, outputs, original_shape):
            # Placeholder for postprocessing logic (e.g., NMS, scaling boxes back to original image size)
            # This will depend on the specific output format of your ONNX model
            pass

        def detect(self, image):
            input_tensor = self.preprocess(image)

            outputs = self.session.run(None, {self.input_name: input_tensor})
