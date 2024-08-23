import onnxruntime as ort

# Load the ONNX model
model_path = '/home/john/krac2024/src/pleaseland/onnx_models/DroneLanding-5000091.onnx'
session = ort.InferenceSession(model_path)

# Get the output details of the model
output_details = [(output.name, output.shape) for output in session.get_outputs()]
print(output_details)