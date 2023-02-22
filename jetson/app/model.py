import os
import pathlib

import numpy as np

try:
    # To use the tflite_runtime package
    import tflite_runtime.interpreter as tflite
except:
    # To use the standard tensorflow package
    from tensorflow import lite as tflite

os.environ["GLOG_minloglevel"] ="3"

MODEL_BUCKET_URL = os.environ['MODEL_BUCKET_URL']
MODEL_SAVE_DIRECTORY = os.environ['MODEL_SAVE_DIRECTORY']
MODEL_FILENAME = os.environ['MODEL_FILENAME']

# Fetch model

def fetchModel(remote_model_name):
    pathlib.Path(MODEL_SAVE_DIRECTORY).mkdir(parents=True, exist_ok=True)
    decompressed_model_name = remote_model_name.replace('.tar.gz', '')
    if decompressed_model_name in os.listdir(MODEL_SAVE_DIRECTORY):
        print('Model \'{}\' exists locally, skipping download'.format(decompressed_model_name))
        return
    os.system('wget {0}/{1} -nv -O /tmp/{1}'.format(MODEL_BUCKET_URL, remote_model_name))
    if remote_model_name.endswith('.tar.gz'):
        os.system('tar -xzf /tmp/{} -C {}'.format(remote_model_name, MODEL_SAVE_DIRECTORY))
    else:
        os.system('mv /tmp/{} {}/{}'.format(remote_model_name, MODEL_SAVE_DIRECTORY, remote_model_name))
    if decompressed_model_name not in os.listdir(MODEL_SAVE_DIRECTORY):
        raise Exception('Error downloading dataset \'{}\''.format(remote_model_name))
    print('Downloaded dataset \'{}\''.format(decompressed_model_name))

class Predictor:
    def __init__(self, model_path):
        self.model_path = model_path
        self.interpreter = tflite.Interpreter(model_path=model_path)
        self.interpreter.allocate_tensors()
        self.input_details = self.interpreter.get_input_details()
        self.output_details = self.interpreter.get_output_details()

    def predict(self, input_data):
        input_data = np.array([input_data], dtype=np.float32)
        # input_data = [[1, 2, 3]]
        self.interpreter.set_tensor(self.input_details[0]['index'], input_data)
        self.interpreter.invoke()
        output_data = self.interpreter.get_tensor(self.output_details[0]['index'])
        # output_data = [[1, 2]]
        return output_data[0]

fetchModel(MODEL_FILENAME)
