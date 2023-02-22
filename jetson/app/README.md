# Tensorflow Model

Below is a code snippet that showcases how to prepare the Tensorflow model:

```python
import numpy as np
import tensorflow as tf

WINDOW_SIZE = 5
NUM_FEATURES = 3
HIDDEN_DIM = 32
NUM_OUTPUTS = 2

model = tf.keras.models.Sequential([
    tf.keras.layers.Flatten(input_shape=(WINDOW_SIZE, NUM_FEATURES), name='flatten'),
    tf.keras.layers.Dense(HIDDEN_DIM, activation='relu', name='hidden'),
    tf.keras.layers.Dense(NUM_OUTPUTS, name='output'),
])
model.summary()

my_input = [
    [10, 1, 2],
    [20, 1, 2],
    [30, 1, 2],
    [40, 1, 2],
    [50, 1, 2],
]
my_input = np.array([my_input], dtype=np.float32)
my_input[0] # (1, 5, 3)

my_output = model.predict(my_input)
my_output[0] # (1, 2)

run_model = tf.function(lambda x: model(x))
concrete_func = run_model.get_concrete_function(
    tf.TensorSpec([1, WINDOW_SIZE, NUM_FEATURES], model.inputs[0].dtype)
)

tf_model_dir = "tf_model"
model.save(tf_model_dir, save_format="tf", signatures=concrete_func)
# !tar -czvf tf_model.tar.gz tf_model

converter = tf.lite.TFLiteConverter.from_saved_model(tf_model_dir)
tflite_model = converter.convert()
with open('tflite_model.tflite', 'wb') as f:
    f.write(tflite_model)
```
