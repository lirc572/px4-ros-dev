import os

from flask import Flask, request, jsonify

from model import Predictor

INPUT_COLUMNS = [
    'x', 'y', 'z',
]
WINDOW_SIZE = 5

MODEL_SAVE_DIRECTORY = os.environ['MODEL_SAVE_DIRECTORY']
MODEL_FILENAME = os.environ['MODEL_FILENAME']

predictor = Predictor('{}/{}'.format(MODEL_SAVE_DIRECTORY, MODEL_FILENAME))

app = Flask(__name__)

@app.route("/")
def root():
    return jsonify({"message": "Server running"})


@app.route("/predict", methods=['POST'])
def predict():
    req = request.get_json()
    my_input = []
    if not isinstance(req, list):
        return jsonify({"error": "Input must be a list"}), 400
    if len(req) != WINDOW_SIZE:
        return jsonify({"error": "Input must be a list of length {}".format(WINDOW_SIZE)}), 400
    try:
        for x in req:
            new_x = []
            for col in INPUT_COLUMNS:
                if col not in x:
                    raise KeyError()
                new_x.append(x[col])
            my_input.append(new_x)
    except KeyError:
        return jsonify({"error": "Missing input columns, required columns: [{}]".format(', '.join(INPUT_COLUMNS))}), 400
    except:
        return jsonify({"error": "Unknown error"}), 500
    print('Prediction:')
    print(my_input)
    my_output = predictor.predict(my_input).tolist()
    print(my_output)
    return jsonify({"a": my_output[0], "b": my_output[1]})
