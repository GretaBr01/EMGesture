import numpy as np
import joblib
import os

model = joblib.load("model/mio_modello.joblib")

def prepare_feature_vector(features_dict):
    ordered_keys = [
        "acc_x", "acc_y", "acc_z",
        "gyr_x", "gyr_y", "gyr_z",
        "emg_0", "emg_1", "emg_2", "emg_3"
    ]
    vector = []
    for key in ordered_keys:
        stats = features_dict[key]
        vector.extend(stats.values())  # stats è un dict: {"mean": ..., "rms_ac": ..., ...}
    return np.array(vector).reshape(1, -1)  # 2D array (1, n_features)

def infer(features_acc, features_gyr, features_emg):    # features_acc è un dict: {acc: {"mean": ...} }
    all_features = {**features_acc, **features_gyr, **features_emg}
    vector = prepare_feature_vector(all_features)
    prediction = model.predict(vector)
    return prediction[0]
