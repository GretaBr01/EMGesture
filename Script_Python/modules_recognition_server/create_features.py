import os
import pandas as pd
import numpy as np
from scipy.stats import skew, kurtosis
import pickle
from scipy.signal import welch
from scipy.signal import butter, filtfilt

# === CONFIGURAZIONE ===
SENSOR_DATA = {
    "acc": {"frequenza_campionamento": 208, "full_scale": 2**15},
    "gyr": {"frequenza_campionamento": 208, "full_scale": 2**15},
    "emg": {"frequenza_campionamento": 1000, "full_scale": 4095.0},
}

feature_functions = {
    'mean': lambda x, **kwargs: np.mean(x),
    'std': lambda x, **kwargs: np.std(x),
    'min': lambda x, **kwargs: np.min(x),
    'max': lambda x, **kwargs: np.max(x),
    'q5': lambda x, **kwargs: np.percentile(x, 5),
    'q10': lambda x, **kwargs: np.percentile(x, 10),
    'q90': lambda x, **kwargs: np.percentile(x, 90),
    'q95': lambda x, **kwargs: np.percentile(x, 95),
    'energy': lambda x, delta_t, **kwargs: np.sum(x**2) * delta_t,
    'rms': lambda x, **kwargs: np.sqrt(np.mean(x**2)),
    'energy_ac': lambda x_ac, delta_t, **kwargs: np.sum(x_ac**2) * delta_t,
    'rms_ac': lambda x_ac, **kwargs: np.sqrt(np.mean(x_ac**2)),
    'skewness': lambda x, **kwargs: skew(x),
    'kurtosis': lambda x, **kwargs: kurtosis(x),
    # Frequency-domain
    'mnf': lambda x, fs, **kwargs: np.sum(np.abs(np.fft.rfft(x)) * np.fft.rfftfreq(len(x), d=1/fs)) / np.sum(np.abs(np.fft.rfft(x))),
    'mdf': lambda x, fs, **kwargs: np.interp(
        0.5 * np.sum(np.abs(np.fft.rfft(x))**2),
        np.cumsum(np.abs(np.fft.rfft(x))**2),
        np.fft.rfftfreq(len(x), d=1/fs)
    ),
}

sensor_features  = {
    'acc': 'mean',
    'gyr': 'mean',
    'emg': 'rms_ac',
}

def bandpass_filter(data, fs, lowcut=60.0, highcut=250.0, order=8):
    nyq = 0.5 * fs
    low = lowcut / nyq
    high = highcut / nyq
    b, a = butter(order, [low, high], btype='band')
    return filtfilt(b, a, data)


def signal_statistic(x, sensor_name,  Norm=True, apply_filter=False):
    fs = SENSOR_DATA[sensor_name]["frequenza_campionamento"]
    if Norm:
        x = x / SENSOR_DATA[sensor_name]["full_scale"]

    if sensor_name == "emg" and apply_filter:
        x = bandpass_filter(x, fs)

    delta_t = 1.0 / fs
    x_dc = np.mean(x)
    x_ac = x - x_dc
    L = len(x)

    selected_feature = sensor_features.get(sensor_name)
    if selected_feature is None:
        raise ValueError(f"Nessuna feature specificata per il sensore '{sensor_name}'.")

    func = feature_functions[selected_feature]
    result = func(x=x, x_ac=x_ac, delta_t=delta_t, L=L, fs=fs)
    return pd.Series({selected_feature: result})

