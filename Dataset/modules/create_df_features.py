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

excluded_features = []

# === FUNZIONI DI SUPPORTO ===
def bandpass_filter(data, fs, lowcut=60.0, highcut=250.0, order=8):
    nyq = 0.5 * fs
    low = lowcut / nyq
    high = highcut / nyq
    b, a = butter(order, [low, high], btype='band')
    return filtfilt(b, a, data)


def get_sensor_type(col_name):
    for sensor in SENSOR_DATA:
        if col_name.startswith(sensor):
            return sensor
    return None

def signal_statistic(x, sensor_name, fs,  Norm=True):
    if Norm:
        x = x / SENSOR_DATA[sensor_name]["full_scale"]

    if sensor_name == "emg":
        x = bandpass_filter(x, fs)

    delta_t = 1.0 / SENSOR_DATA[sensor_name]["frequenza_campionamento"]
    x_dc = np.mean(x)
    x_ac = x - x_dc
    L = len(x)

    return pd.Series({
        feature: func(x=x, x_ac=x_ac, delta_t=delta_t, L=L, fs=fs)
        for feature, func in feature_functions.items()
        if feature not in excluded_features
    })

def dfFeatures(df, sensors, fs):
    features_list = []
    for sensor in sensors:
        stats_df = (
            df.groupby(['label', 'series_id'])[sensor]
            .apply(lambda x: signal_statistic(x, sensor_name=get_sensor_type(sensor),fs=fs, Norm=True))
            .unstack()
            .reset_index()
        )
        stats_df = stats_df.add_prefix(f"{sensor}_")
        stats_df.rename(columns={f"{sensor}_label": "label", f"{sensor}_series_id": "series_id"}, inplace=True)
        features_list.append(stats_df)

    features_df = features_list[0]
    for other_df in features_list[1:]:
        features_df = pd.merge(features_df, other_df, on=["label", "series_id"])
    return features_df

def extract_features(folder_output: str):
    base_dir = os.path.abspath(os.getcwd())
    folder = os.path.join(base_dir, folder_output, "dataset")
    output_folder = os.path.join(base_dir, folder_output, "dataset_features")

    os.makedirs(output_folder, exist_ok=True)

    adc_path = os.path.join(folder, "emg_dataset.csv")
    imu_path = os.path.join(folder, "imu_dataset.csv")

    output_adc_path = os.path.join(output_folder, "emg_features_dataset.csv")
    output_imu_path = os.path.join(output_folder, "imu_features_dataset.csv")
    output_df_path = os.path.join(output_folder, "features_dataset.csv")

    # === IMU ===
    fs=208
    df_imu = pd.read_csv(imu_path)
    df_imu["acc_vector"] = np.sqrt(df_imu["acc_x"]**2 + df_imu["acc_y"]**2 + df_imu["acc_z"]**2)
    df_imu["gyr_vector"] = np.sqrt(df_imu["gyr_x"]**2 + df_imu["gyr_y"]**2 + df_imu["gyr_z"]**2)
    df_imu_features = df_imu.drop(columns=["timestamp", "pkt_time_ns", "time_rel"])
    sensors_imu = [col for col in df_imu_features.columns if col not in ["label", "series_id"]]
    df_imu_features = dfFeatures(df_imu_features, sensors_imu, fs)
    df_imu_features.to_csv(output_imu_path, index=False)

    # === EMG ===
    fs=1000
    df_adc = pd.read_csv(adc_path)
    df_adc_features = df_adc.drop(columns=["timestamp", "pkt_time_ns", "time_rel"])
    sensors_adc = [col for col in df_adc_features.columns if col not in ["label", "series_id"]]
    df_adc_features = dfFeatures(df_adc_features, sensors_adc, fs)
    df_adc_features.to_csv(output_adc_path, index=False)

    # === MERGE ===
    df_features = pd.merge(df_imu_features, df_adc_features, on=["label", "series_id"])
    df_features.to_csv(output_df_path, index=False)

    # === Pickle ===
    with open(os.path.join(output_folder, "features_dataset.pk"), "wb") as f:
        pickle.dump(df_features, f)

    print(f"[OK] Features salvate in {output_folder}")
    return df_features

