import os
import pandas as pd
import numpy as np
from scipy.stats import skew, kurtosis
import pickle

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
    'kurtosis': lambda x, **kwargs: kurtosis(x)
}

excluded_features = []

# === FUNZIONI DI SUPPORTO ===
def get_sensor_type(col_name):
    for sensor in SENSOR_DATA:
        if col_name.startswith(sensor):
            return sensor
    return None

def signal_statistic(x, sensor_name, Norm=True):
    if Norm:
        x = x / SENSOR_DATA[sensor_name]["full_scale"]

    delta_t = 1.0 / SENSOR_DATA[sensor_name]["frequenza_campionamento"]
    x_dc = np.mean(x)
    x_ac = x - x_dc
    L = len(x)

    return pd.Series({
        feature: func(x=x, x_ac=x_ac, delta_t=delta_t, L=L)
        for feature, func in feature_functions.items()
        if feature not in excluded_features
    })

def dfFeatures(df, sensors):
    features_list = []
    for sensor in sensors:
        stats_df = (
            df.groupby(['label', 'series_id'])[sensor]
            .apply(lambda x: signal_statistic(x, sensor_name=get_sensor_type(sensor), Norm=True))
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
    df_imu = pd.read_csv(imu_path)
    df_imu["acc_vector"] = np.sqrt(df_imu["acc_x"]**2 + df_imu["acc_y"]**2 + df_imu["acc_z"]**2)
    df_imu["gyr_vector"] = np.sqrt(df_imu["gyr_x"]**2 + df_imu["gyr_y"]**2 + df_imu["gyr_z"]**2)
    df_imu_features = df_imu.drop(columns=["timestamp", "pkt_time_ns", "time_rel"])
    sensors_imu = [col for col in df_imu_features.columns if col not in ["label", "series_id"]]
    df_imu_features = dfFeatures(df_imu_features, sensors_imu)
    df_imu_features.to_csv(output_imu_path, index=False)

    # === EMG ===
    df_adc = pd.read_csv(adc_path)
    df_adc_features = df_adc.drop(columns=["timestamp", "pkt_time_ns", "time_rel"])
    sensors_adc = [col for col in df_adc_features.columns if col not in ["label", "series_id"]]
    df_adc_features = dfFeatures(df_adc_features, sensors_adc)
    df_adc_features.to_csv(output_adc_path, index=False)

    # === MERGE ===
    df_features = pd.merge(df_imu_features, df_adc_features, on=["label", "series_id"])
    df_features.to_csv(output_df_path, index=False)

    # === Pickle ===
    with open(os.path.join(output_folder, "features_dataset.pk"), "wb") as f:
        pickle.dump(df_features, f)

    print(f"[OK] Features salvate in {output_folder}")
    return df_features

