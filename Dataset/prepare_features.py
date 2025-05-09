import pandas as pd
import os
import numpy as np
from scipy.signal import butter, filtfilt

# === Load datasets ===
emg_path = os.path.join("Dataset", "dataset", "emg_dataset.csv")
imu_path = os.path.join("Dataset", "dataset", "imu_dataset.csv")
output_path = os.path.join("Dataset", "dataset", "extracted_features.csv")

emg_df = pd.read_csv(emg_path)
imu_df = pd.read_csv(imu_path)

# === Drop unnecessary columns ===
emg_df.drop(columns=["timestamp", "pkt_time_ns"], inplace=True)
imu_df.drop(columns=["timestamp", "pkt_time_ns"], inplace=True)

# === Fix EMG time units ===
emg_df["time_rel"] = emg_df["time_rel"].astype(float) / 1000.0

# === Define columns for feature extraction ===
emg_cols = ["emg0", "emg1", "emg2", "emg3"]
imu_cols = ["gyr_x", "gyr_y", "gyr_z", "acc_x", "acc_y", "acc_z"]

# === Signal filtering function ===
def butter_bandpass_filter(data, lowcut, highcut, fs, order=4):
    nyq = 0.5 * fs
    low = lowcut / nyq
    high = highcut / nyq
    b, a = butter(order, [low, high], btype='band')
    return filtfilt(b, a, data)

# === Feature extraction with optional filtering ===
def extract_features(df, columns, prefix, fs=1000, filter_emg=False):
    features = {}
    for col in columns:
        signal = df[col].values.astype(float)

        # Apply bandpass filter to EMG if requested
        if filter_emg and prefix == "emg":
            signal = butter_bandpass_filter(signal, lowcut=20, highcut=450, fs=fs)

        # Standard statistical features
        features[f"{prefix}_{col}_mean"] = np.mean(signal)
        features[f"{prefix}_{col}_std"] = np.std(signal)
        features[f"{prefix}_{col}_min"] = np.min(signal)
        features[f"{prefix}_{col}_max"] = np.max(signal)
        features[f"{prefix}_{col}_median"] = np.median(signal)
        features[f"{prefix}_{col}_energy"] = np.sum(signal ** 2) / len(signal)

    return features

# === Process all series_id ===
all_series_ids = sorted(emg_df["series_id"].unique())
feature_rows = []

for sid in all_series_ids:
    emg_series = emg_df[emg_df["series_id"] == sid]
    imu_series = imu_df[imu_df["series_id"] == sid]

    # Extract and combine features
    emg_features = extract_features(emg_series, emg_cols, prefix="emg", fs=1000, filter_emg=True)
    imu_features = extract_features(imu_series, imu_cols, prefix="imu", fs=208, filter_emg=False)
    combined_features = {**emg_features, **imu_features}
    
    # Add label and series_id for tracking
    combined_features["label"] = emg_series["label"].iloc[0]
    combined_features["series_id"] = sid

    feature_rows.append(combined_features)

# === Build final DataFrame ===
features_df = pd.DataFrame(feature_rows)

# === Save or inspect ===
print("Feature extraction complete. Example:")
print(features_df.head())
print(f"Total movements (rows): {len(features_df)}")
print(f"Total features per movement: {features_df.shape[1] - 2} (excluding label and series_id)")
# Check for class imbalance
print(features_df["label"].value_counts())
print("\nClass distribution (percentages):")
print(features_df["label"].value_counts(normalize=True) * 100)

# Optional: Save to CSV
features_df.to_csv(output_path, index=False)