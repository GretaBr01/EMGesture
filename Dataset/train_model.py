import pandas as pd
import joblib
import os
from sklearn.model_selection import train_test_split
from sklearn.ensemble import RandomForestClassifier
from sklearn.metrics import classification_report, confusion_matrix
import seaborn as sns
import matplotlib.pyplot as plt

# === Create a directory for the model ===
os.makedirs("Dataset/models", exist_ok=True)

# === Load the feature dataset ===
df = pd.read_csv("Dataset/datasetCreate/dataset_features/features_dataset.csv")

# === Filter out the 'notevent' class ===
df = df[df["label"] != "notevent"]

# === Split features and labels ===
X = df.drop(columns=["label", "series_id"])
y = df["label"]

# === Define a specific feature subsets ===
feature_sets = {
    "set1": [
        "emg0_rms_ac", "emg1_rms_ac", "emg2_rms_ac", "emg3_rms_ac",
        "acc_x_mean", "acc_y_mean", "acc_z_mean",
        "gyr_x_mean", "gyr_y_mean", "gyr_z_mean"
    ],
    "set2": [
        "emg0_rms_ac", "emg1_rms_ac",
        "acc_x_mean", "acc_y_mean", "acc_z_mean",
        "gyr_x_mean", "gyr_y_mean", "gyr_z_mean"
    ],
    "set3": [
        "emg0_rms_ac", "emg1_rms_ac", "emg2_rms_ac", "emg3_rms_ac",
        "acc_x_rms_ac", "acc_y_rms_ac", "acc_z_rms_ac",
        "gyr_x_rms_ac", "gyr_y_rms_ac", "gyr_z_rms_ac"
    ],
    "set4": [
        "emg0_rms_ac", "emg1_rms_ac",
        "acc_x_rms_ac", "acc_y_rms_ac", "acc_z_rms_ac",
        "gyr_x_rms_ac", "gyr_y_rms_ac", "gyr_z_rms_ac"
    ]
}

for name, features in feature_sets.items():
    print(f"\n=== Training model: {name} ===")

    # Select features and labels
    X = df[features]
    y = df["label"]

    # Train/test split
    X_train, X_test, y_train, y_test = train_test_split(
        X, y, test_size=0.3, random_state=42, stratify=y # stratify=y: keeps the class distribution the same in both train and test sets (important if your dataset is imbalanced)
    )

    # Train model
    clf = RandomForestClassifier(class_weight="balanced", random_state=42) # class_weight="balanced": automatically adjusts weights inversely proportional to class frequencies. This helps if your dataset has imbalanced classes.
    clf.fit(X_train, y_train)

    # Evaluate
    y_pred = clf.predict(X_test)
    print("=== Classification Report ===")
    print(classification_report(y_test, y_pred))

    # Save model
    model_filename = os.path.join("Dataset/models", f"random_forest_model_{name}.joblib")
    joblib.dump(clf, model_filename)
    print(f"Model saved to {model_filename}")