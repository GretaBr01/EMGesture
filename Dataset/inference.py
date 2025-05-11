import joblib
import numpy as np
import pandas as pd
import psutil
import os

def print_ram_usage():
    ram = psutil.Process(os.getpid()).memory_info().rss / (1024 ** 2)
    print(f"RAM usage: {ram:.2f} MB")

# === RAM before model load ===
print("Before loading model:")
print_ram_usage()

# === Load model ===
model = joblib.load("Dataset/models/random_forest_top10features.joblib")

# === RAM after model load ===
print("After loading model:")
print_ram_usage()

# === Load feature names the model was trained on ===
importance_df = pd.read_csv("Dataset/dataset/feature_importance.csv")
top_features = importance_df["feature"].iloc[:3].tolist()

# === Load full dataset just to compute realistic mean values ===
df = pd.read_csv("Dataset/dataset/extracted_features.csv")
X = df[top_features]

# === Predict ===
prediction = model.predict(X.iloc[[0]])  # Returns a DataFrame (1 row, all columns))
print(f"Predicted label: {prediction[0]}")