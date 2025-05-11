import joblib
import numpy as np

# Load model
model = joblib.load("Dataset/models/random_forest_top10features.joblib")

# Example input: 1 sample with 10 features
# Replace with real sensor input when ready
sample = np.array([[0.12, -0.5, 1.1, ..., 0.03]])  # shape: (1, 10)
prediction = model.predict(sample)

print(f"Predicted label: {prediction[0]}")