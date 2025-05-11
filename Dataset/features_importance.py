import pandas as pd
import joblib
import matplotlib.pyplot as plt
import os

# === Load dataset and model ===
df = pd.read_csv("Dataset/dataset/extracted_features.csv")
X = df.drop(columns=["label", "series_id"])
model = joblib.load("Dataset/models/random_forest_model.joblib")

# === Compute feature importance ===
importances = model.feature_importances_
importance_df = pd.DataFrame({
    "feature": X.columns,
    "importance": importances
})

# === Calculate percentages and cumulative importance ===
importance_df["percent"] = importance_df["importance"] * 100
importance_df = importance_df.sort_values(by="percent", ascending=False)
importance_df["cumulative"] = importance_df["percent"].cumsum()

# === Save to CSV ===
importance_path = "Dataset/dataset/feature_importance.csv"
importance_df.to_csv(importance_path, index=False)
print(f"Saved feature importances to: {importance_path}")
print(importance_df.head(10))

# === Optional: Plot all feature importances ===
plt.figure(figsize=(10, 20))
plt.barh(importance_df["feature"], importance_df["percent"])
plt.xlabel("Feature Importance (%)")
plt.title("Feature Importance (All Features)")
plt.tight_layout()

# === Save the plot ===
plot_path = "Dataset/dataset/feature_importance_plot.png"
plt.savefig(plot_path, dpi=300)
print(f"Saved importance plot to: {plot_path}")
plt.show()