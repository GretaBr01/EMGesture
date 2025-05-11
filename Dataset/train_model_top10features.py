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

# === Load datasets ===
df = pd.read_csv("Dataset/dataset/extracted_features.csv")
importance_df = pd.read_csv("Dataset/dataset/feature_importance.csv")

# === Select top 10 features ===
top_features = importance_df["feature"].iloc[:3].tolist()

# === Prepare data ===
X = df[top_features]
y = df["label"]

X_train, X_test, y_train, y_test = train_test_split(
    X, y, test_size=0.2, stratify=y, random_state=42
)

# === Train model ===
clf = RandomForestClassifier(class_weight="balanced", random_state=42)
clf.fit(X_train, y_train)
y_pred = clf.predict(X_test)

# === Evaluate ===
print("=== Classification Report (Top 10 Features) ===")
print(classification_report(y_test, y_pred))

# === Confusion matrix ===
cm = confusion_matrix(y_test, y_pred, labels=clf.classes_)
sns.heatmap(cm, annot=True, fmt="d", xticklabels=clf.classes_, yticklabels=clf.classes_)
plt.xlabel("Predicted")
plt.ylabel("True")
plt.title("Confusion Matrix")
plt.tight_layout()
plt.show()

# === Save the model ===
model_filename = os.path.join("Dataset/models", "random_forest_top10features.joblib")
joblib.dump(clf, model_filename)
print(f"Model saved to {model_filename}")