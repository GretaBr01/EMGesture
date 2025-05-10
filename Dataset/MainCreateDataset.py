from modules.segment_series import segment_raw_data
from modules.create_df import merge_labeled_series
from modules.create_df_features import extract_features

folder_output = "datasetCreate"
# 1. Segmenta i dati
segment_raw_data(folder_output)

# 2. Concatena tutti i dati
merge_labeled_series(folder_output)

# 3. Estrai le feature
extract_features(folder_output)
