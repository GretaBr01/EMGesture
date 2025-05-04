import pandas as pd
import os

data_folder = "./data_numbered_series"
output_folder = "dataset"
output_adc_path = f"./{output_folder}/adc_dataset.csv"
output_imu_path = f"./{output_folder}/imu_dataset.csv"

os.makedirs(output_folder, exist_ok=True)

dfs_adc = []
dfs_imu = []
series_counter_adc = 0
series_counter_imu = 0

for file_name in os.listdir(data_folder):
    if not file_name.endswith("_labeled.csv"):
        continue  # Salta i file non labeled

    full_path = os.path.join(data_folder, file_name)

    try:
        df = pd.read_csv(full_path)

        # Estrai il label e il tipo di sensore dal nome del file
        parts = file_name.lower().split("_")
        label = parts[0]
        sensor_type = parts[1]

        df["label"] = label

        # Rinumerazione dei series_id separata per tipo di sensore
        old_series = df["series_id"].unique()

        if sensor_type == "adc":
            series_map = {
                old: new for old, new in zip(sorted(old_series), range(series_counter_adc, series_counter_adc + len(old_series)))
            }
            df["series_id"] = df["series_id"].map(series_map)
            series_counter_adc += len(old_series)
            dfs_adc.append(df)

        elif sensor_type == "imu":
            series_map = {
                old: new for old, new in zip(sorted(old_series), range(series_counter_imu, series_counter_imu + len(old_series)))
            }
            df["series_id"] = df["series_id"].map(series_map)
            series_counter_imu += len(old_series)
            dfs_imu.append(df)

        else:
            print(f"[!] Sensore non riconosciuto nel file: {file_name}")

        print(f"[*] File caricato: {file_name}, serie mappate: {len(old_series)}")

    except Exception as e:
        print(f"[Errore] File: {full_path} - {e}")

# Concatenazione e salvataggio
if dfs_adc:
    df_adc = pd.concat(dfs_adc, ignore_index=True)
    df_adc.to_csv(output_adc_path, index=False)
    print(f"[✓] Dataset ADC salvato in {output_adc_path}")
else:
    print("[!] Nessun file ADC trovato o valido.")

if dfs_imu:
    df_imu = pd.concat(dfs_imu, ignore_index=True)
    df_imu.to_csv(output_imu_path, index=False)
    print(f"[✓] Dataset IMU salvato in {output_imu_path}")
else:
    print("[!] Nessun file IMU trovato o valido.")
