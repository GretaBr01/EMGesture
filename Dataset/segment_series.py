import pandas as pd
import matplotlib.pyplot as plt
import os

folder_path = ["./data/adc", "./data/imu"]
output_folder = "./data_numbered_series"


os.makedirs(output_folder, exist_ok=True)

ADC_threshold_time_ns=800*1000
IMU_threshold_time_ns=800*1000/25

adc_th=10*1000
imu_th=10*1000/25

for folder_name in folder_path:
    for root, dirs, files in os.walk(folder_name):
        for file_name in files:
            if file_name.endswith(".csv") and "_labeled" not in file_name:
                file_path = os.path.join(root, file_name)
                lower_name = file_name.lower()

                # Determina la soglia in base alla presenza di "adc" o "imu"
                if "adc" in lower_name:
                    threshold = ADC_threshold_time_ns
                    th=adc_th
                elif "imu" in lower_name:
                    threshold = IMU_threshold_time_ns
                    th=imu_th
                else:
                    print(f"[Ignorato] Tipo non riconosciuto nel file: {file_name}")
                    continue

                # Nome del file in output
                output_file_name = file_name[:-4] + "_labeled.csv"
                output_file_path = os.path.join(output_folder, output_file_name)

                # Salta se il file esiste già
                if os.path.exists(output_file_path):
                    print(f"[Già presente] {output_file_name} - ignorato.")
                    continue

                # Carica ed elabora
                df = pd.read_csv(file_path)

                # Calcola la differenza di timestamp con il precedente e il successivo
                df['prev_diff'] = df['timestamp'] - df['timestamp'].shift(1)
                df['next_diff'] = df['timestamp'].shift(-1) - df['timestamp']

                # Rimuove i punti isolati: sia prev_diff che next_diff > threshold
                df = df[~((df['prev_diff'] > th) & (df['next_diff'] > th))].reset_index(drop=True)

                # Segmentazione
                df['timestamp_diff'] = df['timestamp'].diff()
                df['series_id'] = (df['timestamp_diff'] > threshold).cumsum()

                # Pulisce colonne temporanee
                df.drop(columns=['timestamp_diff', 'prev_diff', 'next_diff'], inplace=True)

                df.to_csv(output_file_path, index=False)

                print(f"[OK] File processato e salvato in: {output_file_path}")