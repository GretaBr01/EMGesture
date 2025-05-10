def merge_labeled_series(folder_output: str):
  import pandas as pd
  import os

  base_dir = os.path.abspath(os.getcwd())

  data_folder = os.path.join(base_dir, folder_output, "data_numbered_series")
  output_folder = os.path.join(base_dir, folder_output, "dataset")

  output_adc_path = f"{output_folder}/emg_dataset.csv"
  output_imu_path = f"{output_folder}/imu_dataset.csv"

  os.makedirs(output_folder, exist_ok=True)
  print(data_folder)

  dfs_adc = []
  dfs_imu = []
  series_counter_emg = 0
  series_counter_imu = 0
  max_time_rel_imu=0
  max_time_rel_emg=0

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

          #rimuovo ultima serie che potrebbe essere corrotta (l'arduino si interrompe durante il movimento)
          max_series_id = df["series_id"].max()
          df = df[df["series_id"] != max_series_id]

          # Rinumerazione dei series_id separata per tipo di sensore
          old_series = df["series_id"].unique()

          if sensor_type == "emg":
              series_map = {
                  old: new for old, new in zip(sorted(old_series), range(series_counter_emg, series_counter_emg + len(old_series)))
              }
              df["series_id"] = df["series_id"].map(series_map)
              series_counter_emg += len(old_series)
              df["time_rel"]= df["time_rel"]+ max_time_rel_emg
              dfs_adc.append(df)
              max_time_rel_emg=max(df["time_rel"])

          elif sensor_type == "imu":
              series_map = {
                  old: new for old, new in zip(sorted(old_series), range(series_counter_imu, series_counter_imu + len(old_series)))
              }
              df["series_id"] = df["series_id"].map(series_map)
              series_counter_imu += len(old_series)
              df["time_rel"]= df["time_rel"]+ max_time_rel_imu
              dfs_imu.append(df)
              max_time_rel_imu=max(df["time_rel"])

          else:
              print(f"[!] Sensore non riconosciuto nel file: {file_name}")

          print(f"[*] File caricato: {file_name}, serie mappate: {len(old_series)}")

      except Exception as e:
          print(f"[Errore] File: {full_path} - {e}")

  # Concatenazione e salvataggio
  if dfs_adc:
      df_adc = pd.concat(dfs_adc, ignore_index=True)
      df_adc.to_csv(output_adc_path, index=False)
      print(f"[OK] Dataset ADC salvato in {output_adc_path}")
  else:
      print("[!] Nessun file ADC trovato o valido.")

  if dfs_imu:
      df_imu = pd.concat(dfs_imu, ignore_index=True)
      df_imu.to_csv(output_imu_path, index=False)
      print(f"[OK] Dataset IMU salvato in {output_imu_path}")
  else:
      print("[!] Nessun file IMU trovato o valido.")
