import pandas as pd
import matplotlib.pyplot as plt

file_name = "adc_20250420_005238"

ADC_threshold_time_ns=800*1000
IMU_threshold_time_ns=800*1000/25

prefix = file_name[:3].lower()

# Imposta la soglia in base al prefisso
if prefix == "adc":
    threshold = ADC_threshold_time_ns
elif prefix == "imu":
    threshold = IMU_threshold_time_ns
else:
    raise ValueError("Prefisso file non riconosciuto: usa 'adc' o 'imu'.")


# Carica il file CSV
df = pd.read_csv(file_name+".csv")
# Calcoliamo la differenza tra i timestamp consecutivi
df['timestamp_diff'] = df['timestamp_device'].diff()

# Etichettiamo ogni serie: quando la differenza è > 800, incrementiamo l'etichetta
df['series_id'] = (df['timestamp_diff'] > threshold ).cumsum()

# Rimuoviamo la colonna temporanea della differenza
df.drop(columns=['timestamp_diff'], inplace=True)

# Salviamo il nuovo file con l'etichetta delle serie
output_path = file_name+"_labeled.csv"
df.to_csv(output_path, index=False)

output_path
