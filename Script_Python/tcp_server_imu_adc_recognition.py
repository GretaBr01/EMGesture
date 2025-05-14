import socketserver
import struct
import csv
import os
from datetime import datetime
import time
import subprocess
from modules_recognition_server.create_features import signal_statistic
import numpy as np

from modules_recognition_server.inference_model import infer

NAME_DIR = "dati_recognition"

# Config
IP = "192.168.5.1"
PORT = 6789

def myget_time():
    return datetime.now().strftime('%Y%m%d_%H%M%S')

def get_timestamp():
    return time.time_ns()

def log_message(log_file, message):
    log_file.write(f"{myget_time()} ns - {message}\n")
    log_file.flush()

def decode_imu_sample(data):
    timestamp = struct.unpack("<I", data[:4])[0]
    gyr_x, gyr_y, gyr_z = struct.unpack("<hhh", data[4:10])
    acc_x, acc_y, acc_z = struct.unpack("<hhh", data[10:16])
    return (timestamp, gyr_x, gyr_y, gyr_z, acc_x, acc_y, acc_z)

def decode_adc_sample(data):
    timestamp, packed_adc = struct.unpack(ADC_STRUCT_FORMAT, data)
    b = list(packed_adc)
    adc0 = b[0] | ((b[1] & 0x0F) << 8)
    adc1 = ((b[1] >> 4) & 0x0F) | (b[2] << 4)
    adc2 = b[3] | ((b[4] & 0x0F) << 8)
    adc3 = ((b[4] >> 4) & 0x0F) | (b[5] << 4)
    return (timestamp, adc0, adc1, adc2, adc3)



IMU_STRUCT_FORMAT = "<I6s6s"
ADC_STRUCT_FORMAT = "<I6s"
NUM_STRUCT_FORMAT  = "<I"

N_IMU_SAMPLE_PACKET = 12
N_ADC_SAMPLE_PACKET = 58

IMU_SAMPLE_SIZE = struct.calcsize(IMU_STRUCT_FORMAT)
ADC_SAMPLE_SIZE = struct.calcsize(ADC_STRUCT_FORMAT)
NUM_PKT_SIZE = struct.calcsize(NUM_STRUCT_FORMAT)

PAYLOAD_SIZE = (IMU_SAMPLE_SIZE * N_IMU_SAMPLE_PACKET) + (ADC_SAMPLE_SIZE * N_ADC_SAMPLE_PACKET)
TOTAL_PACKET_SIZE = NUM_PKT_SIZE + PAYLOAD_SIZE + NUM_PKT_SIZE

acc_buffer = []
gyr_buffer = []
emg_buffer = []


# # create csv file
# os.makedirs(NAME_DIR+"/dati_imu", exist_ok=True)
# os.makedirs(NAME_DIR+"/dati_adc", exist_ok=True)
# os.makedirs(NAME_DIR+"/packet_bits", exist_ok=True)
os.makedirs(NAME_DIR+"/logfile", exist_ok=True)

# t = myget_time()
# imu_file = open(f"{NAME_DIR}/dati_imu/imu_{t}.csv", "w", newline="")
# adc_file = open(f"{NAME_DIR}/dati_adc/adc_{t}.csv", "w", newline="")
# pkt_file = open(f"{NAME_DIR}/packet_bits/packet_bits_{t}.txt", "w")
log_file = open(f"{NAME_DIR}/logfile/log_{t}.log", "w")

# imu_writer = csv.writer(imu_file)
# adc_writer = csv.writer(adc_file)
# imu_writer.writerow(["timestamp", "gyr_x", "gyr_y", "gyr_z", "acc_x", "acc_y", "acc_z", "pkt_time_ns"])
# adc_writer.writerow(["timestamp", "adc0", "adc1", "adc2", "adc3", "pkt_time_ns"])


# Buffer IMU
acc_x_buffer, acc_y_buffer, acc_z_buffer = [], [], []
gyr_x_buffer, gyr_y_buffer, gyr_z_buffer = [], [], []

# Buffer EMG (4 canali)
emg_0_buffer, emg_1_buffer, emg_2_buffer, emg_3_buffer = [], [], [], []

class MyTCPHandler(socketserver.BaseRequestHandler):
    def handle(self):
        buffer = b""
        log_message(log_file, f"Connessione da {self.client_address[0]}")

        try:
            while True:
                chunk =  self.request.recv(NUM_PKT_SIZE*2)
                if not chunk:
                    log_message(log_file, "Pacchetto vuoto, connessione interrotta.")
                    print(" - Connessione interrotta - ")
                    break

                buffer += chunk

                while len(buffer) >= TOTAL_PACKET_SIZE:
                    block = buffer[:TOTAL_PACKET_SIZE]  # packet

                    num_pkt = struct.unpack(NUM_STRUCT_FORMAT, block[:NUM_PKT_SIZE])[0]
                    num_pkt_neg = struct.unpack(NUM_STRUCT_FORMAT, block[-NUM_PKT_SIZE:])[0]

                    if num_pkt_neg != (~num_pkt & 0xFFFFFFFF):
                        log_message(log_file, f"Pacchetto scartato: num_pkt={num_pkt}, negato={num_pkt_neg}")
                        buffer = buffer[1:]
                        continue
                    
                    if num_pkt != 0 :
                        packet_data = block[NUM_PKT_SIZE:-NUM_PKT_SIZE]  # payload
                        buffer = buffer[TOTAL_PACKET_SIZE:]  # buffer rimanente

                        bit_string = ''.join(f"{byte:08b}" for byte in packet_data)
                        pkt_file.write(bit_string + '\n')
                        pkt_file.flush()

                        time_pkt = get_timestamp()

                        # IMU decoding
                        try:
                            for i in range(N_IMU_SAMPLE_PACKET):
                                offset = i * IMU_SAMPLE_SIZE
                                _, gx, gy, gz, ax, ay, az = decode_imu_sample(packet_data[offset:offset + IMU_SAMPLE_SIZE])
                                gyr_x_buffer.append(gx)
                                gyr_y_buffer.append(gy)
                                gyr_z_buffer.append(gz)
                                acc_x_buffer.append(ax)
                                acc_y_buffer.append(ay)
                                acc_z_buffer.append(az)
                        except Exception as e:
                            log_message(log_file, f"Errore decoding IMU: {e}")

                        # ADC decoding
                        try:
                            adc_offset_start = N_IMU_SAMPLE_PACKET * IMU_SAMPLE_SIZE
                            for i in range(N_ADC_SAMPLE_PACKET):
                                offset = adc_offset_start + i * ADC_SAMPLE_SIZE
                                _, adc0, adc1, adc2, adc3 = decode_adc_sample(packet_data[offset:offset + ADC_SAMPLE_SIZE])
                                emg_0_buffer.append(adc0)
                                emg_1_buffer.append(adc1)
                                emg_2_buffer.append(adc2)
                                emg_3_buffer.append(adc3)
                        except Exception as e:
                            log_message(log_file, f"Errore decoding ADC: {e}")

                        log_message(log_file, f"Pacchetto valido ricevuto. Size={len(packet_data)}, num_pkt={num_pkt}, t_Rpi={time_pkt}, t_IMU={time_imu}, t_ADC={time_adc}")
                        print(f"Pacchetto valido ricevuto. Size={len(packet_data)}, num_pkt={num_pkt}")
                    else:
                        # IMU
                        features_acc = {
                            "acc_x": signal_statistic(np.array(acc_x_buffer), "acc"),
                            "acc_y": signal_statistic(np.array(acc_y_buffer), "acc"),
                            "acc_z": signal_statistic(np.array(acc_z_buffer), "acc"),
                        }

                        features_gyr = {
                            "gyr_x": signal_statistic(np.array(gyr_x_buffer), "gyr"),
                            "gyr_y": signal_statistic(np.array(gyr_y_buffer), "gyr"),
                            "gyr_z": signal_statistic(np.array(gyr_z_buffer), "gyr"),
                        }

                        # EMG (con filtro)
                        features_emg = {
                            "emg_0": signal_statistic(np.array(emg_0_buffer), "emg", apply_filter=True),
                            "emg_1": signal_statistic(np.array(emg_1_buffer), "emg", apply_filter=True),
                            "emg_2": signal_statistic(np.array(emg_2_buffer), "emg", apply_filter=True),
                            "emg_3": signal_statistic(np.array(emg_3_buffer), "emg", apply_filter=True),
                        }

                        gesture_class = infer(features_acc, features_gyr, features_emg)

                        print(f">>> Gesto riconosciuto: {gesture_class}")
                        log_message(log_file, f">>> Gesto riconosciuto: {gesture_class}")

                        log_message(log_file, f"Fine acquisizione gesto. Size={len(packet_data)}")
                        print(f"Fine acquisizione gesto. Size={len(packet_data)}, num_pkt={num_pkt}")

        except Exception as e:
            log_message(log_file, f"Errore generico: {e}")

if __name__ == "__main__":
    try:
        script_dir = os.path.dirname(os.path.abspath(__file__))
        sh_path = os.path.join(script_dir, "disconnettiDispWifi.sh")
        subprocess.run([sh_path], check=True)
        log_message(log_file, "Script disconnettiDispWifi.sh eseguito con successo.")
        print("Script disconnettiDispWifi.sh eseguito con successo.")
    except subprocess.CalledProcessError as e:
        log_message(log_file, f"Errore nell'esecuzione dello script: {e}")
        print(f"Errore nell'esecuzione dello script: {e}")
    except FileNotFoundError:
        log_message(log_file, "Script disconnettiDispWifi.sh non trovato.")
        print("Script disconnettiDispWifi.sh non trovato.")

    with socketserver.TCPServer((IP, PORT), MyTCPHandler) as server:
        log_message(log_file, f"Server avviato su {IP}:{PORT}")
        try:
            server.serve_forever()
        except KeyboardInterrupt:
            log_message(log_file, "Interruzione manuale (Ctrl+C)")
        finally:
            log_file.close()
            server.server_close()
