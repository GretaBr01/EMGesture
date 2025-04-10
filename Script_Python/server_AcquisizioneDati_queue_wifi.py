import socket
import struct
import csv
import os
from datetime import datetime
import time

# Config
IP = "192.168.5.1"
PORT = 6789

IMU_STRUCT_FORMAT = "<I6s6s"       # timestamp (4) + gyr (6) + acc (6)
ADC_STRUCT_FORMAT = "<I6s"          # timestamp (4) + adc[6] (6 bytes packed)

# Packet sizes
N_IMU_SAMPLE_PACKET = 26
N_ADC_SAMPLE_PACKET = 125

IMU_SAMPLE_SIZE = 4 + 6 + 6
ADC_SAMPLE_SIZE = 4 + 6

PACKET_SIZE = (IMU_SAMPLE_SIZE * N_IMU_SAMPLE_PACKET) + (ADC_SAMPLE_SIZE * N_ADC_SAMPLE_PACKET)

# Cartella di output
os.makedirs("dati_csv", exist_ok=True)
imu_file = open(f"dati_csv/imu_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv", "w", newline="")
adc_file = open(f"dati_csv/adc_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv", "w", newline="")

imu_writer = csv.writer(imu_file)
adc_writer = csv.writer(adc_file)

imu_writer.writerow(["timestamp", "gyr_x", "gyr_y", "gyr_z", "acc_x", "acc_y", "acc_z"])
adc_writer.writerow(["timestamp", "adc0", "adc1", "adc2", "adc3"])


def decode_imu_sample(data):
    timestamp_bytes, gyr_bytes, acc_bytes = struct.unpack(IMU_STRUCT_FORMAT, data)
    gyr = struct.unpack("<hhh", gyr_bytes)
    acc = struct.unpack("<hhh", acc_bytes)
    return (timestamp_bytes, *gyr, *acc)

def decode_adc_sample(data):
    timestamp, packed_adc = struct.unpack(ADC_STRUCT_FORMAT, data)
    # Unpacking 6 bytes → 4 valori ADC da 12 bit
    b = list(packed_adc)
    adc0 = b[0] | ((b[1] & 0x0F) << 8)
    adc1 = ((b[1] >> 4) & 0x0F) | (b[2] << 4)
    adc2 = b[3] | ((b[4] & 0x0F) << 8)
    adc3 = ((b[4] >> 4) & 0x0F) | (b[5] << 4)
    return (timestamp, adc0, adc1, adc2, adc3)



def start_server():
    print(f"Server in ascolto su {IP}:{PORT}")

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind((IP, PORT))
        s.listen(1)
        conn, addr = s.accept()
        print(f"Connected by {addr}")

        buffer = b""
        try:             
            while True:
                packet = bytearray()
                while len(packet) < PACKET_SIZE:
                    chunk = conn.recv(min(PACKET_SIZE, PACKET_SIZE - len(packet)))
                    if not chunk:
                        break
                    packet.extend(chunk)

                print(len(packet))
                for i in range(N_IMU_SAMPLE_PACKET):
                        offset = i * IMU_SAMPLE_SIZE
                        imu_data = decode_imu_sample(packet[offset:offset + IMU_SAMPLE_SIZE])
                        imu_writer.writerow(imu_data)

                adc_offset_start = N_IMU_SAMPLE_PACKET * IMU_SAMPLE_SIZE
                for i in range(N_ADC_SAMPLE_PACKET):
                    offset = adc_offset_start + i * ADC_SAMPLE_SIZE
                    adc_data = decode_adc_sample(packet[offset:offset + ADC_SAMPLE_SIZE])
                    adc_writer.writerow(adc_data)

                print(time.time(),": Pacchetto ricevuto e salvato")

        except KeyboardInterrupt:
            print("Interrotto dall'utente")
        finally:
            imu_file.close()
            adc_file.close()
            print("File CSV chiusi")

if __name__ == "__main__":
    start_server()
