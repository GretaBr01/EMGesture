import socketserver
import struct
import csv
import os
from datetime import datetime
import time

def myget_time():
    return datetime.now().strftime('%Y%m%d_%H%M%S')

def get_timestamp():
    return time.time_ns()

def log_message(log_file, message):
    log_file.write(f"{myget_time()} ns - {message}\n")
    log_file.flush()

def decode_imu_sample(data):
    timestamp = struct.unpack("<I", data[:4])[0]
    gyr_x, gyr_y, gyr_z = struct.unpack("<HHH", data[4:10])
    acc_x, acc_y, acc_z = struct.unpack("<HHH", data[10:16])
    return (timestamp, gyr_x, gyr_y, gyr_z, acc_x, acc_y, acc_z)

def decode_adc_sample(data):
    timestamp, packed_adc = struct.unpack(ADC_STRUCT_FORMAT, data)
    b = list(packed_adc)
    adc0 = b[0] | ((b[1] & 0x0F) << 8)
    adc1 = ((b[1] >> 4) & 0x0F) | (b[2] << 4)
    adc2 = b[3] | ((b[4] & 0x0F) << 8)
    adc3 = ((b[4] >> 4) & 0x0F) | (b[5] << 4)
    return (timestamp, adc0, adc1, adc2, adc3)

# Config
IP = "192.168.5.1"
PORT = 6789

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

# Output dirs
os.makedirs("datiServerTCP/dati_imu", exist_ok=True)
os.makedirs("datiServerTCP/dati_adc", exist_ok=True)
os.makedirs("datiServerTCP/packet_bits", exist_ok=True)
os.makedirs("datiServerTCP/logfile", exist_ok=True)

t = myget_time()
imu_file = open(f"datiServerTCP/dati_imu/imu_{t}.csv", "w", newline="")
adc_file = open(f"datiServerTCP/dati_adc/adc_{t}.csv", "w", newline="")
pkt_file = open(f"datiServerTCP/packet_bits/packet_bits_{t}.txt", "w")
log_file = open(f"datiServerTCP/logfile/log_{t}.log", "w")

imu_writer = csv.writer(imu_file)
adc_writer = csv.writer(adc_file)
imu_writer.writerow(["timestamp", "gyr_x", "gyr_y", "gyr_z", "acc_x", "acc_y", "acc_z", "pkt_time_ns"])
adc_writer.writerow(["timestamp", "adc0", "adc1", "adc2", "adc3", "pkt_time_ns"])

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
                            imu_data = decode_imu_sample(packet_data[offset:offset + IMU_SAMPLE_SIZE])
                            imu_writer.writerow([*imu_data, time_pkt])
                            if i == 0:
                                time_imu = imu_data[0]
                    except Exception as e:
                        log_message(log_file, f"Errore decoding IMU: {e}")

                    # ADC decoding
                    try:
                        adc_offset_start = N_IMU_SAMPLE_PACKET * IMU_SAMPLE_SIZE
                        for i in range(N_ADC_SAMPLE_PACKET):
                            offset = adc_offset_start + i * ADC_SAMPLE_SIZE
                            adc_data = decode_adc_sample(packet_data[offset:offset + ADC_SAMPLE_SIZE])
                            adc_writer.writerow([*adc_data, time_pkt])
                            if i == 0:
                                time_adc = adc_data[0]
                    except Exception as e:
                        log_message(log_file, f"Errore decoding ADC: {e}")

                    log_message(log_file, f"Pacchetto valido ricevuto. Size={len(packet_data)}, num_pkt={num_pkt}, t_Rpi={time_pkt}, t_IMU={time_imu}, t_ADC={time_adc}")
                    print(f"Pacchetto valido ricevuto. Size={len(packet_data)}, num_pkt={num_pkt}")

        except Exception as e:
            log_message(log_file, f"Errore generico: {e}")

if __name__ == "__main__":
    with socketserver.TCPServer((IP, PORT), MyTCPHandler) as server:
        log_message(log_file, f"Server avviato su {IP}:{PORT}")
        try:
            server.serve_forever()
        except KeyboardInterrupt:
            log_message(log_file, "Interruzione manuale (Ctrl+C)")
        finally:
            log_file.close()
            adc_file.close()
            imu_file.close()
            pkt_file.close()
            server.server_close()
