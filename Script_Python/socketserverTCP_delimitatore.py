import socketserver
import struct
import csv
import os
from datetime import datetime
import time

# #
# AGGUNGERE CRC pacchetto, test su raspberry per qualità della connessione
# #


# Config
IP = "192.168.5.1"
PORT = 6789

IMU_STRUCT_FORMAT = "<I6s6s"
ADC_STRUCT_FORMAT = "<I6s"
NUM_STRUCT_FORMAT  = "<I"

N_IMU_SAMPLE_PACKET = 18    #18/208 = 0.08653
N_ADC_SAMPLE_PACKET = 87    #87/1000 = 0.087

IMU_SAMPLE_SIZE = struct.calcsize(IMU_STRUCT_FORMAT)
ADC_SAMPLE_SIZE = struct.calcsize(ADC_STRUCT_FORMAT)
NUM_PKT_SIZE = struct.calcsize(NUM_STRUCT_FORMAT)
CRC_SIZE = 0
PACKET_SIZE = NUM_PKT_SIZE + (IMU_SAMPLE_SIZE * N_IMU_SAMPLE_PACKET) + (ADC_SAMPLE_SIZE * N_ADC_SAMPLE_PACKET) + CRC_SIZE

START_END_DELIMITER = b"zutss"

# Output dirs
os.makedirs("datiServerTCP/dati_csv/all_raw_bits", exist_ok=True)
os.makedirs("datiServerTCP/dati_csv/packet_bits", exist_ok=True)
os.makedirs("datiServerTCP/logfile", exist_ok=True)

def get_time():
    return datetime.now().strftime('%Y%m%d_%H%M%S')

def get_timestamp():
    return time.time_ns()

def log_message(log_file, message):
    log_file.write(f"{get_time()} ns - {message}\n")
    log_file.flush()

t = get_time()
bit_file = open(f"datiServerTCP/dati_csv/all_raw_bits/raw_bits_{t}.txt", "w")
pkt_file = open(f"datiServerTCP/dati_csv/packet_bits/packet_bits_{t}.txt", "w")
log_file = open(f"datiServerTCP/logfile/log_{t}.log", "w")

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

class MyTCPHandler(socketserver.BaseRequestHandler):
    def handle(self):
        buffer = b""
        log_message(log_file, f"Connessione da {self.client_address[0]}")

        try:
            while True:
                chunk = self.request.recv(1024)
                bit_string = ''.join(f"{byte:08b}" for byte in chunk)
                bit_file.write(bit_string + '\n')
                bit_file.flush()
                if not chunk:
                    log_message(log_file, "PAcchetto 0, Connessione interrotta.")
                    break

                buffer += chunk

                while len(buffer)>len(START_END_DELIMITER):
                    start_idx = buffer.find(START_END_DELIMITER)
                    if start_idx == -1:
                        break
                    end_idx = buffer.find(START_END_DELIMITER, start_idx + len(START_END_DELIMITER))
                    if end_idx == -1:
                        # Delimitatore finale non ancora ricevuto
                        break

                    packet_data = buffer[start_idx + len(START_END_DELIMITER):end_idx]
                    #buffer = buffer[end_idx + len(START_END_DELIMITER):]

                    if len(packet_data) == 0:
                        start_idx = end_idx
                        end_idx = buffer.find(START_END_DELIMITER, start_idx + len(START_END_DELIMITER))
                        if end_idx == -1:
                            # Delimitatore finale non ancora ricevuto
                            break                  

                    if len(packet_data) != PACKET_SIZE:
                        buffer = buffer[end_idx:]
                        log_message(log_file, f"Pacchetto scartato: dimensione errata ({len(packet_data)})")
                        continue
                    
                    packet_data = buffer[start_idx + len(START_END_DELIMITER):end_idx] 
                    buffer = buffer[end_idx + len(START_END_DELIMITER):]

                    bit_string = ''.join(f"{byte:08b}" for byte in packet_data)
                    pkt_file.write(bit_string + '\n')
                    pkt_file.flush()

                    time_pkt = get_timestamp()

                    # NUM_PKT decoding                    
                    num_pkt = struct.unpack(NUM_STRUCT_FORMAT, packet_data[:NUM_PKT_SIZE])[0]

                    offset_payload = NUM_PKT_SIZE
                    # IMU decoding
                    try:
                        for i in range(N_IMU_SAMPLE_PACKET):
                            offset = i * IMU_SAMPLE_SIZE + offset_payload
                            imu_data = decode_imu_sample(packet_data[offset:offset + IMU_SAMPLE_SIZE])
                            if i == 0:
                                time_imu = imu_data[0]
                    except Exception as e:
                        log_message(log_file, f"Errore decoding IMU: {e}")

                    # ADC decoding
                    try:
                        adc_offset_start = N_IMU_SAMPLE_PACKET * IMU_SAMPLE_SIZE + offset_payload
                        for i in range(N_ADC_SAMPLE_PACKET):
                            offset = adc_offset_start + i * ADC_SAMPLE_SIZE
                            adc_data = decode_adc_sample(packet_data[offset:offset + ADC_SAMPLE_SIZE])
                            if i == 0:
                                time_adc = adc_data[0]
                    except Exception as e:
                        log_message(log_file, f"Errore decoding ADC: {e}")

                    log_message(log_file, f"Pacchetto valido ricevuto. Size={len(packet_data)}, num_pkt={num_pkt}, t_Rpi={time_pkt}, t_IMU={time_imu}, t_ADC={time_adc}")

        except ConnectionResetError:
            log_message(log_file, f"Connessione resettata dal client {self.client_address[0]}")
        except BrokenPipeError:
            log_message(log_file, f"Broken pipe: il client {self.client_address[0]} ha chiuso la connessione.")
        except OSError as e:
            log_message(log_file, f"OSError: possibile crash del client {self.client_address[0]} - {e}")
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
            bit_file.close()
            log_file.close()
