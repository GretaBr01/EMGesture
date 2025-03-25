import socket
import time
import csv
import os

def get_timestamp():
    return time.time_ns()

def log_message(file_log, message):
    timestamp = get_timestamp()
    file_log.write(f"{timestamp} ns - {message}\n")
    file_log.flush()

TIMEOUT = 30

IP = "192.168.5.1"
PORT = 6789

TEMPO = 21600 # tempo esecuzione in s = 6 ore
Bps = 18000  # byte inviati al secondo
TOT_BYTE = TEMPO*Bps
PACKETS_PER_SECOND = [5]
BUFFER_SIZES = [Bps // pps for pps in PACKETS_PER_SECOND]

LOG_FILE = "logTest_py.txt"
DATA_FILE = "dataset.txt"

folder_path = "Test_python"
os.makedirs(folder_path, exist_ok=True)

file_index = 1
while os.path.exists(f"./{folder_path}/test_wifi_{file_index}.csv"):
    file_index += 1

CSV_FILE = f"./{folder_path}/test_wifi_{file_index}.csv"

# Apertura file log
with open(LOG_FILE, mode="a") as file_log, open(DATA_FILE, mode="a") as file_data:
    # Avvia il server TCP
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server:
        server.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 10*1024*1024) #dimensione in byte, impostata a 5kB

        server.bind((IP, PORT))
        server.listen(1)
        print(f"Server in ascolto su {IP}:{PORT}")
        print(f"\n{PACKETS_PER_SECOND}")
        log_message(file_log, f"Server in ascolto su {IP}:{PORT}")

        # Apre il file CSV per la scrittura
        with open(CSV_FILE, mode="w", newline="") as file:
            writer = csv.writer(file)
            writer.writerow(["packet_number", "start_timestamp", "end_timestamp", "packet_size_byte", "packets_per_second"])

            for i, pps in enumerate(PACKETS_PER_SECOND):
                buffer_size = BUFFER_SIZES[i]
                print(f"\n{get_timestamp()} ns - In attesa di connessione (PACKETS_PER_SECOND = {pps}, BUFFER_SIZE = {buffer_size})")
                
                log_message(file_log, f"In attesa di connessione (PACKETS_PER_SECOND = {pps}, BUFFER_SIZE = {buffer_size})")

                conn, addr = server.accept()
                timeout_counter = 0
                # Connessione attiva
                with conn:
                    conn.settimeout(TIMEOUT) 
                    print(f"\n{get_timestamp()} ns - Connessione ricevuta da {addr}")
                    
                    log_message(file_log, f"Connessione ricevuta da {addr}")

                    packet_count = 0
                    data_ricevuti = 0
                    
                    try:
                        while True:
                            data = bytearray()
                            start_time = get_timestamp()
                            while len(data) < 3600:
                                chunk = conn.recv(min(buffer_size, 3600 - len(data)))
                                if not chunk:
                                  break
                                data.extend(chunk)
                            end_time = get_timestamp()

                            file_data.write(data.hex() + '\n')
                            file_data.flush()
                            data_ricevuti +=len(data)

                            log_message(file_log, f"Ricevuto pacchetto {packet_count}, size: {len(data)}")

                            if not data:
                                log_message(file_log, f"Connessione chiusa, nessun dato ricevuto da Arduino (PACKETS_PER_SECOND = {pps}, pacchetti ricevuti {packet_count})")
                                break

                            packet_count += 1
                            writer.writerow([packet_count, start_time, end_time, len(data), pps])
                            file.flush()

                            if data_ricevuti >= TOT_BYTE:
                                log_message(file_log, f"FINE RICEZIONE: Connessione chiusa (PACKETS_PER_SECOND = {pps}, pacchetti ricevuti {packet_count})")                                
                                break
                                
                    except socket.timeout:    
                        log_message(file_log, f"Timeout: nessun pacchetto ricevuto in {TIMEOUT} secondi")        
                        conn.close()
                    except KeyboardInterrupt:
                        log_message(file_log, f"Interrotto manualmente")
                    except Exception as e:
                        log_message(file_log, f"Errore: {str(e)}")
                    finally:
                        if not conn._closed:                            
                            print(f"\n{get_timestamp()} ns - Test completato (PACKETS_PER_SECOND = {pps})")
                            log_message(file_log, f"Test completato (PACKETS_PER_SECOND = {pps})")
                            conn.close()
server.close()
