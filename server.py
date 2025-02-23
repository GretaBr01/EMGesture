import socket
import time
import csv
import os
import argparse

HOST = "192.168.144.74"
PORT = 6789
BUFFER_SIZE = 1024
DURATION = 10
TIMEOUT = 1.0

# Parsing degli argomenti da riga di comando
parser = argparse.ArgumentParser(description="Script per acquisizione dati con timestamp sincronizzati.")
parser.add_argument("pps", type=int, help="Numero di pacchetti al secondo")
args = parser.parse_args()
pps = args.pps  # Numero di pacchetti al secondo passato dall'utente


FOLDER_NAME = "WiFiTestResult"
if not os.path.exists(FOLDER_NAME):
    os.makedirs(FOLDER_NAME)

print(pps)
file_index = 1
while os.path.exists(f"{FOLDER_NAME}/timestamp_data_{pps}pps_{file_index}.csv"):
    file_index += 1

FILENAME = f"{FOLDER_NAME}/timestamp_data_{pps}pps_{file_index}.csv"

server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.bind((HOST, PORT))
server_socket.listen(1)

print(f"Server in ascolto su {HOST}:{PORT}...")

conn, addr = server_socket.accept()
print(f"Connesso da {addr}")
conn.settimeout(TIMEOUT)

# Inizializza variabili di test
start_time = time.time()
total_bytes_received = 0
packets_received = 0
packet_size = 2400  # Modifica a 1200 o 480 se necessario
end_time = 0

# Apri il file CSV per salvare i dati
with open(FILENAME, mode="w", newline="") as file:
    writer = csv.writer(file)
    writer.writerow(["Timestamp", "BytesReceived", "PacketsReceived", "SpeedKbps"])  # Intestazione CSV
    
    try:
        while True:
            try:
                data = conn.recv(packet_size)
                if not data:
                    print("Connessione persa, in attesa di nuova connessione...")
                    break

                total_bytes_received += len(data)
                packets_received += 1
                elapsed_time = time.time() - start_time
                speed_kbps = (total_bytes_received * 8) / (elapsed_time * 1000)  # Calcola velocità in Kbps
                
                # Stampa aggiornamento in tempo reale
                print(f"{packets_received} - Ricevuti {len(data)} byte - {speed_kbps:.2f} Kbps")

                # Salva dati su CSV
                writer.writerow([elapsed_time, len(data), packets_received, speed_kbps])
            except socket.timeout:
                end_time = time.time()
                pass

    except KeyboardInterrupt:
        print("\nTest interrotto manualmente.")
    finally:
        # Chiusura e stampa risultati finali
        duration = end_time - start_time - TIMEOUT
        avg_speed_kbps = (total_bytes_received * 8) / (duration * 1000)
        
        print("\n**Test Concluso**")
        print(f"Durata: {duration:.2f} s")
        print(f"Pacchetti ricevuti: {packets_received}")
        print(f"Totale dati ricevuti: {total_bytes_received} byte")
        print(f"Velocità media: {avg_speed_kbps:.2f} Kbps")
        print(f"Dati salvati in {FILENAME}")

        conn.close()
        server_socket.close()
