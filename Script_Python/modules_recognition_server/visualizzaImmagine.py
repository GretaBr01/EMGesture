import subprocess
import os
import sys
import signal

# Questo script mostra un'immagine in un terminale Linux
# utilizzando il comando 'fbi' (framebuffer image viewer).
# esempio: python3 visualizzaImmagine.py immagini/freccia.jpg


def mostra_immagine(path_img):
    if not os.path.exists(path_img):
        print(f"Errore: il file '{path_img}' non esiste.")
        return

    try:
        # Avvia il processo fbi per mostrare l'immagine
        subprocess.run([
            "sudo", "fbi",
            "-T", "1",              # Usa la tty1
            "-d", "/dev/fb0",       # Usa il framebuffer
            "-noverbose",          # Nessun output verboso
            "-a",                  # Scala automaticamente
            path_img
        ])
    except Exception as e:
        print(f"Errore durante l'esecuzione di fbi: {e}")
   # finally:
        # Ripristina l'eco del terminale (scritto su schermo)
        #os.system("stty echo")

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Uso: python3 mostra_immagine.py /percorso/immagine.jpg")
    else:
        mostra_immagine(sys.argv[1])
        
