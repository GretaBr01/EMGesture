import os
from PIL import Image

ASCII_CHARS = "@%#*+=-:. "

def resize_image(image, max_width, max_height):
    width, height = image.size
    aspect_ratio = height / width
    new_width = min(max_width, width)
    new_height = int(new_width * aspect_ratio * 0.55)

    # Limita l'altezza se necessario
    if new_height > max_height:
        new_height = max_height
        new_width = int(new_height / (aspect_ratio * 0.55))

    return image.resize((new_width, new_height))

def grayify(image):
    return image.convert("L")

def pixels_to_ascii(image):
    pixels = image.getdata()
    chars = "".join([ASCII_CHARS[min(pixel // 25, len(ASCII_CHARS) - 1)] for pixel in pixels])
    return chars

def image_to_ascii(image_path):
    try:
        image = Image.open(image_path)
    except Exception as e:
        print("Errore nell'apertura dell'immagine:", e)
        return

    # Ottieni dimensioni del terminale
    try:
        term_size = os.get_terminal_size()
        max_width = term_size.columns
        max_height = term_size.lines - 2  # lasciano margine
    except OSError:
        max_width = 80
        max_height = 24

    image = resize_image(image, max_width, max_height)
    image = grayify(image)

    ascii_str = pixels_to_ascii(image)

    pixel_count = len(ascii_str)
    width = image.width
    ascii_image = "\n".join(ascii_str[i:i+width] for i in range(0, pixel_count, width))

    return ascii_image

if __name__ == "__main__":
    ascii_art = image_to_ascii("img/right-arrow.jpg")
    if ascii_art:
        print("\033c", end="")  # pulisce il terminale
        print(ascii_art)
