import os
import time
import base64
import requests

SERVER = "http://localhost:8080/update"
IMAGES_DIR = os.path.join(os.path.dirname(__file__), "images")

def send_update(text, image_path=None):
    data = {"text": text}

    if image_path:
        full_path = os.path.join(IMAGES_DIR, image_path)
        with open(full_path, "rb") as f:
            b64 = base64.b64encode(f.read()).decode("utf-8")
        ext = os.path.splitext(image_path)[1].lower().lstrip(".") or "png"
        mime = f"image/{'jpeg' if ext in ['jpg','jpeg'] else ext}"
        data["image"] = f"data:{mime};base64,{b64}"
    else:
        data["image"] = ""

    r = requests.post(SERVER, json=data)
    print(f"{image_path or 'no image'} -> {r.status_code} {r.text}")

if __name__ == "__main__":
    # 1) text-only
    send_update("Hello, this is text-only. Stand by.")
    time.sleep(2)

    # 2) with gloves.jpg
    send_update("Here is the gloves picture.", "gloves.jpg")
    time.sleep(2)

    # 3) with iri_logo.png
    send_update("SoftEnable IRI Logo incoming.", "iri_logo.png")
    time.sleep(2)

