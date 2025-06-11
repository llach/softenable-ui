const textEl = document.getElementById("dynamic-text");
const imageEl = document.getElementById("dynamic-image");
const imageContainer = document.getElementById("image-container");

const socket = new WebSocket("ws://localhost:8080");

socket.onmessage = (event) => {
  try {
    const data = JSON.parse(event.data);
    if (typeof data.text === "string") {
      textEl.textContent = data.text;
    }
    if (typeof data.image === "string") {
      if (data.image.trim() === "") {
        imageEl.style.display = "none";
      } else {
        imageEl.src = data.image;
        imageEl.style.display = "block";
      }
    }
  } catch (e) {
    console.error("Invalid WebSocket message:", e);
  }
};
