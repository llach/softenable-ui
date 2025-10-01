 const evtSource = new EventSource("/events");

const imgContainer = document.getElementById("image-container");
const img = document.getElementById("dynamic-image");
const textEl = document.getElementById("dynamic-text");
const contentBox = document.getElementById("content-box");

evtSource.onmessage = (e) => {
  const { text, image } = JSON.parse(e.data);

  textEl.innerHTML = (text || "").replace(/\n/g, "<br/>");

  if (image && image.length) {
    img.src = image;                  // data URL
    img.alt = "";                     // no fallback text
    imgContainer.style.display = "";  // show
    imgContainer.classList.remove("hidden");
    contentBox.classList.remove("justify-center");
  } else {
    // nuke the image completely
    img.removeAttribute("src");
    img.alt = "";                     // avoid alt text showing
    // hide container by both class and inline style (covers Tailwind purge cases)
    imgContainer.classList.add("hidden");
    imgContainer.style.display = "none";
    contentBox.classList.add("justify-center");
  }
};
