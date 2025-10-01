// server.js — serves your index/assets, pushes updates via SSE, accepts POST /update
const http = require("http");
const fs = require("fs");
const path = require("path");
const { URL } = require("url");

const PORT = process.env.PORT || 8080;
const ROOT = process.env.WEB_ROOT ? path.resolve(process.env.WEB_ROOT) : path.resolve(__dirname);

let clients = [];

function serveFile(filePath, res) {
  fs.readFile(filePath, (err, data) => {
    if (err) { res.writeHead(404); res.end("Not found"); return; }
    const ext = path.extname(filePath).toLowerCase();
    const types = {
      ".html":"text/html; charset=utf-8",
      ".js":"text/javascript; charset=utf-8",
      ".css":"text/css; charset=utf-8",
      ".png":"image/png",
      ".jpg":"image/jpeg",
      ".jpeg":"image/jpeg",
      ".gif":"image/gif",
      ".webp":"image/webp",
      ".svg":"image/svg+xml",
    };
    res.writeHead(200, {"Content-Type": types[ext] || "application/octet-stream", "Cache-Control":"no-store"});
    res.end(data);
  });
}

const server = http.createServer((req, res) => {
  const url = new URL(req.url, `http://${req.headers.host}`);

  // SSE stream for browsers
  if (req.method === "GET" && url.pathname === "/events") {
    res.writeHead(200, {
      "Content-Type": "text/event-stream",
      "Cache-Control": "no-cache",
      "Connection": "keep-alive",
    });
    res.write("\n");
    clients.push(res);
    req.on("close", () => { clients = clients.filter(c => c !== res); });
    return;
  }

  // Accept updates (only thing that mutates state)
  if (req.method === "POST" && url.pathname === "/update") {
    let body = "";
    req.on("data", chunk => body += chunk);
    req.on("end", () => {
      try {
        let data = {};
        try { data = JSON.parse(body || "{}"); } catch {}
        const text = String(data.text || "");
        const image = ("image" in data) ? String(data.image || "") : ""; // <— missing => clear

        const payload = { text, image };
        const msg = `data: ${JSON.stringify(payload)}\n\n`;
        clients.forEach(c => c.write(msg));
        res.writeHead(200); res.end("ok");
      } catch {
        res.writeHead(400); res.end("bad json");
      }
    });
    return;
  }

  // Serve your index and assets (output.css, images, etc.)
  if (req.method === "GET") {
    const rel = url.pathname === "/" ? "index.html" : decodeURIComponent(url.pathname.replace(/^\/+/, ""));
    const filePath = path.join(ROOT, rel);
    // prevent path traversal
    if (!filePath.startsWith(ROOT)) { res.writeHead(403); return res.end("forbidden"); }
    return serveFile(filePath, res);
  }

  res.writeHead(404); res.end("not found");
});

server.listen(PORT, () => {
  console.log(`open http://localhost:${PORT}`);
});
