import { app, BrowserWindow, Menu } from "electron";
import { WebSocketServer } from "ws";

function createWindow() {
  const win = new BrowserWindow({
    width: 1200,
    height: 800,
    webPreferences: {
      contextIsolation: true
    }
  });
  Menu.setApplicationMenu(null);
  win.loadURL("http://localhost:5173");
}

app.whenReady().then(() => {
  createWindow();

  // Inicjalizacja serwera WebSocket dla komunikacji Python <-> React
  const wss = new WebSocketServer({ port: 8080 });
  console.log("Serwer WebSocket działa na porcie 8080");

  wss.on('connection', function connection(ws) {
    console.log("Nowy klient podłączony (Python lub React)");
    
    ws.on('message', function message(data) {
      // Rozsyłanie odebranych danych (broadcast) do pozostałych klientów
      wss.clients.forEach(function each(client) {
        if (client !== ws && client.readyState === 1) {
          client.send(data.toString());
        }
      });
    });
  });
});