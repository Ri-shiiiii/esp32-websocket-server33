const WebSocket = require('ws');
const PORT = process.env.PORT || 10000;

const wss = new WebSocket.Server({ port: PORT });

wss.on('connection', (ws) => {
  console.log('ESP32 connected.');

  ws.on('message', (message) => {
    try {
      const data = JSON.parse(message);
      console.log(`Received heading: ${data.heading}`);
    } catch (err) {
      console.log('Invalid JSON:', message);
    }
  });

  ws.on('close', () => {
    console.log('ESP32 disconnected.');
  });
});

console.log(`WebSocket server is running on ws://localhost:${PORT}`);
