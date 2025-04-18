const WebSocket = require('ws');
const PORT = process.env.PORT || 10000;

// Initialize WebSocket server
const wss = new WebSocket.Server({ port: PORT });

wss.on('connection', (ws) => {
  console.log('ESP32 connected.');

  // When a message is received from ESP32
  ws.on('message', (message) => {
    try {
      // Parse the received message
      const data = JSON.parse(message);
      console.log(`Received heading: ${data.heading}`);
    } catch (err) {
      // Handle invalid JSON messages
      console.log('Invalid JSON:', message);
    }
  });

  // When the connection is closed (ESP32 disconnects)
  ws.on('close', () => {
    console.log('ESP32 disconnected.');
  });

  // Optionally handle error events
  ws.on('error', (error) => {
    console.log('WebSocket Error:', error);
  });
});

// Log that the WebSocket server is running and the URL
console.log(`WebSocket server is running on wss://localhost:${PORT}`);
