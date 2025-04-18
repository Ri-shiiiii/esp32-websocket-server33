// index.js
const WebSocket = require('ws');
const PORT = process.env.PORT || 10000;

const wss = new WebSocket.Server({ port: PORT });

wss.on('connection', (ws) => {
  console.log('ESP32 connected.');

  const interval = setInterval(() => {
    if (ws.readyState === WebSocket.OPEN) {
      ws.send('pong'); // respond to ping to keep connection alive
    }
  }, 10000); // Send pong every 10s

  ws.on('message', (message) => {
    if (message === 'ping') {
      console.log('[ESP32 Ping Received]');
      return;
    }

    try {
      const data = JSON.parse(message);
      console.log(`📡 Received heading: ${data.heading}`);
    } catch (err) {
      console.log('❌ Invalid JSON or other message:', message);
    }
  });

  ws.on('close', () => {
    console.log('🚫 ESP32 disconnected.');
    clearInterval(interval);
  });

  ws.on('error', (error) => {
    console.error('💥 WebSocket Error:', error);
  });
});

console.log(`🚀 WebSocket server is running on ws://localhost:${PORT}`);
