/*
  ESP32  –  Modbus RTU ↔ TCP Gateway
  ====================================
  Chain:  Modbus Sensor → Modbus IC → [ESP32] → WiFi → Bridge app → config.exe

  The ESP32:
  • Connects to your WiFi
  • Listens for a TCP connection from the bridge app on port 502
  • When a Modbus TCP (MBAP) request arrives, strips the MBAP header,
    sends raw RTU to the sensor via UART2, reads the RTU response,
    wraps it back in MBAP, returns it over TCP.

  Wiring (adjust pins to your board):
    ESP32 GPIO16 (RX2) ← TX of Modbus IC / RS485 driver
    ESP32 GPIO17 (TX2) → RX of Modbus IC / RS485 driver
    GPIO4 (optional)   → DE/RE pin of MAX485 (HIGH=transmit, LOW=receive)

  Library needed:  none beyond Arduino ESP32 core
  Board:           "ESP32 Dev Module" in Arduino IDE
*/

#include <WiFi.h>

// ── WiFi credentials ──────────────────────────────────────────
const char* WIFI_SSID = "abcd";
const char* WIFI_PASS = "12345678";

// ── TCP server ────────────────────────────────────────────────
const uint16_t TCP_PORT = 502;          // standard Modbus TCP port
WiFiServer   server(TCP_PORT);
WiFiClient   client;

// ── RS-485 / UART ─────────────────────────────────────────────
#define MODBUS_SERIAL   Serial2
#define RS485_BAUD      9600           // match your sensor baud rate
#define PIN_RX2         16
#define PIN_TX2         17
#define PIN_DE_RE       4              // MAX485 direction pin; -1 if not used

// ── Timeouts ─────────────────────────────────────────────────
#define RTU_RESPONSE_TIMEOUT_MS  1000  // wait up to 1 s for sensor reply
#define RTU_INTER_CHAR_GAP_US    2000  // 2 ms silence = end of RTU frame

// ── Buffers ───────────────────────────────────────────────────
uint8_t tcpBuf[256];
uint8_t rtuBuf[256];

// ─────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  Serial.println("\n[ESP32] Modbus RTU↔TCP Gateway starting");

  // RS-485 direction pin
  if (PIN_DE_RE >= 0) {
    pinMode(PIN_DE_RE, OUTPUT);
    digitalWrite(PIN_DE_RE, LOW);  // receive mode
  }

  MODBUS_SERIAL.begin(RS485_BAUD, SERIAL_8N1, PIN_RX2, PIN_TX2);

  // WiFi
  Serial.printf("[WiFi] Connecting to %s", WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(300); Serial.print(".");
  }
  Serial.printf("\n[WiFi] Connected — IP: %s\n", WiFi.localIP().toString().c_str());

  server.begin();
  Serial.printf("[TCP]  Listening on port %d\n", TCP_PORT);
}

// ─────────────────────────────────────────────────────────────
void loop() {
  // Accept a new client if none connected
  if (!client || !client.connected()) {
    client = server.available();
    if (client) {
      Serial.printf("[TCP]  Client connected from %s\n",
                    client.remoteIP().toString().c_str());
    }
    return;
  }

  // ── Wait for a complete MBAP request from the bridge ──────
  if (!client.available()) return;

  // Read 6-byte MBAP header
  int got = readExact(client, tcpBuf, 6, 2000);
  if (got < 6) {
    Serial.println("[TCP]  Incomplete MBAP header, closing.");
    client.stop(); return;
  }

  uint16_t transId  = (tcpBuf[0] << 8) | tcpBuf[1];
  uint16_t protocol = (tcpBuf[2] << 8) | tcpBuf[3];
  uint16_t length   = (tcpBuf[4] << 8) | tcpBuf[5];

  if (protocol != 0x0000 || length < 2 || length > 250) {
    Serial.println("[TCP]  Bad MBAP, discarding.");
    client.stop(); return;
  }

  // Read the PDU (unit ID + function code + data)
  got = readExact(client, tcpBuf + 6, length, 2000);
  if (got < (int)length) {
    Serial.println("[TCP]  Incomplete PDU.");
    client.stop(); return;
  }

  // ── Build RTU frame:  [unitId][FC][data][CRC_lo][CRC_hi] ──
  uint8_t  unitId    = tcpBuf[6];
  uint8_t* pdu       = tcpBuf + 7;          // function code onwards
  uint16_t pduLen    = length - 1;           // minus the unit byte

  int rtuLen = 0;
  rtuBuf[rtuLen++] = unitId;
  memcpy(rtuBuf + rtuLen, pdu, pduLen);
  rtuLen += pduLen;

  uint16_t crc = crc16(rtuBuf, rtuLen);
  rtuBuf[rtuLen++] = crc & 0xFF;            // CRC low byte first
  rtuBuf[rtuLen++] = (crc >> 8) & 0xFF;    // CRC high byte

  // ── Send RTU to sensor ────────────────────────────────────
  if (PIN_DE_RE >= 0) digitalWrite(PIN_DE_RE, HIGH);  // transmit
  MODBUS_SERIAL.write(rtuBuf, rtuLen);
  MODBUS_SERIAL.flush();
  if (PIN_DE_RE >= 0) {
    delayMicroseconds(100);
    digitalWrite(PIN_DE_RE, LOW);   // back to receive
  }

  Serial.printf("[RTU]  Sent %d bytes to sensor  [fn=%02X]\n", rtuLen, rtuBuf[1]);

  // ── Wait for RTU response ─────────────────────────────────
  int rspLen = readRTUResponse(rtuBuf, sizeof(rtuBuf));
  if (rspLen < 4) {
    Serial.println("[RTU]  No / short response from sensor.");
    // Send Modbus exception 0x0B (Gateway Target Device Failed to Respond)
    sendMBAPException(client, transId, unitId, rtuBuf[1]);
    return;
  }

  Serial.printf("[RTU]  Got %d bytes from sensor\n", rspLen);

  // ── Wrap RTU response in MBAP, send back ─────────────────
  // PDU = unit + function + data (everything except 2-byte CRC)
  int       respPduLen = rspLen - 2;        // strip CRC
  uint16_t  respLength = respPduLen;        // bytes after MBAP header

  uint8_t resp[256];
  resp[0] = transId >> 8;   resp[1] = transId & 0xFF;
  resp[2] = 0; resp[3] = 0;                // protocol
  resp[4] = respLength >> 8; resp[5] = respLength & 0xFF;
  memcpy(resp + 6, rtuBuf, respPduLen);    // unit+FC+data (no CRC)

  client.write(resp, 6 + respPduLen);
  Serial.printf("[TCP]  Forwarded %d-byte response\n", 6 + respPduLen);
}

// ─────────────────────────────────────────────────────────────
// Read exactly 'n' bytes from client, with timeout (ms)
int readExact(WiFiClient& c, uint8_t* buf, int n, unsigned long timeoutMs) {
  unsigned long t0 = millis();
  int got = 0;
  while (got < n && (millis() - t0) < timeoutMs) {
    if (c.available()) buf[got++] = c.read();
    else delay(1);
  }
  return got;
}

// Read a complete RTU frame from MODBUS_SERIAL (silence-delimited)
int readRTUResponse(uint8_t* buf, int maxLen) {
  unsigned long t0  = millis();
  unsigned long last = 0;
  int n = 0;
  bool started = false;

  while (millis() - t0 < RTU_RESPONSE_TIMEOUT_MS) {
    if (MODBUS_SERIAL.available()) {
      buf[n++] = MODBUS_SERIAL.read();
      last = micros();
      started = true;
      if (n >= maxLen) break;
    } else if (started && (micros() - last) > RTU_INTER_CHAR_GAP_US) {
      break;   // inter-frame silence detected → frame complete
    }
  }
  return n;
}

// Send a Modbus TCP exception response
void sendMBAPException(WiFiClient& c, uint16_t transId,
                        uint8_t unitId, uint8_t funcCode) {
  uint8_t ex[9];
  ex[0] = transId >> 8; ex[1] = transId & 0xFF;
  ex[2] = 0; ex[3] = 0;
  ex[4] = 0; ex[5] = 3;            // length = 3 bytes (unit+FC+ex code)
  ex[6] = unitId;
  ex[7] = funcCode | 0x80;         // error flag
  ex[8] = 0x0B;                    // gateway target failed
  c.write(ex, 9);
}

// Standard Modbus CRC-16
uint16_t crc16(uint8_t* buf, int len) {
  uint16_t crc = 0xFFFF;
  for (int i = 0; i < len; i++) {
    crc ^= buf[i];
    for (int b = 0; b < 8; b++) {
      if (crc & 0x0001) crc = (crc >> 1) ^ 0xA001;
      else              crc >>= 1;
    }
  }
  return crc;
}
