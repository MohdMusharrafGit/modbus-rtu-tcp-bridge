# 🔌 Modbus RTU ↔ TCP Bridge using ESP32

## 📌 Overview

This project enables legacy serial-based Modbus RTU software to communicate with Modbus TCP devices over WiFi using an ESP32 and a Python bridge.

It is designed for systems where existing software supports only serial COM communication, while the actual device is accessible over TCP/IP.

---

## 🧠 System Architecture

```
Modbus Sensor → RS485 → ESP32 → WiFi (TCP/IP)
                                      ↓
                             Python Bridge
                                      ↓
                    Virtual COM Port (VSPE)
                                      ↓
                               config.exe
```

---

## 🚀 Features

* Convert **Modbus RTU ↔ Modbus TCP**
* Works with **serial-only legacy software**
* Enables **wireless communication using ESP32**
* No modification required in existing software
* Low-cost and scalable solution

---

## 🛠 Components Used

* ESP32 (WiFi-enabled microcontroller)
* MAX485 / RS485 module
* Modbus RTU sensor/device
* Python (TCP ↔ Serial bridge)
* VSPE (Virtual Serial Port Emulator)

---

## ⚙️ Setup Guide

### 1. ESP32 Setup
- Upload `esp32_modbus_gateway.ino`
- Connect RS485 module to ESP32
- Open Serial Monitor and note IP address

---

### 2. Create Virtual COM Ports
- Open VSPE (Run as Administrator)
- Create:
```
COM20 ↔ COM21
```

---

### 3. Run Python Bridge
Install:
```
pip install pyserial
```

Run:
```
python modbus_bridge.py
```

---

### 4. Configure Bridge
- COM Port: COM21
- Baud Rate: 9600
- ESP32 IP: <your ESP32 IP>
- TCP Port: 502
- Mode: Modbus TCP

---

### 5. Configure config.exe
- COM Port: COM20
- Baud: 9600

---

## 🔁 Data Flow

```
config.exe → COM20 → COM21 → Python Bridge → ESP32 → Sensor
```

---

## 📦 Project Structure

```
├── esp32_modbus_gateway.ino
├── modbus_bridge.py
├── README.md
└── requirements.txt
```

---

## 📋 Requirements

- Python 3.x
- pyserial
- VSPE

---

## ⚠️ Notes

- ESP32 and PC must be on same WiFi
- Close Arduino Serial Monitor before running bridge
- Firewall may block port 502
- Baud rate must match sensor

---

## 📌 Use Cases

- Industrial IoT
- Remote sensor monitoring
- Converting serial systems to wireless
- Smart factory applications

---

## 👨‍💻 Author

Mohd Musharraf

---

## ⭐ Future Improvements

- GUI for bridge
- Auto COM detection
- Multi-device support
- Web dashboard
