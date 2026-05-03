"""
Modbus RTU ↔ TCP Bridge
========================
For the chain:  Modbus Sensor → Modbus IC → ESP32 (TCP) → [THIS] → config.exe

config.exe speaks Modbus RTU over a COM port.
Your ESP32 serves Modbus over TCP (either raw RTU-over-TCP or Modbus TCP with MBAP header).
This bridge sits between them and does the protocol translation.

HOW TO USE
──────────
1.  Install:  pip install pyserial pymodbus
2.  Install com0com (free virtual null-modem driver):
    https://sourceforge.net/projects/com0com/
    Create a pair, e.g.  COM10 ↔ COM11
3.  In this app:
    - Serial Port  → COM11  (bridge side)
    - ESP32 IP/Port → your ESP32's IP, port 502 (Modbus TCP) or custom
4.  In config.exe:
    - Select  COM10  (the other half of the virtual pair)
5.  Click  Start Bridge
    config.exe → COM10 → COM11 → Bridge → ESP32 TCP → Modbus sensor
"""

import tkinter as tk
from tkinter import ttk, scrolledtext, messagebox
import threading
import socket
import serial
import serial.tools.list_ports
import time
import queue
import struct
import traceback

# ── Theme ──────────────────────────────────────────────────────
C = {
    "bg":      "#0f1117",
    "panel":   "#161b22",
    "border":  "#30363d",
    "fg":      "#c9d1d9",
    "muted":   "#8b949e",
    "accent":  "#58a6ff",
    "green":   "#3fb950",
    "red":     "#f85149",
    "yellow":  "#d29922",
    "orange":  "#f0883e",
    "purple":  "#bc8cff",
}

BAUD_RATES = ["9600", "19200", "38400", "57600", "115200", "4800", "2400"]

# ─────────────────────────────────────────────────────────────
#  Modbus helpers
# ─────────────────────────────────────────────────────────────
MBAP_HEADER_LEN = 6   # Transaction(2) + Protocol(2) + Length(2)
_mbap_counter   = 0

def rtu_to_mbap(rtu_frame: bytes) -> bytes:
    """Wrap a Modbus RTU frame (minus CRC) into a Modbus TCP MBAP packet."""
    global _mbap_counter
    _mbap_counter = (_mbap_counter + 1) & 0xFFFF
    pdu = rtu_frame[:-2]          # strip 2-byte CRC
    length = len(pdu)
    header = struct.pack(">HHHB",
                         _mbap_counter,   # Transaction ID
                         0x0000,          # Protocol ID (Modbus = 0)
                         length,          # byte count that follows
                         pdu[0])          # Unit ID (= slave address)
    return header + pdu[1:]               # header + PDU without unit byte (already in header)

def mbap_to_rtu(mbap_frame: bytes) -> bytes:
    """Unwrap a Modbus TCP MBAP response into a Modbus RTU frame (adds CRC)."""
    if len(mbap_frame) < MBAP_HEADER_LEN:
        return mbap_frame
    unit_id = mbap_frame[6]                 # 7th byte
    pdu     = mbap_frame[7:]                # PDU (function code + data)
    rtu     = bytes([unit_id]) + pdu
    crc     = _crc16(rtu)
    return rtu + struct.pack("<H", crc)

def _crc16(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc

def valid_rtu_crc(frame: bytes) -> bool:
    if len(frame) < 4:
        return False
    calc = _crc16(frame[:-2])
    got  = struct.unpack("<H", frame[-2:])[0]
    return calc == got

# ─────────────────────────────────────────────────────────────
#  Main app
# ─────────────────────────────────────────────────────────────
class ModbusBridgeApp(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Modbus RTU ↔ TCP Bridge  │  ESP32 Gateway")
        self.configure(bg=C["bg"])
        self.resizable(False, False)

        self._running   = False
        self._ser       = None
        self._sock      = None
        self._threads   = []
        self._log_q     = queue.Queue()
        self._req_count = 0
        self._err_count = 0
        self._last_rtt  = 0.0

        self._build_ui()
        self._refresh_ports()
        self._poll_log()
        self._poll_stats()

    # ── UI ────────────────────────────────────────────────────
    def _build_ui(self):
        # top accent line
        tk.Frame(self, bg=C["accent"], height=3).pack(fill="x")

        # header
        hdr = tk.Frame(self, bg=C["bg"])
        hdr.pack(fill="x", padx=24, pady=(16, 4))

        tk.Label(hdr, text="Modbus RTU ↔ TCP Bridge",
                 bg=C["bg"], fg=C["fg"],
                 font=("Courier New", 15, "bold")).pack(side="left")

        self._status_pill = tk.Label(hdr, text="  STOPPED  ",
                                     bg=C["red"], fg="white",
                                     font=("Courier New", 8, "bold"),
                                     padx=6, pady=2)
        self._status_pill.pack(side="right", pady=4)

        tk.Label(self, text="config.exe  →  COM (virtual)  →  Bridge  →  ESP32 TCP  →  Modbus sensor",
                 bg=C["bg"], fg=C["muted"],
                 font=("Courier New", 8)).pack(padx=24, anchor="w")

        # ── main settings row ──
        row = tk.Frame(self, bg=C["bg"])
        row.pack(fill="x", padx=24, pady=12)

        # Serial card
        sc = self._card(row, "SERIAL  (connect config.exe here)")
        sc.pack(side="left", fill="y", padx=(0, 8))
        self._port_var = tk.StringVar()
        self._port_cb  = self._combo(sc, "Virtual COM Port", self._port_var, [], 0)
        btn_ref = tk.Button(sc, text="↺ Refresh",
                            bg=C["panel"], fg=C["accent"],
                            relief="flat", font=("Courier New", 8),
                            cursor="hand2", command=self._refresh_ports)
        btn_ref.grid(row=0, column=2, padx=(4, 0), pady=6)

        self._baud_var = tk.StringVar(value="9600")
        self._combo(sc, "Baud Rate", self._baud_var, BAUD_RATES, 1)

        # ESP32 / TCP card
        tc = self._card(row, "ESP32  TCP/IP")
        tc.pack(side="left", fill="y", padx=(8, 0))

        self._ip_var = tk.StringVar(value="192.168.1.100")
        self._port_entry_var = tk.StringVar(value="502")
        self._field(tc, "ESP32 IP Address", self._ip_var, 0)
        self._field(tc, "TCP Port (502 = Modbus TCP)", self._port_entry_var, 1)

        # ── protocol mode ──
        pm = tk.Frame(self, bg=C["panel"],
                      highlightthickness=1, highlightbackground=C["border"])
        pm.pack(fill="x", padx=24, pady=(0, 8))

        tk.Label(pm, text="ESP32 Protocol Mode",
                 bg=C["panel"], fg=C["muted"],
                 font=("Courier New", 8, "bold")).pack(side="left", padx=12, pady=8)

        self._mode_var = tk.StringVar(value="mbap")
        for val, label, tip in [
            ("mbap", "Modbus TCP  (MBAP)",
             "ESP32 adds 6-byte MBAP header  [standard port 502]"),
            ("raw",  "Raw RTU over TCP",
             "ESP32 forwards raw RTU bytes as-is  [common in cheap gateways]"),
        ]:
            f = tk.Frame(pm, bg=C["panel"])
            f.pack(side="left", padx=12, pady=6)
            tk.Radiobutton(f, text=label, variable=self._mode_var, value=val,
                           bg=C["panel"], fg=C["fg"], selectcolor=C["bg"],
                           activebackground=C["panel"], activeforeground=C["accent"],
                           font=("Courier New", 9)
                           ).pack(anchor="w")
            tk.Label(f, text=tip, bg=C["panel"], fg=C["muted"],
                     font=("Courier New", 7)).pack(anchor="w")

        # ── stats bar ──
        sb = tk.Frame(self, bg=C["panel"],
                      highlightthickness=1, highlightbackground=C["border"])
        sb.pack(fill="x", padx=24, pady=(0, 8))

        self._stat_labels = {}
        for key, label in [("req", "Requests"), ("err", "Errors"), ("rtt", "Last RTT")]:
            f = tk.Frame(sb, bg=C["panel"])
            f.pack(side="left", padx=16, pady=6)
            tk.Label(f, text=label, bg=C["panel"], fg=C["muted"],
                     font=("Courier New", 7)).pack()
            lbl = tk.Label(f, text="—", bg=C["panel"], fg=C["accent"],
                           font=("Courier New", 11, "bold"))
            lbl.pack()
            self._stat_labels[key] = lbl

        # ── start button ──
        self._btn = tk.Button(self,
                              text="▶   START BRIDGE",
                              bg=C["green"], fg=C["bg"],
                              font=("Courier New", 11, "bold"),
                              relief="flat", pady=10,
                              cursor="hand2",
                              command=self._toggle)
        self._btn.pack(fill="x", padx=24, pady=(0, 8))

        # ── log ──
        tk.Label(self, text="ACTIVITY LOG",
                 bg=C["bg"], fg=C["muted"],
                 font=("Courier New", 7, "bold")).pack(anchor="w", padx=26)

        self._log_box = scrolledtext.ScrolledText(
            self, width=70, height=14,
            bg=C["panel"], fg=C["fg"],
            font=("Courier New", 8),
            relief="flat", state="disabled",
            insertbackground=C["fg"])
        self._log_box.pack(padx=24, pady=(2, 16), fill="x")

        for tag, color in [("ok", C["green"]), ("err", C["red"]),
                           ("req", C["accent"]), ("rsp", C["purple"]),
                           ("warn", C["yellow"]), ("dim", C["muted"])]:
            self._log_box.tag_config(tag, foreground=color)

        self.protocol("WM_DELETE_WINDOW", self._on_close)

    def _card(self, parent, title):
        f = tk.Frame(parent, bg=C["panel"],
                     highlightthickness=1, highlightbackground=C["border"])
        tk.Label(f, text=title, bg=C["panel"], fg=C["muted"],
                 font=("Courier New", 7, "bold")).grid(
            row=0, column=0, columnspan=3, padx=12, pady=(8, 4), sticky="w")
        return f

    def _combo(self, parent, label, var, values, row):
        tk.Label(parent, text=label, bg=C["panel"], fg=C["fg"],
                 font=("Courier New", 8), anchor="w",
                 width=22).grid(row=row+1, column=0, padx=(12, 6), pady=5, sticky="w")
        cb = ttk.Combobox(parent, textvariable=var, values=values,
                          width=14, state="readonly",
                          font=("Courier New", 8))
        cb.grid(row=row+1, column=1, padx=(0, 12), pady=5, sticky="w")
        if values:
            cb.set(values[0])
        return cb

    def _field(self, parent, label, var, row):
        tk.Label(parent, text=label, bg=C["panel"], fg=C["fg"],
                 font=("Courier New", 8), anchor="w",
                 width=28).grid(row=row+1, column=0, padx=(12, 6), pady=5, sticky="w")
        e = tk.Entry(parent, textvariable=var, width=18,
                     bg=C["bg"], fg=C["fg"], insertbackground=C["fg"],
                     relief="flat", font=("Courier New", 8),
                     highlightthickness=1, highlightbackground=C["border"])
        e.grid(row=row+1, column=1, padx=(0, 12), pady=5, sticky="w")

    def _refresh_ports(self):
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self._port_cb["values"] = ports
        if ports and not self._port_var.get():
            self._port_var.set(ports[0])
        if not ports:
            self._port_var.set("")

    # ── bridge control ────────────────────────────────────────
    def _toggle(self):
        if self._running:
            self._stop()
        else:
            self._start()

    def _start(self):
        port = self._port_var.get()
        baud = int(self._baud_var.get())
        ip   = self._ip_var.get().strip()
        tcp_port = int(self._port_entry_var.get().strip())
        mode = self._mode_var.get()

        if not port:
            messagebox.showwarning("No Port", "Select a virtual COM port (from com0com).")
            return

        # Open serial
        try:
            self._ser = serial.Serial(port, baud, timeout=0.1)
            self._log(f"Serial open: {port} @ {baud} baud", "ok")
        except Exception as e:
            messagebox.showerror("Serial Error", str(e)); return

        # Connect to ESP32
        try:
            self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self._sock.settimeout(5)
            self._sock.connect((ip, tcp_port))
            self._sock.settimeout(2)
            self._log(f"TCP connected → {ip}:{tcp_port}", "ok")
        except Exception as e:
            self._ser.close(); self._ser = None
            messagebox.showerror("TCP Error", f"Cannot reach ESP32 at {ip}:{tcp_port}\n\n{e}")
            return

        self._running = True
        self._req_count = self._err_count = 0
        self._mode_val  = mode

        t = threading.Thread(target=self._bridge_loop, daemon=True)
        t.start(); self._threads.append(t)

        self._btn.config(text="■   STOP BRIDGE", bg=C["red"], fg="white")
        self._status_pill.config(text="  RUNNING  ", bg=C["green"])
        self._log(f"Bridge running  [mode: {'Modbus TCP / MBAP' if mode == 'mbap' else 'Raw RTU over TCP'}]", "ok")
        self._log("Waiting for config.exe to send a request …", "dim")

    def _stop(self):
        self._running = False
        try:
            if self._sock: self._sock.close()
        except: pass
        try:
            if self._ser:  self._ser.close()
        except: pass
        self._sock = self._ser = None
        self._threads.clear()
        self._btn.config(text="▶   START BRIDGE", bg=C["green"], fg=C["bg"])
        self._status_pill.config(text="  STOPPED  ", bg=C["red"])
        self._log("Bridge stopped.", "warn")

    # ── main relay loop ───────────────────────────────────────
    def _bridge_loop(self):
        """
        Reads a complete Modbus RTU frame from the serial port,
        converts it for ESP32, sends it over TCP, waits for the
        response, converts back, writes to serial.
        """
        while self._running:
            try:
                rtu_req = self._read_rtu_frame()
                if not rtu_req:
                    continue

                self._req_count += 1
                self._log_frame("config.exe → bridge", rtu_req, "req")

                if not valid_rtu_crc(rtu_req):
                    self._log("⚠ Bad CRC on RTU request – forwarding anyway", "warn")
                    self._err_count += 1

                t0 = time.perf_counter()

                # ── convert & send to ESP32 ──
                if self._mode_val == "mbap":
                    tcp_payload = rtu_to_mbap(rtu_req)
                else:
                    tcp_payload = rtu_req          # raw RTU as-is

                self._sock.sendall(tcp_payload)
                self._log_frame("bridge → ESP32", tcp_payload, "dim")

                # ── receive response from ESP32 ──
                tcp_resp = self._recv_tcp_response()
                if not tcp_resp:
                    self._log("No response from ESP32 (timeout)", "err")
                    self._err_count += 1
                    continue

                self._log_frame("ESP32 → bridge", tcp_resp, "dim")

                # ── convert back to RTU and send to config.exe ──
                if self._mode_val == "mbap":
                    rtu_resp = mbap_to_rtu(tcp_resp)
                else:
                    rtu_resp = tcp_resp            # already RTU

                self._ser.write(rtu_resp)
                rtt = (time.perf_counter() - t0) * 1000
                self._last_rtt = rtt
                self._log_frame(f"bridge → config.exe  [{rtt:.1f} ms]", rtu_resp, "rsp")

            except Exception as e:
                if self._running:
                    self._log(f"Bridge error: {e}", "err")
                    self._err_count += 1
                time.sleep(0.1)

    def _read_rtu_frame(self) -> bytes:
        """
        Read a complete Modbus RTU frame from serial.
        RTU has no length field — we detect end-of-frame by
        inter-character silence (3.5 char times).
        """
        frame = b""
        self._ser.timeout = 0.05        # 50 ms inter-char gap = end of frame
        while self._running:
            chunk = self._ser.read(256)
            if chunk:
                frame += chunk
            elif frame:
                # silence after data = complete frame
                return frame
        return b""

    def _recv_tcp_response(self) -> bytes:
        """Receive a complete response from ESP32."""
        data = b""
        try:
            if self._mode_val == "mbap":
                # Read 6-byte MBAP header first
                header = self._recv_exact(6)
                if not header or len(header) < 6:
                    return b""
                length = struct.unpack(">H", header[4:6])[0]
                rest   = self._recv_exact(length)
                return header + (rest or b"")
            else:
                # Raw RTU: read until silence
                self._sock.settimeout(0.5)
                while True:
                    try:
                        chunk = self._sock.recv(256)
                        if not chunk: break
                        data += chunk
                    except socket.timeout:
                        break
                return data
        except Exception as e:
            if self._running:
                self._log(f"TCP recv error: {e}", "err")
            return b""

    def _recv_exact(self, n: int) -> bytes:
        """Read exactly n bytes from TCP socket."""
        buf = b""
        while len(buf) < n:
            try:
                chunk = self._sock.recv(n - len(buf))
                if not chunk:
                    return buf
                buf += chunk
            except socket.timeout:
                return buf
        return buf

    # ── logging ───────────────────────────────────────────────
    def _log_frame(self, label: str, frame: bytes, tag: str):
        hex_s = " ".join(f"{b:02X}" for b in frame)
        if len(hex_s) > 60:
            hex_s = hex_s[:60] + " …"
        self._log(f"{label:<30}  {hex_s}", tag)

    def _log(self, msg: str, tag: str = ""):
        self._log_q.put((msg, tag))

    def _poll_log(self):
        while not self._log_q.empty():
            msg, tag = self._log_q.get_nowait()
            ts = time.strftime("%H:%M:%S")
            self._log_box.config(state="normal")
            self._log_box.insert("end", f"[{ts}]  {msg}\n", tag)
            self._log_box.see("end")
            self._log_box.config(state="disabled")
        self.after(100, self._poll_log)

    def _poll_stats(self):
        self._stat_labels["req"].config(text=str(self._req_count))
        self._stat_labels["err"].config(
            text=str(self._err_count),
            fg=C["red"] if self._err_count else C["green"])
        self._stat_labels["rtt"].config(
            text=f"{self._last_rtt:.1f} ms" if self._last_rtt else "—")
        self.after(500, self._poll_stats)

    def _on_close(self):
        self._stop()
        self.destroy()


# ──────────────────────────────────────────────────────────────
if __name__ == "__main__":
    app = ModbusBridgeApp()
    app.mainloop()
