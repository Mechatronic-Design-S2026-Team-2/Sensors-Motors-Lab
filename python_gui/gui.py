import tkinter as tk
from tkinter import ttk, messagebox
import serial
import serial.tools.list_ports
import threading
import time
from queue import Queue, Empty

# ====== CONFIG ======
BAUD = 115200          # must match Arduino Serial.begin(...)
READ_TIMEOUT = 0.1
STATE_PREFIX = "STATE,"
# ====================


class SerialWorker:
    """Background serial reader/writer wrapper."""
    def __init__(self):
        self.ser = None
        self.rx_queue = Queue()
        self._stop = threading.Event()
        self._thread = None

    def is_connected(self):
        return self.ser is not None and self.ser.is_open

    def connect(self, port: str, baud: int = BAUD):
        if self.is_connected():
            return
        self.ser = serial.Serial(port, baud, timeout=READ_TIMEOUT)
        # Arduino often resets on serial connect; wait a bit:
        time.sleep(0.25)
        self._stop.clear()
        self._thread = threading.Thread(target=self._reader_loop, daemon=True)
        self._thread.start()

    def disconnect(self):
        self._stop.set()
        if self.ser:
            try:
                self.ser.close()
            except Exception:
                pass
        self.ser = None

    def send_line(self, line: str):
        if not self.is_connected():
            raise RuntimeError("Not connected")
        payload = (line.strip() + "\n").encode("utf-8")
        self.ser.write(payload)

    def _reader_loop(self):
        while not self._stop.is_set():
            if not self.ser:
                break
            try:
                line = self.ser.readline().decode("utf-8", errors="ignore").strip()
                if line:
                    self.rx_queue.put(line)
            except Exception:
                time.sleep(0.05)


class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("ILR Sensor-Motor GUI")
        self.geometry("980x600")

        self.worker = SerialWorker()

        # ----- UI state -----
        self.port_var = tk.StringVar()
        self.status_var = tk.StringVar(value="Disconnected")

        # Live values
        self.s1_var = tk.StringVar(value="-")
        self.s2_var = tk.StringVar(value="-")
        self.s3_var = tk.StringVar(value="-")
        self.servo_deg_var = tk.StringVar(value="-")
        self.dc_pos_deg_var = tk.StringVar(value="-")
        self.dc_vel_dps_var = tk.StringVar(value="-")
        self.step_pos_deg_var = tk.StringVar(value="-")
        self.mode_var = tk.StringVar(value="-")

        self._build_ui()
        self._refresh_ports()

        # Poll serial queue regularly
        self.after(50, self._poll_serial)

    def _build_ui(self):
        # ===== Top bar =====
        top = ttk.Frame(self, padding=10)
        top.pack(fill="x")

        ttk.Label(top, text="Serial Port:").pack(side="left")
        self.port_combo = ttk.Combobox(top, textvariable=self.port_var, width=26, state="readonly")
        self.port_combo.pack(side="left", padx=6)

        ttk.Button(top, text="Refresh", command=self._refresh_ports).pack(side="left", padx=6)
        ttk.Button(top, text="Connect", command=self.connect).pack(side="left", padx=6)
        ttk.Button(top, text="Disconnect", command=self.disconnect).pack(side="left", padx=6)

        ttk.Label(top, textvariable=self.status_var, foreground="blue").pack(side="left", padx=16)

        # ===== Main area =====
        main = ttk.Frame(self, padding=10)
        main.pack(fill="both", expand=True)

        # ---- Left: Live status ----
        status = ttk.LabelFrame(main, text="Live Status (Arduino -> GUI)", padding=10)
        status.grid(row=0, column=0, sticky="nsew", padx=(0, 10))

        def add_row(r, label, var):
            ttk.Label(status, text=label, width=20).grid(row=r, column=0, sticky="w", pady=2)
            ttk.Label(status, textvariable=var, width=18).grid(row=r, column=1, sticky="w", pady=2)

        add_row(0, "Sensor 1:", self.s1_var)
        add_row(1, "Sensor 2:", self.s2_var)
        add_row(2, "Sensor 3:", self.s3_var)

        ttk.Separator(status, orient="horizontal").grid(row=3, column=0, columnspan=2, sticky="ew", pady=10)

        add_row(4, "Servo angle (deg):", self.servo_deg_var)
        add_row(5, "DC pos (deg):", self.dc_pos_deg_var)
        add_row(6, "DC vel (deg/s):", self.dc_vel_dps_var)
        add_row(7, "Stepper pos (deg):", self.step_pos_deg_var)
        add_row(8, "Mode:", self.mode_var)

        # ---- Right: Controls ----
        ctrl = ttk.LabelFrame(main, text="Controls (GUI -> Arduino)", padding=10)
        ctrl.grid(row=0, column=1, sticky="nsew")

        # Servo
        servo_box = ttk.LabelFrame(ctrl, text="RC Servo", padding=10)
        servo_box.pack(fill="x", pady=6)

        self.servo_scale = ttk.Scale(servo_box, from_=0, to=180, orient="horizontal")
        self.servo_scale.set(90)
        self.servo_scale.pack(fill="x")

        btn_row = ttk.Frame(servo_box)
        btn_row.pack(fill="x", pady=6)
        ttk.Button(btn_row, text="Send", command=self.send_servo).pack(side="left")
        ttk.Button(btn_row, text="0°", command=lambda: self._set_servo_and_send(0)).pack(side="left", padx=6)
        ttk.Button(btn_row, text="180°", command=lambda: self._set_servo_and_send(180)).pack(side="left")

        # DC
        dc_box = ttk.LabelFrame(ctrl, text="DC Motor (Encoder + PID)", padding=10)
        dc_box.pack(fill="x", pady=6)

        row1 = ttk.Frame(dc_box)
        row1.pack(fill="x", pady=2)
        ttk.Label(row1, text="ΔPos (deg):").pack(side="left")
        self.dc_dpos = ttk.Entry(row1, width=10)
        self.dc_dpos.insert(0, "90")
        self.dc_dpos.pack(side="left", padx=6)
        ttk.Label(row1, text="Vel (deg/s):").pack(side="left")
        self.dc_vel = ttk.Entry(row1, width=10)
        self.dc_vel.insert(0, "120")
        self.dc_vel.pack(side="left", padx=6)
        ttk.Button(dc_box, text="Send DC Move", command=self.send_dc).pack(pady=6)

        # Stepper
        stp_box = ttk.LabelFrame(ctrl, text="Stepper", padding=10)
        stp_box.pack(fill="x", pady=6)

        row2 = ttk.Frame(stp_box)
        row2.pack(fill="x", pady=2)
        ttk.Label(row2, text="ΔAngle (deg):").pack(side="left")
        self.stp_dang = ttk.Entry(row2, width=10)
        self.stp_dang.insert(0, "45")
        self.stp_dang.pack(side="left", padx=6)
        ttk.Button(stp_box, text="Send Stepper Move", command=self.send_stepper).pack(pady=6)

        # Mode / Stop
        mode_box = ttk.LabelFrame(ctrl, text="Mode / System State", padding=10)
        mode_box.pack(fill="x", pady=6)

        row3 = ttk.Frame(mode_box)
        row3.pack(fill="x")
        ttk.Button(row3, text="MODE 1 (Sensor)", command=lambda: self.send_mode(1)).pack(side="left", padx=4)
        ttk.Button(row3, text="MODE 2 (GUI)", command=lambda: self.send_mode(2)).pack(side="left", padx=4)
        ttk.Button(row3, text="MODE 3 (STOP)", command=lambda: self.send_mode(3)).pack(side="left", padx=4)

        # ===== Log area =====
        log_frame = ttk.LabelFrame(self, text="Serial Log", padding=10)
        log_frame.pack(fill="both", expand=False, padx=10, pady=(0, 10))
        self.log = tk.Text(log_frame, height=8)
        self.log.pack(fill="both", expand=True)

        # Layout weights
        main.columnconfigure(0, weight=1)
        main.columnconfigure(1, weight=2)
        main.rowconfigure(0, weight=1)

    def _refresh_ports(self):
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.port_combo["values"] = ports
        if ports and self.port_var.get() not in ports:
            self.port_var.set(ports[0])

    def _log(self, msg: str):
        self.log.insert("end", msg + "\n")
        self.log.see("end")

    # ===== Serial =====
    def connect(self):
        if self.worker.is_connected():
            messagebox.showinfo("Info", "Already connected.")
            return
        port = self.port_var.get().strip()
        if not port:
            messagebox.showerror("Error", "No serial port selected.")
            return
        try:
            self.worker.connect(port, BAUD)
            self.status_var.set(f"Connected: {port} @ {BAUD}")
            self._log(f"[GUI] Connected to {port}")
        except Exception as e:
            messagebox.showerror("Connect failed", str(e))

    def disconnect(self):
        self.worker.disconnect()
        self.status_var.set("Disconnected")
        self._log("[GUI] Disconnected")

    def _poll_serial(self):
        try:
            while True:
                line = self.worker.rx_queue.get_nowait()
                self._handle_line(line)
        except Empty:
            pass
        self.after(50, self._poll_serial)

    def _handle_line(self, line: str):
        # Expected format:
        # STATE,s1,s2,s3,servo_deg,dc_pos_deg,dc_vel_dps,stepper_pos_deg,mode
        if line.startswith(STATE_PREFIX):
            parts = line.split(",")
            if len(parts) >= 9:
                self.s1_var.set(parts[1])
                self.s2_var.set(parts[2])
                self.s3_var.set(parts[3])
                self.servo_deg_var.set(parts[4])
                self.dc_pos_deg_var.set(parts[5])
                self.dc_vel_dps_var.set(parts[6])
                self.step_pos_deg_var.set(parts[7])
                self.mode_var.set(parts[8])
        else:
            self._log("[ARD] " + line)

    def _send(self, cmd: str):
        if not self.worker.is_connected():
            self._log("[GUI] Not connected.")
            return
        try:
            self.worker.send_line(cmd)
            self._log("[TX] " + cmd)
        except Exception as e:
            self._log("[GUI] Send failed: " + str(e))

    # ===== Commands =====
    def _set_servo_and_send(self, angle: int):
        self.servo_scale.set(angle)
        self.send_servo()

    def send_servo(self):
        angle = int(float(self.servo_scale.get()))
        self._send(f"SER,{angle}")

    def send_dc(self):
        try:
            dpos = float(self.dc_dpos.get())
            vel = float(self.dc_vel.get())
        except Exception:
            messagebox.showerror("Input error", "DC inputs must be numbers.")
            return
        self._send(f"DC,POS,{dpos},VEL,{vel}")

    def send_stepper(self):
        try:
            dang = float(self.stp_dang.get())
        except Exception:
            messagebox.showerror("Input error", "Stepper angle must be a number.")
            return
        self._send(f"STP,{dang}")

    def send_mode(self, m: int):
        self._send(f"MODE,{m}")

    def on_close(self):
        self.disconnect()
        self.destroy()


if __name__ == "__main__":
    app = App()
    app.protocol("WM_DELETE_WINDOW", app.on_close)
    app.mainloop()
