import tkinter as tk
from tkinter import ttk, messagebox
import serial
import serial.tools.list_ports
import threading
import time
from queue import Queue, Empty

from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
import collections

BAUD = 115200
READ_TIMEOUT = 0.1


class SerialWorker:
    def __init__(self):
        self.ser = None
        self.rx_queue = Queue()
        self._stop = threading.Event()

    def is_connected(self):
        return self.ser is not None and self.ser.is_open

    def connect(self, port, baud=BAUD):
        self.ser = serial.Serial(port, baud, timeout=READ_TIMEOUT)
        time.sleep(0.25)
        self._stop.clear()
        threading.Thread(target=self._reader, daemon=True).start()

    def disconnect(self):
        self._stop.set()
        if self.ser:
            self.ser.close()
            self.ser = None

    def send_line(self, line):
        if self.ser and self.ser.is_open:
            self.ser.write((line.strip() + "\n").encode())

    def _reader(self):
        while not self._stop.is_set():
            try:
                line = self.ser.readline().decode(errors="ignore").strip()
                if line:
                    self.rx_queue.put(line)
            except Exception:
                pass


class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("ILR Sensor-Motor GUI")
        self.geometry("1200x800")

        self.worker = SerialWorker()

        # GUI variables
        self.port_var = tk.StringVar()
        self.status_var = tk.StringVar(value="Disconnected")

        # Sensor values
        self.us_var = tk.StringVar(value="-")
        self.pot_var = tk.StringVar(value="-")
        self.pitch_var = tk.StringVar(value="-")

        # IMU raw
        self.ax_var = tk.StringVar(value="-")
        self.ay_var = tk.StringVar(value="-")
        self.az_var = tk.StringVar(value="-")
        self.gx_var = tk.StringVar(value="-")
        self.gy_var = tk.StringVar(value="-")
        self.gz_var = tk.StringVar(value="-")
        self.mx_var = tk.StringVar(value="-")
        self.my_var = tk.StringVar(value="-")
        self.mz_var = tk.StringVar(value="-")

        # Motor values
        self.servo_deg_var = tk.StringVar(value="-")
        self.dc_speed_var = tk.StringVar(value="-")
        self.step_pos_var = tk.StringVar(value="-")
        self.mode_var = tk.StringVar(value="-")

        # Graph buffers
        self.hist_len = 200
        self.hist_us = collections.deque(maxlen=self.hist_len)
        self.hist_pot = collections.deque(maxlen=self.hist_len)
        self.hist_pitch = collections.deque(maxlen=self.hist_len)
        self.hist_dc = collections.deque(maxlen=self.hist_len)
        self.hist_step = collections.deque(maxlen=self.hist_len)

        self._build_ui()
        self._refresh_ports()
        self.after(50, self._poll_serial)
        self.after(100, self._update_graphs)

    # ===================== UI =====================
    def _build_ui(self):
        top = ttk.Frame(self, padding=10)
        top.pack(fill="x")

        ttk.Label(top, text="Serial Port:").pack(side="left")
        self.port_combo = ttk.Combobox(top, textvariable=self.port_var, width=26, state="readonly")
        self.port_combo.pack(side="left", padx=6)

        ttk.Button(top, text="Refresh", command=self._refresh_ports).pack(side="left", padx=6)
        ttk.Button(top, text="Connect", command=self.connect).pack(side="left", padx=6)
        ttk.Button(top, text="Disconnect", command=self.disconnect).pack(side="left", padx=6)
        ttk.Label(top, textvariable=self.status_var).pack(side="left", padx=16)

        main = ttk.Frame(self, padding=10)
        main.pack(fill="both", expand=True)

        # ---------------- STATUS PANEL ----------------
        status = ttk.LabelFrame(main, text="Live Status", padding=10)
        status.grid(row=0, column=0, sticky="n", padx=(0, 10))

        def row(frame, r, label, var):
            ttk.Label(frame, text=label, width=20).grid(row=r, column=0, sticky="w")
            ttk.Label(frame, textvariable=var, width=18).grid(row=r, column=1, sticky="w")

        row(status, 0, "Ultrasonic (cm):", self.us_var)
        row(status, 1, "Potentiometer:", self.pot_var)
        row(status, 2, "Pitch (deg):", self.pitch_var)

        ttk.Separator(status).grid(row=3, columnspan=2, sticky="ew", pady=10)

        row(status, 4, "Servo angle:", self.servo_deg_var)
        row(status, 5, "DC speed:", self.dc_speed_var)
        row(status, 6, "Stepper pos:", self.step_pos_var)
        row(status, 7, "Mode:", self.mode_var)

        ttk.Separator(status).grid(row=8, columnspan=2, sticky="ew", pady=10)

        # IMU RAW
        row(status, 9, "ax:", self.ax_var)
        row(status, 10, "ay:", self.ay_var)
        row(status, 11, "az:", self.az_var)
        row(status, 12, "gx:", self.gx_var)
        row(status, 13, "gy:", self.gy_var)
        row(status, 14, "gz:", self.gz_var)
        row(status, 15, "mx:", self.mx_var)
        row(status, 16, "my:", self.my_var)
        row(status, 17, "mz:", self.mz_var)

        # ---------------- GRAPHS ----------------
        graph_frame = ttk.LabelFrame(main, text="Live Graphs", padding=10)
        graph_frame.grid(row=0, column=1, sticky="nsew")

        fig = Figure(figsize=(7, 6), dpi=100)
        self.ax1 = fig.add_subplot(511)
        self.ax2 = fig.add_subplot(512)
        self.ax3 = fig.add_subplot(513)
        self.ax4 = fig.add_subplot(514)
        self.ax5 = fig.add_subplot(515)

        self.canvas = FigureCanvasTkAgg(fig, master=graph_frame)
        self.canvas.get_tk_widget().pack(fill="both", expand=True)

        # ---------------- LOG ----------------
        log_frame = ttk.LabelFrame(self, text="Serial Log", padding=10)
        log_frame.pack(fill="both", padx=10, pady=(0, 10))

        self.log = tk.Text(log_frame, height=8)
        self.log.pack(fill="both", expand=True)

    # ===================== Serial =====================
    def _refresh_ports(self):
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.port_combo["values"] = ports
        if ports:
            self.port_var.set(ports[0])

    def connect(self):
        try:
            self.worker.connect(self.port_var.get())
            self.status_var.set("Connected")
            self._log("[GUI] Connected")
        except Exception as e:
            messagebox.showerror("Error", str(e))

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

    # ===================== Helpers =====================
    def _set_if_float(self, var, val, hist=None, fmt="{:.2f}"):
        try:
            f = float(val)
        except Exception:
            return
        var.set(fmt.format(f))
        if hist is not None:
            hist.append(f)

    # ===================== Parsing =====================
    def _handle_line(self, line):
        self._log("[ARD] " + line)

        tokens = [t.strip() for t in line.split(",") if t.strip()]
        if not tokens:
            return

        # First: handle well-formed IMU lines explicitly
        # Expected from Arduino:
        # IMU,ax,ay,az,gx,gy,gz,mx,my,mz,PITCH_DEG,pitch,STEPPER_TGT,... etc.
        if tokens[0] == "IMU":
            if len(tokens) >= 12:
                try:
                    ax = tokens[1]
                    ay = tokens[2]
                    az = tokens[3]
                    gx = tokens[4]
                    gy = tokens[5]
                    gz = tokens[6]
                    mx = tokens[7]
                    my = tokens[8]
                    mz = tokens[9]
                    pitch_val = tokens[11]

                    self._set_if_float(self.ax_var, ax)
                    self._set_if_float(self.ay_var, ay)
                    self._set_if_float(self.az_var, az)
                    self._set_if_float(self.gx_var, gx)
                    self._set_if_float(self.gy_var, gy)
                    self._set_if_float(self.gz_var, gz)
                    self._set_if_float(self.mx_var, mx)
                    self._set_if_float(self.my_var, my)
                    self._set_if_float(self.mz_var, mz)
                    self._set_if_float(self.pitch_var, pitch_val, self.hist_pitch)
                except Exception:
                    pass

        # Generic key/value scan
        for i in range(len(tokens) - 1):
            raw_key = tokens[i]
            val = tokens[i + 1]

            # Normalize keys that sometimes get glued to "CW"
            if raw_key.endswith("POT"):
                key = "POT"
            elif raw_key.endswith("US_CM"):
                key = "US_CM"
            else:
                key = raw_key

            # Sensors
            if key == "US_CM":
                self._set_if_float(self.us_var, val, self.hist_us)

            elif key == "POT":
                self._set_if_float(self.pot_var, val, self.hist_pot, fmt="{:.0f}")

            elif key == "PITCH_DEG":
                self._set_if_float(self.pitch_var, val, self.hist_pitch)

            # Motors
            elif key == "SERVO_CMD":
                self._set_if_float(self.servo_deg_var, val, fmt="{:.0f}")

            elif key == "STEPPER_POS":
                self._set_if_float(self.step_pos_var, val, self.hist_step, fmt="{:.0f}")

            elif key == "DCM_SPEED":
                self._set_if_float(self.dc_speed_var, val, self.hist_dc, fmt="{:.0f}")

            elif key == "MOTORS":
                self.mode_var.set(val)

    # ===================== Graph Updates =====================
    def _update_graphs(self):
        self.ax1.clear()
        self.ax2.clear()
        self.ax3.clear()
        self.ax4.clear()
        self.ax5.clear()

        if self.hist_us:
            self.ax1.plot(self.hist_us)
        self.ax1.set_ylabel("US (cm)")

        if self.hist_pot:
            self.ax2.plot(self.hist_pot)
        self.ax2.set_ylabel("POT")

        if self.hist_pitch:
            self.ax3.plot(self.hist_pitch)
        self.ax3.set_ylabel("Pitch")

        if self.hist_dc:
            self.ax4.plot(self.hist_dc)
        self.ax4.set_ylabel("DC Speed")

        if self.hist_step:
            self.ax5.plot(self.hist_step)
        self.ax5.set_ylabel("Stepper")

        self.canvas.draw()
        self.after(100, self._update_graphs)

    # ===================== Utils =====================
    def _log(self, msg):
        self.log.insert("end", msg + "\n")
        self.log.see("end")


if __name__ == "__main__":
    app = App()
    app.mainloop()
