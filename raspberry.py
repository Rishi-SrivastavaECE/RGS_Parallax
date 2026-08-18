"""
PHASE 2 — ATC Integrated Approach Analysis
===========================================

Inputs
------
GROUND / CENTER ESP32 over UDP :8889
    ILS,A101,L,C,R,ALIGNMENT,GLIDE

AIRCRAFT RIG over Bluetooth serial
    A101,ROLL,PITCH

The Center ESP32 remains responsible for the instantaneous ILS decision.
The Raspberry Pi/laptop performs higher-level temporal analysis:

1. RSSI history and first differences
2. RSSI autocorrelation / stability
3. RSSI change cross-correlation
4. Ambiguous-centerline detection
5. Ultrasonic glide-state monitoring
6. Guidance-event detection
7. Aircraft roll/pitch response detection
8. Guidance -> aircraft response delay
9. Guidance -> RSSI response consistency
10. Combined approach confidence
11. Pilot/aircraft response status

IMPORTANT
---------
This is a prototype research/hackathon analysis layer, not certified
aviation guidance logic.
"""

import socket
import threading
import time
import math
import tkinter as tk
from tkinter import ttk

# ST7735 TFT support
try:
    from PIL import Image, ImageDraw, ImageFont
    import st7735
    TFT_AVAILABLE = True
except ImportError:
    TFT_AVAILABLE = False

try:
    import serial
except ImportError:
    serial = None

# ================================================================
# CONFIG
# ================================================================

UDP_PORT = 8889
BT_BAUD = 9600
STALE_SECONDS = 2.5

HISTORY_SIZE = 120

# A guidance event is considered new when alignment changes.
# We also create a new event when a correction remains active and
# the previous event has been completed.
RESPONSE_WINDOW = 5.0

# Minimum roll/pitch change considered a response.
ANGLE_RESPONSE_THRESHOLD = 2.0

# Minimum RSSI change in dB considered meaningful.
RSSI_RESPONSE_THRESHOLD = 0.8

# ================================================================
# TFT CONFIGURATION
# ================================================================
# Raspberry Pi ST7735 1.8" / 160x128 display.
# Set TFT_ENABLED = False if you want laptop-only operation.
TFT_ENABLED = True

TFT_PORT = 0
TFT_CS = 0
TFT_DC = 24
TFT_RST = 25
TFT_WIDTH = 128
TFT_HEIGHT = 160
TFT_ROTATION = 90
TFT_OFFSET_LEFT = 0
TFT_OFFSET_TOP = 0
TFT_INVERT = False
TFT_BGR = True
TFT_SPI_SPEED = 4000000


# ================================================================
# HELPERS
# ================================================================

def mean(values):
    return sum(values) / len(values) if values else 0.0


def pearson(x, y):
    n = min(len(x), len(y))
    if n < 2:
        return 0.0

    x = list(x)[-n:]
    y = list(y)[-n:]

    mx = mean(x)
    my = mean(y)

    numerator = sum((a - mx) * (b - my) for a, b in zip(x, y))

    dx = math.sqrt(sum((a - mx) ** 2 for a in x))
    dy = math.sqrt(sum((b - my) ** 2 for b in y))

    if dx == 0 or dy == 0:
        return 0.0

    return numerator / (dx * dy)


def autocorrelation(values, lag=1):
    if len(values) < 10 or len(values) <= lag:
        return None

    return pearson(values[:-lag], values[lag:])


def differences(values):
    if len(values) < 2:
        return []

    return [
        values[i] - values[i - 1]
        for i in range(1, len(values))
    ]


# ================================================================
# APPLICATION
# ================================================================

class Phase2ATC:

    def __init__(self, root):

        self.root = root
        self.root.title("ATC — Phase 2 Integrated Approach Analysis")
        self.root.geometry("1280x760")
        self.root.minsize(1000, 650)

        self.running = True
        self.lock = threading.Lock()

        # --------------------------------------------------------
        # Ground telemetry
        # --------------------------------------------------------

        self.ground = {
            "id": "A101",
            "l": -99.0,
            "c": -99.0,
            "r": -99.0,
            "align": "WAITING",
            "glide": "UNKNOWN",
            "last": 0.0,
        }

        # --------------------------------------------------------
        # Aircraft telemetry
        # --------------------------------------------------------

        self.aircraft = {
            "id": "A101",
            "roll": None,
            "pitch": None,
            "last": 0.0,
        }

        # --------------------------------------------------------
        # Histories
        # --------------------------------------------------------

        self.rssi = {
            "L": [],
            "C": [],
            "R": [],
        }

        self.roll_history = []
        self.pitch_history = []

        self.time_history = []

        # --------------------------------------------------------
        # Previous states
        # --------------------------------------------------------

        self.previous_alignment = "WAITING"

        # --------------------------------------------------------
        # Guidance event
        # --------------------------------------------------------

        self.guidance_event = {
            "active": False,
            "command": None,
            "time": 0.0,
            "baseline_roll": None,
            "baseline_pitch": None,
            "baseline_rssi": None,
            "rssi_response": False,
            "aircraft_response": False,
            "response_time": None,
            "completed": False,
        }

        # --------------------------------------------------------
        # UI
        # --------------------------------------------------------

        self.build_style()
        self.build_ui()

        # --------------------------------------------------------
        # TFT
        # --------------------------------------------------------
        self.tft = None
        self.tft_font_s = None
        self.tft_font_m = None
        self.init_tft()

        # --------------------------------------------------------
        # Threads
        # --------------------------------------------------------

        threading.Thread(
            target=self.udp_listener,
            daemon=True
        ).start()

        self.bt_serial = None
        self.bt_thread = None

        self.root.protocol(
            "WM_DELETE_WINDOW",
            self.close
        )

        self.update_ui()

    # ============================================================
    # STYLE
    # ============================================================

    def build_style(self):

        self.colors = {
            "bg": "#09111f",
            "card": "#111c2e",
            "panel": "#0d1728",
            "text": "#edf3fa",
            "muted": "#8fa2bb",
            "green": "#22c55e",
            "amber": "#f59e0b",
            "red": "#ef4444",
            "blue": "#38bdf8",
            "purple": "#a78bfa",
            "border": "#27384f",
        }

        self.root.configure(
            bg=self.colors["bg"]
        )

    # ============================================================
    # UI BUILD
    # ============================================================

    def build_ui(self):

        main = tk.Frame(
            self.root,
            bg=self.colors["bg"],
            padx=24,
            pady=20
        )

        main.pack(
            fill="both",
            expand=True
        )

        # --------------------------------------------------------
        # Header
        # --------------------------------------------------------

        header = tk.Frame(
            main,
            bg=self.colors["bg"]
        )

        header.pack(
            fill="x",
            pady=(0, 18)
        )

        tk.Label(
            header,
            text="ATC  /  PHASE 2 INTEGRATED APPROACH ANALYSIS",
            bg=self.colors["bg"],
            fg=self.colors["text"],
            font=("Segoe UI Semibold", 21)
        ).pack(side="left")

        self.system_status = tk.Label(
            header,
            text="● WAITING",
            bg=self.colors["bg"],
            fg=self.colors["amber"],
            font=("Segoe UI Semibold", 11)
        )

        self.system_status.pack(
            side="right"
        )

        # --------------------------------------------------------
        # Top panels
        # --------------------------------------------------------

        top = tk.Frame(
            main,
            bg=self.colors["bg"]
        )

        top.pack(
            fill="x"
        )

        ground = self.make_card(top)
        ground.pack(
            side="left",
            fill="both",
            expand=True,
            padx=(0, 8)
        )

        aircraft = self.make_card(top)
        aircraft.pack(
            side="left",
            fill="both",
            expand=True,
            padx=(8, 0)
        )

        self.build_ground_panel(ground)
        self.build_aircraft_panel(aircraft)

        # --------------------------------------------------------
        # Analysis panel
        # --------------------------------------------------------

        analysis = self.make_card(main)

        analysis.pack(
            fill="both",
            expand=True,
            pady=(16, 0)
        )

        tk.Label(
            analysis,
            text="INTEGRATED RESPONSE ANALYSIS",
            bg=self.colors["card"],
            fg=self.colors["text"],
            font=("Segoe UI Semibold", 15)
        ).pack(
            anchor="w",
            padx=18,
            pady=(15, 0)
        )

        grid = tk.Frame(
            analysis,
            bg=self.colors["card"]
        )

        grid.pack(
            fill="both",
            expand=True,
            padx=18,
            pady=12
        )

        # Analysis metrics
        self.metrics = {}

        metric_names = [
            "LATERAL CONFIDENCE",
            "VERTICAL STATUS",
            "RSSI STABILITY",
            "RSSI TREND",
            "AIRCRAFT RESPONSE",
            "RESPONSE DELAY",
            "RSSI RESPONSE",
            "OVERALL ASSESSMENT",
        ]

        for i, name in enumerate(metric_names):

            row = i // 2
            col = i % 2

            frame = tk.Frame(
                grid,
                bg=self.colors["panel"],
                highlightbackground=self.colors["border"],
                highlightthickness=1
            )

            frame.grid(
                row=row,
                column=col,
                sticky="nsew",
                padx=5,
                pady=5
            )

            grid.grid_columnconfigure(
                col,
                weight=1
            )

            grid.grid_rowconfigure(
                row,
                weight=1
            )

            tk.Label(
                frame,
                text=name,
                bg=self.colors["panel"],
                fg=self.colors["muted"],
                font=("Segoe UI Semibold", 9)
            ).pack(
                anchor="w",
                padx=12,
                pady=(9, 0)
            )

            label = tk.Label(
                frame,
                text="WAITING",
                bg=self.colors["panel"],
                fg=self.colors["text"],
                font=("Segoe UI Semibold", 14)
            )

            label.pack(
                anchor="w",
                padx=12,
                pady=(4, 9)
            )

            self.metrics[name] = label

        # --------------------------------------------------------
        # Event log
        # --------------------------------------------------------

        self.event_log = tk.Text(
            analysis,
            height=5,
            bg="#07101d",
            fg=self.colors["muted"],
            insertbackground=self.colors["text"],
            relief="flat",
            font=("Consolas", 9)
        )

        self.event_log.pack(
            fill="x",
            padx=18,
            pady=(0, 15)
        )

    # ============================================================
    # PANELS
    # ============================================================

    def make_card(self, parent):

        return tk.Frame(
            parent,
            bg=self.colors["card"],
            highlightbackground=self.colors["border"],
            highlightthickness=1
        )

    def build_ground_panel(self, panel):

        tk.Label(
            panel,
            text="GROUND ILS",
            bg=self.colors["card"],
            fg=self.colors["text"],
            font=("Segoe UI Semibold", 15)
        ).pack(
            anchor="w",
            padx=18,
            pady=(15, 0)
        )

        tk.Label(
            panel,
            text="Center ESP32 / three-node RF guidance",
            bg=self.colors["card"],
            fg=self.colors["muted"],
            font=("Segoe UI", 9)
        ).pack(
            anchor="w",
            padx=18
        )

        self.ground_values = {}

        row = tk.Frame(
            panel,
            bg=self.colors["card"]
        )

        row.pack(
            fill="x",
            padx=18,
            pady=15
        )

        for node in ("L", "C", "R"):

            box = tk.Frame(
                row,
                bg=self.colors["panel"],
                highlightbackground=self.colors["border"],
                highlightthickness=1
            )

            box.pack(
                side="left",
                fill="x",
                expand=True,
                padx=3
            )

            tk.Label(
                box,
                text={"L":"LEFT","C":"CENTER","R":"RIGHT"}[node],
                bg=self.colors["panel"],
                fg=self.colors["muted"],
                font=("Segoe UI Semibold", 8)
            ).pack(pady=(7, 0))

            label = tk.Label(
                box,
                text="-- dBm",
                bg=self.colors["panel"],
                fg=self.colors["text"],
                font=("Consolas", 17, "bold")
            )

            label.pack(pady=(3, 8))

            self.ground_values[node] = label

        self.ground_align = tk.Label(
            panel,
            text="ALIGNMENT: WAITING",
            bg=self.colors["card"],
            fg=self.colors["amber"],
            font=("Segoe UI Semibold", 14)
        )

        self.ground_align.pack(
            anchor="w",
            padx=18,
            pady=(2, 4)
        )

        self.ground_glide = tk.Label(
            panel,
            text="GLIDE: UNKNOWN",
            bg=self.colors["card"],
            fg=self.colors["muted"],
            font=("Segoe UI Semibold", 12)
        )

        self.ground_glide.pack(
            anchor="w",
            padx=18,
            pady=(0, 15)
        )

    def build_aircraft_panel(self, panel):

        tk.Label(
            panel,
            text="AIRCRAFT A101",
            bg=self.colors["card"],
            fg=self.colors["text"],
            font=("Segoe UI Semibold", 15)
        ).pack(
            anchor="w",
            padx=18,
            pady=(15, 0)
        )

        tk.Label(
            panel,
            text="Arduino + MPU6050 / Bluetooth",
            bg=self.colors["card"],
            fg=self.colors["muted"],
            font=("Segoe UI", 9)
        ).pack(
            anchor="w",
            padx=18
        )

        controls = tk.Frame(
            panel,
            bg=self.colors["card"]
        )

        controls.pack(
            fill="x",
            padx=18,
            pady=10
        )

        tk.Label(
            controls,
            text="COM:",
            bg=self.colors["card"],
            fg=self.colors["muted"]
        ).pack(side="left")

        self.com_var = tk.StringVar(
            value="COM7"
        )

        tk.Entry(
            controls,
            textvariable=self.com_var,
            width=10,
            bg=self.colors["panel"],
            fg=self.colors["text"],
            insertbackground=self.colors["text"],
            relief="flat"
        ).pack(
            side="left",
            padx=7,
            ipady=4
        )

        self.bt_button = tk.Button(
            controls,
            text="CONNECT",
            command=self.connect_bluetooth,
            bg=self.colors["blue"],
            fg="#06111d",
            relief="flat",
            padx=10
        )

        self.bt_button.pack(
            side="left"
        )

        self.bt_status = tk.Label(
            controls,
            text="● OFFLINE",
            bg=self.colors["card"],
            fg=self.colors["muted"]
        )

        self.bt_status.pack(
            side="right"
        )

        attitude = tk.Frame(
            panel,
            bg=self.colors["card"]
        )

        attitude.pack(
            fill="x",
            padx=18,
            pady=(4, 15)
        )

        self.roll_label = self.attitude_box(
            attitude,
            "ROLL"
        )

        self.pitch_label = self.attitude_box(
            attitude,
            "PITCH"
        )

    def attitude_box(self, parent, title):

        frame = tk.Frame(
            parent,
            bg=self.colors["panel"],
            highlightbackground=self.colors["border"],
            highlightthickness=1
        )

        frame.pack(
            side="left",
            fill="x",
            expand=True,
            padx=3
        )

        tk.Label(
            frame,
            text=title,
            bg=self.colors["panel"],
            fg=self.colors["muted"],
            font=("Segoe UI Semibold", 9)
        ).pack(
            pady=(8, 0)
        )

        label = tk.Label(
            frame,
            text="--°",
            bg=self.colors["panel"],
            fg=self.colors["text"],
            font=("Consolas", 22, "bold")
        )

        label.pack(
            pady=(2, 9)
        )

        return label

    # ============================================================
    # TFT
    # ============================================================

    def init_tft(self):
        if not TFT_ENABLED:
            self.log("[TFT] Disabled")
            return

        if not TFT_AVAILABLE:
            self.log("[TFT] PIL/st7735 not available; laptop UI still running")
            return

        try:
            self.tft = st7735.ST7735(
                port=TFT_PORT,
                cs=TFT_CS,
                dc=TFT_DC,
                rst=TFT_RST,
                width=TFT_WIDTH,
                height=TFT_HEIGHT,
                rotation=TFT_ROTATION,
                offset_left=TFT_OFFSET_LEFT,
                offset_top=TFT_OFFSET_TOP,
                invert=TFT_INVERT,
                bgr=TFT_BGR,
                spi_speed_hz=TFT_SPI_SPEED,
            )
            self.tft.begin()

            self.tft_font_s = self.load_tft_font(9)
            self.tft_font_m = self.load_tft_font(11)

            self.log("[TFT] ST7735 initialized")

        except Exception as e:
            self.tft = None
            self.log(f"[TFT ERROR] {e}")

    def load_tft_font(self, size):
        try:
            return ImageFont.truetype(
                "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf",
                size
            )
        except Exception:
            return ImageFont.load_default()

    def draw_tft(self, analysis, ground, aircraft):
        if self.tft is None:
            return

        try:
            WIDTH, HEIGHT = 160, 128

            bg = (15, 23, 42)
            card = (30, 41, 59)
            white = (241, 245, 249)
            muted = (148, 163, 184)
            green = (34, 197, 94)
            amber = (245, 158, 11)
            red = (239, 68, 68)
            blue = (56, 189, 248)

            img = Image.new("RGB", (WIDTH, HEIGHT), color=bg)
            draw = ImageDraw.Draw(img)

            ground_ok = analysis["ground_ok"]
            aircraft_ok = analysis["aircraft_ok"]

            # Header
            draw.rectangle((2, 2, 158, 18), fill=card)
            draw.text(
                (6, 4),
                f"ATC | {ground['id']}",
                fill=white,
                font=self.tft_font_s
            )

            if ground_ok and aircraft_ok:
                dot = green
            elif ground_ok or aircraft_ok:
                dot = amber
            else:
                dot = red

            draw.ellipse((146, 6, 154, 14), fill=dot)

            # RSSI
            draw.rectangle((2, 22, 158, 48), fill=card)
            draw.text(
                (6, 24),
                "GROUND RSSI",
                fill=muted,
                font=self.tft_font_s
            )
            draw.text(
                (6, 35),
                f"L:{ground['l']:.0f} C:{ground['c']:.0f} R:{ground['r']:.0f}",
                fill=white,
                font=self.tft_font_s
            )

            # Alignment
            align = ground["align"] if ground_ok else "LINK LOST"

            if "CENTER" in align:
                a_color = green
            elif "CORRECT" in align:
                a_color = amber
            else:
                a_color = red

            draw.rectangle((2, 52, 78, 82), fill=card)
            draw.text(
                (6, 55),
                "ALIGN",
                fill=muted,
                font=self.tft_font_s
            )
            draw.text(
                (6, 67),
                align[:11],
                fill=a_color,
                font=self.tft_font_s
            )

            # Glide
            glide = ground["glide"] if ground_ok else "LINK LOST"

            if glide == "ON GLIDE":
                g_color = green
            elif glide in ("TOO HIGH", "TOO LOW"):
                g_color = red
            else:
                g_color = amber

            draw.rectangle((82, 52, 158, 82), fill=card)
            draw.text(
                (86, 55),
                "GLIDE",
                fill=muted,
                font=self.tft_font_s
            )
            draw.text(
                (86, 67),
                glide[:10],
                fill=g_color,
                font=self.tft_font_s
            )

            # Aircraft attitude
            draw.rectangle((2, 86, 78, 124), fill=card)
            draw.text(
                (6, 89),
                "AIRCRAFT",
                fill=muted,
                font=self.tft_font_s
            )

            if aircraft["roll"] is not None:
                attitude = f"R {aircraft['roll']:+.1f}"
                attitude2 = f"P {aircraft['pitch']:+.1f}"
            else:
                attitude = "R --"
                attitude2 = "P --"

            draw.text(
                (6, 101),
                attitude,
                fill=white,
                font=self.tft_font_s
            )
            draw.text(
                (40, 101),
                attitude2,
                fill=white,
                font=self.tft_font_s
            )

            # Integrated result
            draw.rectangle((82, 86, 158, 124), fill=card)
            draw.text(
                (86, 89),
                "ANALYSIS",
                fill=muted,
                font=self.tft_font_s
            )

            overall = analysis["overall"]
            overall_color = (
                green if overall >= 80
                else amber if overall >= 35
                else red
            )

            draw.text(
                (86, 100),
                f"CONF {overall}%",
                fill=overall_color,
                font=self.tft_font_s
            )

            if analysis["aircraft_response"]:
                response_text = "RESP YES"
            elif analysis["response"] == "NO AIRCRAFT RESPONSE":
                response_text = "RESP NO"
            else:
                response_text = "RESP --"

            draw.text(
                (86, 112),
                response_text,
                fill=white,
                font=self.tft_font_s
            )

            self.tft.display(img)

        except Exception as e:
            # Don't kill the analysis/UI if the TFT has a transient SPI problem.
            self.log(f"[TFT ERROR] {e}")

    # ============================================================
    # UDP RECEIVER
    # ============================================================

    def udp_listener(self):

        sock = socket.socket(
            socket.AF_INET,
            socket.SOCK_DGRAM
        )

        sock.setsockopt(
            socket.SOL_SOCKET,
            socket.SO_REUSEADDR,
            1
        )

        try:
            sock.bind(
                ("0.0.0.0", UDP_PORT)
            )

            self.log(
                f"[UDP] Listening on :{UDP_PORT}"
            )

        except Exception as e:

            self.log(
                f"[UDP ERROR] {e}"
            )

            return

        while self.running:

            try:

                data, _ = sock.recvfrom(2048)

                p = data.decode(
                    "utf-8",
                    errors="ignore"
                ).strip().split(",")

                if len(p) < 7:
                    continue

                if p[0] != "ILS":
                    continue

                with self.lock:

                    old_alignment = self.ground["align"]

                    self.ground["id"] = p[1]
                    self.ground["l"] = float(p[2])
                    self.ground["c"] = float(p[3])
                    self.ground["r"] = float(p[4])
                    self.ground["align"] = p[5]
                    self.ground["glide"] = p[6]
                    self.ground["last"] = time.time()

                    self.rssi["L"].append(
                        self.ground["l"]
                    )

                    self.rssi["C"].append(
                        self.ground["c"]
                    )

                    self.rssi["R"].append(
                        self.ground["r"]
                    )

                    self.trim_history()

                    new_alignment = self.ground["align"]

                    # Detect guidance event.
                    if new_alignment != old_alignment:

                        self.start_guidance_event(
                            new_alignment
                        )

            except Exception as e:

                self.log(
                    f"[UDP] {e}"
                )

    # ============================================================
    # BLUETOOTH
    # ============================================================

    def connect_bluetooth(self):

        if serial is None:

            self.log(
                "[BT] Install pyserial first: pip install pyserial"
            )

            return

        port = self.com_var.get().strip()

        try:

            self.bt_serial = serial.Serial(
                port=port,
                baudrate=BT_BAUD,
                timeout=1
            )

            self.bt_status.config(
                text="● CONNECTED",
                fg=self.colors["green"]
            )

            self.bt_button.config(
                text="CONNECTED"
            )

            self.bt_thread = threading.Thread(
                target=self.bluetooth_listener,
                daemon=True
            )

            self.bt_thread.start()

            self.log(
                f"[BT] Connected to {port}"
            )

        except Exception as e:

            self.bt_status.config(
                text="● FAILED",
                fg=self.colors["red"]
            )

            self.log(
                f"[BT ERROR] {e}"
            )

    def bluetooth_listener(self):

        while self.running and self.bt_serial:

            try:

                line = self.bt_serial.readline().decode(
                    "utf-8",
                    errors="ignore"
                ).strip()

                if not line:
                    continue

                p = line.split(",")

                if len(p) != 3:
                    continue

                aircraft_id = p[0]
                roll = float(p[1])
                pitch = float(p[2])

                with self.lock:

                    self.aircraft["id"] = aircraft_id
                    self.aircraft["roll"] = roll
                    self.aircraft["pitch"] = pitch
                    self.aircraft["last"] = time.time()

                    self.roll_history.append(
                        roll
                    )

                    self.pitch_history.append(
                        pitch
                    )

                    self.trim_history()

            except Exception as e:

                self.log(
                    f"[BT] {e}"
                )

    # ============================================================
    # HISTORY
    # ============================================================

    def trim_history(self):

        for key in self.rssi:

            if len(self.rssi[key]) > HISTORY_SIZE:
                self.rssi[key] = self.rssi[key][-HISTORY_SIZE:]

        if len(self.roll_history) > HISTORY_SIZE:
            self.roll_history = self.roll_history[-HISTORY_SIZE:]

        if len(self.pitch_history) > HISTORY_SIZE:
            self.pitch_history = self.pitch_history[-HISTORY_SIZE:]

    # ============================================================
    # GUIDANCE EVENT
    # ============================================================

    def start_guidance_event(self, command):

        if command not in (
            "CORRECT LEFT",
            "CORRECT RIGHT",
            "CENTERLINE"
        ):
            return

        now = time.time()

        baseline_roll = self.aircraft["roll"]
        baseline_pitch = self.aircraft["pitch"]

        baseline_rssi = {
            "L": self.ground["l"],
            "C": self.ground["c"],
            "R": self.ground["r"],
        }

        self.guidance_event = {
            "active": command != "CENTERLINE",
            "command": command,
            "time": now,
            "baseline_roll": baseline_roll,
            "baseline_pitch": baseline_pitch,
            "baseline_rssi": baseline_rssi,
            "rssi_response": False,
            "aircraft_response": False,
            "response_time": None,
            "completed": False,
        }

        self.log(
            f"[GUIDANCE] {command}"
        )

    # ============================================================
    # PHASE 2 ANALYSIS
    # ============================================================

    def analyze(self):

        now = time.time()

        ground_ok = (
            self.ground["last"] > 0
            and now - self.ground["last"] < STALE_SECONDS
        )

        aircraft_ok = (
            self.aircraft["last"] > 0
            and now - self.aircraft["last"] < STALE_SECONDS
        )

        # --------------------------------------------------------
        # Lateral stability
        # --------------------------------------------------------

        ac_values = []

        for node in ("L", "C", "R"):

            ac = autocorrelation(
                self.rssi[node]
            )

            if ac is not None:
                ac_values.append(ac)

        if len(ac_values) == 3:

            avg_ac = mean(ac_values)

            if avg_ac >= 0.70:
                stability = "HIGH"
            elif avg_ac >= 0.35:
                stability = "MEDIUM"
            else:
                stability = "LOW"

        else:

            stability = "ACQUIRING"

        # --------------------------------------------------------
        # Ambiguous centerline
        # --------------------------------------------------------

        ambiguous = False

        if self.ground["align"] == "CENTERLINE":

            l = self.ground["l"]
            c = self.ground["c"]
            r = self.ground["r"]

            if (
                abs(l - r) <= 4
                and ((l + r) / 2 - c) >= 6
            ):
                ambiguous = True

        # --------------------------------------------------------
        # RSSI trend during correction
        # --------------------------------------------------------

        command = self.guidance_event["command"]

        trend = 0.0

        if command == "CORRECT LEFT":

            d = differences(
                self.rssi["L"]
            )

            if d:
                trend = mean(d[-8:])

        elif command == "CORRECT RIGHT":

            d = differences(
                self.rssi["R"]
            )

            if d:
                trend = mean(d[-8:])

        # --------------------------------------------------------
        # RSSI response
        # --------------------------------------------------------

        if self.guidance_event["active"]:

            elapsed = (
                now -
                self.guidance_event["time"]
            )

            if elapsed <= RESPONSE_WINDOW:

                base = self.guidance_event["baseline_rssi"]

                if command == "CORRECT LEFT":

                    change = (
                        self.ground["l"] -
                        base["L"]
                    )

                elif command == "CORRECT RIGHT":

                    change = (
                        self.ground["r"] -
                        base["R"]
                    )

                else:

                    change = 0

                if change >= RSSI_RESPONSE_THRESHOLD:

                    self.guidance_event[
                        "rssi_response"
                    ] = True

        # --------------------------------------------------------
        # Aircraft response
        # --------------------------------------------------------

        if (
            self.guidance_event["active"]
            and aircraft_ok
            and self.aircraft["roll"] is not None
        ):

            elapsed = (
                now -
                self.guidance_event["time"]
            )

            if elapsed <= RESPONSE_WINDOW:

                base_roll = (
                    self.guidance_event[
                        "baseline_roll"
                    ]
                )

                if base_roll is not None:

                    roll_change = abs(
                        self.aircraft["roll"]
                        - base_roll
                    )

                    if (
                        roll_change
                        >= ANGLE_RESPONSE_THRESHOLD
                    ):

                        if not self.guidance_event[
                            "aircraft_response"
                        ]:

                            self.guidance_event[
                                "response_time"
                            ] = elapsed

                        self.guidance_event[
                            "aircraft_response"
                        ] = True

        # --------------------------------------------------------
        # Response status
        # --------------------------------------------------------

        if not ground_ok:

            response = "GROUND LINK LOST"

        elif not aircraft_ok:

            response = "AIRCRAFT DATA WAITING"

        elif not self.guidance_event["active"]:

            response = "MONITORING"

        elif self.guidance_event[
            "aircraft_response"
        ]:

            response = "AIRCRAFT RESPONSE DETECTED"

        elif (
            now -
            self.guidance_event["time"]
            > RESPONSE_WINDOW
        ):

            response = "NO AIRCRAFT RESPONSE"

        else:

            response = "WAITING FOR RESPONSE"

        # --------------------------------------------------------
        # Lateral confidence
        # --------------------------------------------------------

        if not ground_ok:

            lateral_conf = 0

        elif len(ac_values) < 3:

            lateral_conf = 20

        else:

            lateral_conf = 60

            if stability == "HIGH":
                lateral_conf += 20
            elif stability == "MEDIUM":
                lateral_conf += 10

            if ambiguous:
                lateral_conf -= 30

            if command in (
                "CORRECT LEFT",
                "CORRECT RIGHT"
            ):

                if trend >= 0.5:
                    lateral_conf += 10

                elif trend <= -0.5:
                    lateral_conf -= 10

            lateral_conf = max(
                0,
                min(100, lateral_conf)
            )

        # --------------------------------------------------------
        # Overall assessment
        # --------------------------------------------------------

        overall = lateral_conf

        if self.ground["glide"] == "ON GLIDE":
            overall += 10

        elif self.ground["glide"] in (
            "TOO HIGH",
            "TOO LOW"
        ):
            overall -= 20

        elif self.ground["glide"] == "NO ECHO":
            overall -= 10

        # Aircraft response becomes useful evidence,
        # but absence of response should not instantly declare
        # the aircraft unsafe.
        if self.guidance_event["active"]:

            if self.guidance_event[
                "aircraft_response"
            ]:
                overall += 10

            if self.guidance_event[
                "rssi_response"
            ]:
                overall += 5

        overall = max(
            0,
            min(100, int(overall))
        )

        if overall >= 80:
            assessment = "HIGH CONFIDENCE"

        elif overall >= 60:
            assessment = "MODERATE CONFIDENCE"

        elif overall >= 35:
            assessment = "LOW CONFIDENCE"

        else:
            assessment = "UNTRUSTWORTHY"

        return {
            "ground_ok": ground_ok,
            "aircraft_ok": aircraft_ok,
            "stability": stability,
            "ambiguous": ambiguous,
            "trend": trend,
            "lateral_conf": lateral_conf,
            "response": response,
            "rssi_response": self.guidance_event[
                "rssi_response"
            ],
            "aircraft_response": self.guidance_event[
                "aircraft_response"
            ],
            "response_time": self.guidance_event[
                "response_time"
            ],
            "overall": overall,
            "assessment": assessment,
        }

    # ============================================================
    # UI UPDATE
    # ============================================================

    def update_ui(self):

        if not self.running:
            return

        with self.lock:

            analysis = self.analyze()

            g = dict(self.ground)
            a = dict(self.aircraft)

        # The TFT consumes the exact same analysis result as the laptop UI.
        # No duplicate analysis is performed on the TFT.
        self.draw_tft(analysis, g, a)

        # --------------------------------------------------------
        # System
        # --------------------------------------------------------

        if (
            analysis["ground_ok"]
            and analysis["aircraft_ok"]
        ):

            self.system_status.config(
                text="● INTEGRATED TELEMETRY ONLINE",
                fg=self.colors["green"]
            )

        elif analysis["ground_ok"]:

            self.system_status.config(
                text="● GROUND ONLINE / AIRCRAFT OFFLINE",
                fg=self.colors["amber"]
            )

        elif analysis["aircraft_ok"]:

            self.system_status.config(
                text="● AIRCRAFT ONLINE / GROUND OFFLINE",
                fg=self.colors["amber"]
            )

        else:

            self.system_status.config(
                text="● TELEMETRY OFFLINE",
                fg=self.colors["red"]
            )

        # --------------------------------------------------------
        # Ground
        # --------------------------------------------------------

        for node in ("L", "C", "R"):

            self.ground_values[node].config(
                text=f'{g[node.lower()]:.0f} dBm'
            )

        align_color = (
            self.colors["green"]
            if "CENTER" in g["align"]
            else self.colors["amber"]
            if "CORRECT" in g["align"]
            else self.colors["red"]
        )

        self.ground_align.config(
            text=f'ALIGNMENT: {g["align"]}',
            fg=align_color
        )

        glide_color = (
            self.colors["green"]
            if g["glide"] == "ON GLIDE"
            else self.colors["red"]
            if g["glide"] in ("TOO HIGH", "TOO LOW")
            else self.colors["amber"]
        )

        self.ground_glide.config(
            text=f'GLIDE: {g["glide"]}',
            fg=glide_color
        )

        # --------------------------------------------------------
        # Aircraft
        # --------------------------------------------------------

        if a["roll"] is None:
            self.roll_label.config(
                text="--°"
            )
            self.pitch_label.config(
                text="--°"
            )

        else:

            self.roll_label.config(
                text=f'{a["roll"]:+.2f}°'
            )

            self.pitch_label.config(
                text=f'{a["pitch"]:+.2f}°'
            )

        if analysis["aircraft_ok"]:

            self.bt_status.config(
                text="● LIVE",
                fg=self.colors["green"]
            )

        # --------------------------------------------------------
        # Analysis metrics
        # --------------------------------------------------------

        self.metrics[
            "LATERAL CONFIDENCE"
        ].config(
            text=f'{analysis["lateral_conf"]}%'
        )

        self.metrics[
            "VERTICAL STATUS"
        ].config(
            text=g["glide"],
            fg=glide_color
        )

        self.metrics[
            "RSSI STABILITY"
        ].config(
            text=analysis["stability"]
        )

        if analysis["trend"] >= 0.5:
            trend_text = f'INCREASING  +{analysis["trend"]:.2f} dB/sample'
            trend_color = self.colors["green"]

        elif analysis["trend"] <= -0.5:
            trend_text = f'DECREASING  {analysis["trend"]:.2f} dB/sample'
            trend_color = self.colors["red"]

        else:
            trend_text = "NO CLEAR TREND"
            trend_color = self.colors["amber"]

        self.metrics[
            "RSSI TREND"
        ].config(
            text=trend_text,
            fg=trend_color
        )

        self.metrics[
            "AIRCRAFT RESPONSE"
        ].config(
            text=analysis["response"],
            fg=(
                self.colors["green"]
                if analysis["aircraft_response"]
                else self.colors["amber"]
            )
        )

        if analysis["response_time"] is not None:

            self.metrics[
                "RESPONSE DELAY"
            ].config(
                text=f'{analysis["response_time"]:.2f} s'
            )

        else:

            self.metrics[
                "RESPONSE DELAY"
            ].config(
                text="--"
            )

        self.metrics[
            "RSSI RESPONSE"
        ].config(
            text=(
                "CONFIRMED"
                if analysis["rssi_response"]
                else "NOT YET"
            ),
            fg=(
                self.colors["green"]
                if analysis["rssi_response"]
                else self.colors["muted"]
            )
        )

        assessment_color = (
            self.colors["green"]
            if analysis["overall"] >= 80
            else self.colors["amber"]
            if analysis["overall"] >= 35
            else self.colors["red"]
        )

        self.metrics[
            "OVERALL ASSESSMENT"
        ].config(
            text=f'{analysis["overall"]}% — {analysis["assessment"]}',
            fg=assessment_color
        )

        # --------------------------------------------------------
        # Warnings
        # --------------------------------------------------------

        if analysis["ambiguous"]:

            self.log(
                "[WARNING] CENTERLINE is potentially ambiguous: "
                "L≈R while center is weaker."
            )

        self.root.after(
            100,
            self.update_ui
        )

    # ============================================================
    # LOG
    # ============================================================

    def log(self, message):

        timestamp = time.strftime(
            "%H:%M:%S"
        )

        print(
            f"{timestamp} {message}"
        )

        # Tk widgets should ideally be updated from the main thread.
        # For the prototype, the visible log is intentionally minimal.
        # Terminal logging remains authoritative.

    # ============================================================
    # CLOSE
    # ============================================================

    def close(self):

        self.running = False

        try:

            if self.bt_serial:
                self.bt_serial.close()

        except Exception:
            pass

        self.tft = None
        self.root.destroy()


# ================================================================
# MAIN
# ================================================================

if __name__ == "__main__":

    root = tk.Tk()

    app = Phase2ATC(root)

    root.mainloop()
