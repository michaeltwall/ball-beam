import sys
import time
import serial
import threading
from collections import deque
from PyQt6.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QWidget
import pyqtgraph as pg
from PyQt6.QtCore import QTimer, Qt

# --- CONFIGURATION ---
SERIAL_PORT = 'COM5'     # Change to your port (e.g., '/dev/ttyUSB0')
BAUD_RATE = 9600
MAX_POINTS = 200
HISTORY_SECONDS = 10    # how much history to show

class PlotterWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Independent Live Plotter")
        self.resize(800, 500)

        # Initialize Serial Port
        try:
            self.ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
        except serial.SerialException as e:
            print(f"[Plotter Error] Could not open port {SERIAL_PORT}: {e}")
            sys.exit(1)

        # Thread-Safe Data Buffers — each line has its own (t, y) deques
        self.lock = threading.Lock()
        self.t_serial   = deque()
        self.y_serial   = deque()
        self.t_internal = deque()
        self.y_internal = deque()

        # Common time reference so both lines share the same t=0
        self.t0 = time.monotonic()

        # Latest serial value for main.py to read on demand
        self.latest_serial = None

        # Key event state for main.py hotkey consumption
        self.key_events = {}

        # Create Layout and Plot
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        layout = QVBoxLayout(central_widget)

        self.plot_widget = pg.PlotWidget()
        self.plot_widget.addLegend()
        self.plot_widget.setLabel('bottom', 'Time (s)')
        self.plot_widget.setLabel('left', 'Value')
        self.plot_widget.showGrid(x=True, y=True)
        layout.addWidget(self.plot_widget)

        # Plot Lines
        self.curve_serial   = self.plot_widget.plot(pen=pg.mkPen('r', width=2), name="Incoming Serial")
        self.curve_internal = self.plot_widget.plot(pen=pg.mkPen('b', width=2), name="Main Script Data")

        # UI Refresh Timer
        self.timer = QTimer()
        self.timer.setInterval(30)
        self.timer.timeout.connect(self.refresh_gui_lines)
        self.timer.start()

        # Start Background Serial Reader Thread
        self.running = True
        self.serial_thread = threading.Thread(target=self._background_serial_reader, daemon=True)
        self.serial_thread.start()

    def _background_serial_reader(self):
        """Listens to serial, feeds the plot deque and overwrites latest_serial for main.py."""
        while self.running:
            if self.ser.in_waiting > 0:
                try:
                    line_raw = self.ser.readline().decode('utf-8').strip()
                    if line_raw:
                        val = float(line_raw)
                        t   = time.monotonic() - self.t0
                        with self.lock:
                            self.t_serial.append(t)
                            self.y_serial.append(val)
                            self.latest_serial = val   # always the freshest reading
                except (ValueError, IndexError, serial.SerialException):
                    continue

    def add_main_data(self, value):
        """Pushes data from main.py calculations into the internal plot line."""
        t = time.monotonic() - self.t0
        with self.lock:
            self.t_internal.append(t)
            self.y_internal.append(float(value))

    def send_serial(self, text):
        """Sends a value out over the serial port."""
        try:
            self.ser.write((str(text) + '\n').encode('utf-8'))
        except serial.SerialException as e:
            print(f"[Plotter Error] Failed to send data: {e}")

    def read_latest_serial(self):
        """Returns the most recent serial value, or None if nothing received yet."""
        with self.lock:
            return self.latest_serial

    def refresh_gui_lines(self):
        now = time.monotonic() - self.t0
        cutoff = now - HISTORY_SECONDS

        with self.lock:
            # Trim old points from the left
            while self.t_serial and self.t_serial[0] < cutoff:
                self.t_serial.popleft()
                self.y_serial.popleft()
            while self.t_internal and self.t_internal[0] < cutoff:
                self.t_internal.popleft()
                self.y_internal.popleft()

            t_ser = list(self.t_serial)
            y_ser = list(self.y_serial)
            t_int = list(self.t_internal)
            y_int = list(self.y_internal)

        if t_ser:
            self.curve_serial.setData(t_ser, y_ser)
        if t_int:
            self.curve_internal.setData(t_int, y_int)

    def keyPressEvent(self, event):
        key_map = {
            Qt.Key.Key_Escape: 'esc',
            Qt.Key.Key_C:      'c',
            Qt.Key.Key_R:      'r',
        }
        mapped = key_map.get(event.key())
        if mapped:
            self.key_events[mapped] = True
        super().keyPressEvent(event)

    def consume_key(self, key):
        """Returns True once if that key was pressed since last check, then clears it."""
        return self.key_events.pop(key, False)

    def closeEvent(self, event):
        self.running = False
        self.timer.stop()
        if self.ser.isOpen():
            self.ser.close()
        event.accept()


# Global state
_app    = None
_window = None

def initialize_plotter():
    """Initializes the plotter window and returns interface functions."""
    global _app, _window
    _app = QApplication.instance()
    if not _app:
        _app = QApplication(sys.argv)
    _window = PlotterWindow()
    _window.show()

    # send_func, plot_func, read_func, ui_update_func, consume_key_func
    return (
        _window.send_serial,
        _window.add_main_data,
        _window.read_latest_serial,
        _process_events,
        _window.consume_key,
    )

def _process_events():
    """Pumps the Qt event loop. Call once per main loop iteration."""
    if _app:
        _app.processEvents()