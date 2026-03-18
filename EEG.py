import sys
import serial
import struct
import threading
import numpy as np
import pyqtgraph as pg
from scipy.signal import butter, sosfilt, iirnotch, hilbert, lfilter, savgol_filter
from PyQt5.QtCore import QTimer, Qt
from PyQt5.QtWidgets import (QApplication, QMainWindow, QVBoxLayout, QHBoxLayout, 
                             QWidget, QCheckBox, QGroupBox, QDoubleSpinBox, 
                             QSpinBox, QPushButton, QLabel, QLineEdit, QMessageBox, QProgressBar)

DEFAULT_BAUD = 250000
SAMPLE_RATE = 300
MAX_BUFFER_SEC = 20
FILTER_PADDING = 100 
COLORS = ['y', 'c', 'm', 'g', 'r', 'b', 'w']

class EEGReceiver:
    def __init__(self):
        self.running = False
        self.ser = None
        self.num_channels = 2
        self.buffers = [] 
        self.lock = threading.Lock()

    def connect(self, port, baud, channels):
        self.running = False
        if self.ser and self.ser.is_open: self.ser.close()
        try:
            self.ser = serial.Serial(port, baud, timeout=1)
            self.num_channels = channels
            self.buffers = [[] for _ in range(channels)]
            self.running = True
            t = threading.Thread(target=self._read_loop, daemon=True)
            t.start()
            return True
        except Exception as e:
            print(f"Connection Error: {e}")
            return False

    def disconnect(self):
        self.running = False
        if self.ser: self.ser.close()

    def _read_loop(self):
        packet_size = 4 * self.num_channels
        unpack_fmt = f'{self.num_channels}f'
        
        while self.running:
            try:
                if self.ser.read(1) != b'\xAA': continue
                data = self.ser.read(packet_size)
                if len(data) < packet_size: continue
                values = struct.unpack(unpack_fmt, data)
                
                with self.lock:
                    for i in range(self.num_channels):
                        self.buffers[i].append(values[i])
                        if len(self.buffers[i]) > SAMPLE_RATE * MAX_BUFFER_SEC:
                            self.buffers[i] = self.buffers[i][-int(SAMPLE_RATE * MAX_BUFFER_SEC):]
            except:
                break

class EEGDashboard(QMainWindow):
    def __init__(self, receiver):
        super().__init__()
        self.receiver = receiver
        self.paused = False
        
        self.is_calibrating = False
        self.calibration_data = []
        self.baseline_std = [1.0] * 8
        self.artifact_thresh_sigma = 4.0
        self.is_calibrated = False

        self.setWindowTitle("EEG Dashboard")
        self.resize(1400, 900)
        
        central = QWidget()
        self.setCentralWidget(central)
        self.main_layout = QHBoxLayout(central)

        self.init_sidebar()
        self.plot_layout = QVBoxLayout()
        self.init_plots()
        
        self.main_layout.addLayout(self.sidebar_layout, 1)
        self.main_layout.addLayout(self.plot_layout, 4)

        self.update_filters()
        
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_loop)
        self.timer.start(40)

    def init_sidebar(self):
        self.sidebar_layout = QVBoxLayout()
        
        self.grp_conn = QGroupBox("1. Connection")
        vbox_conn = QVBoxLayout()
        self.txt_port = QLineEdit("/dev/ttyUSB0") 
        self.txt_port.setPlaceholderText("Port")
        self.spin_channels = QSpinBox(); self.spin_channels.setValue(2)
        self.btn_connect = QPushButton("Connect")
        self.btn_connect.setStyleSheet("background-color: #ccffcc; font-weight: bold;")
        self.btn_connect.clicked.connect(self.handle_connect)
        vbox_conn.addWidget(self.txt_port)
        vbox_conn.addWidget(QLabel("Channels:"))
        vbox_conn.addWidget(self.spin_channels)
        vbox_conn.addWidget(self.btn_connect)
        self.grp_conn.setLayout(vbox_conn)
        self.sidebar_layout.addWidget(self.grp_conn)

        self.grp_calib = QGroupBox("2. Noise Calibration")
        vbox_calib = QVBoxLayout()
        self.btn_calibrate = QPushButton("Calibration (5s)")
        self.btn_calibrate.clicked.connect(self.start_calibration)
        self.btn_calibrate.setStyleSheet("background-color: #ffeb3b; font-weight: bold;")
        self.lbl_calib_status = QLabel("Status: Uncalibrated")
        self.lbl_calib_status.setStyleSheet("color: red")
        self.progress_calib = QProgressBar()
        self.progress_calib.setRange(0, 100); self.progress_calib.setValue(0)
        vbox_calib.addWidget(self.btn_calibrate)
        vbox_calib.addWidget(self.progress_calib)
        vbox_calib.addWidget(self.lbl_calib_status)
        self.grp_calib.setLayout(vbox_calib)
        self.sidebar_layout.addWidget(self.grp_calib)

        # 3. Controls
        self.grp_control = QGroupBox("3. Visualization")
        vbox_c = QVBoxLayout()
        self.btn_pause = QPushButton("Pause")
        self.btn_pause.setCheckable(True)
        self.btn_pause.clicked.connect(self.toggle_pause)
        
        self.lbl_window = QLabel("Window:")
        self.spin_window = QDoubleSpinBox()
        self.spin_window.setRange(1.0, 15.0); self.spin_window.setValue(3.0)

        # Envelope
        self.lbl_savgol = QLabel("SavGol Window:")
        self.spin_savgol_win = QSpinBox()
        self.spin_savgol_win.setRange(3, 101); self.spin_savgol_win.setValue(51); self.spin_savgol_win.setSingleStep(2)
        
        self.lbl_savgol_poly = QLabel("SavGol Poly Order:")
        self.spin_savgol_poly = QSpinBox()
        self.spin_savgol_poly.setRange(1, 5); self.spin_savgol_poly.setValue(3)

        vbox_c.addWidget(self.btn_pause)
        vbox_c.addWidget(self.lbl_window)
        vbox_c.addWidget(self.spin_window)
        vbox_c.addWidget(self.lbl_savgol)
        vbox_c.addWidget(self.spin_savgol_win)
        vbox_c.addWidget(self.lbl_savgol_poly)
        vbox_c.addWidget(self.spin_savgol_poly)
        self.grp_control.setLayout(vbox_c)
        self.sidebar_layout.addWidget(self.grp_control)
        
        # 4. Filters
        self.grp_filters = QGroupBox("4. Filters")
        vbox_f = QVBoxLayout()
        self.chk_notch = QCheckBox("50Hz Notch"); self.chk_notch.setChecked(True)
        self.chk_bandpass = QCheckBox("Bandpass"); self.chk_bandpass.setChecked(True)
        self.chk_detrend = QCheckBox("Detrend"); self.chk_detrend.setChecked(True)
        self.chk_clip = QCheckBox("Clip Artifacts"); self.chk_clip.setChecked(False) 
        
        self.chk_notch.toggled.connect(self.update_filters)
        self.chk_bandpass.toggled.connect(self.update_filters)
        
        self.spin_low = QDoubleSpinBox(); self.spin_low.setRange(0.1, 50.0); self.spin_low.setValue(8.0)
        self.spin_high = QDoubleSpinBox(); self.spin_high.setRange(1.0, 100.0); self.spin_high.setValue(12.0)
        self.spin_low.valueChanged.connect(self.update_filters)
        self.spin_high.valueChanged.connect(self.update_filters)

        vbox_f.addWidget(self.chk_notch)
        vbox_f.addWidget(self.chk_bandpass)
        vbox_f.addWidget(self.chk_detrend)
        vbox_f.addWidget(self.chk_clip)
        vbox_f.addWidget(QLabel("Freq Low/High:"))
        vbox_f.addWidget(self.spin_low)
        vbox_f.addWidget(self.spin_high)
        self.grp_filters.setLayout(vbox_f)
        self.sidebar_layout.addWidget(self.grp_filters)

        self.sidebar_layout.addStretch()

    def init_plots(self):
        self.plot_raw = pg.PlotWidget(title="Raw")
        self.plot_proc = pg.PlotWidget(title="Filtered")
        self.plot_env = pg.PlotWidget(title="Envelope")
        
        for p in [self.plot_raw, self.plot_proc, self.plot_env]:
            p.showGrid(x=True, y=True)
            self.plot_layout.addWidget(p)
            
        self.curves_raw, self.curves_proc, self.curves_env = [], [], []
        self.chk_channels = []

    def handle_connect(self):
        if self.receiver.connect(self.txt_port.text(), DEFAULT_BAUD, self.spin_channels.value()):
            self.btn_connect.setText("Connected"); self.btn_connect.setStyleSheet("background-color: #aaffaa;")
            self.rebuild_channels_ui(self.spin_channels.value())

    def rebuild_channels_ui(self, n):
        for p in [self.plot_raw, self.plot_proc, self.plot_env]: p.clear()
        self.curves_raw, self.curves_proc, self.curves_env = [], [], []
        
        for i in range(n):
            color = COLORS[i % len(COLORS)]
            pen = pg.mkPen(color, width=1)
            pen_thick = pg.mkPen(color, width=2)
            self.curves_raw.append(self.plot_raw.plot(pen=pen))
            self.curves_proc.append(self.plot_proc.plot(pen=pen))
            self.curves_env.append(self.plot_env.plot(pen=pen_thick))

    def start_calibration(self):
        self.is_calibrating = True
        self.calibration_data = [[] for _ in range(self.receiver.num_channels)]
        self.btn_calibrate.setText("Calibrating...")
        self.btn_calibrate.setStyleSheet("background-color: orange")
        QTimer.singleShot(500, self.update_calibration)
    
    def update_calibration(self):
        if not self.is_calibrating: return
        total_points = SAMPLE_RATE * 5
        current_points = len(self.calibration_data[0]) if self.calibration_data else 0
        progress = int((current_points / total_points) * 100) if total_points > 0 else 0
        self.progress_calib.setValue(min(progress, 100))
        if(progress >= 100):
            self.finish_calibration()
        else:
            QTimer.singleShot(500, self.update_calibration)
    
    def finish_calibration(self):
        self.is_calibrating = False
        
        for i in range(len(self.calibration_data)):
            if len(self.calibration_data[i]) > 100:
                data = np.array(self.calibration_data[i])
                if self.chk_detrend.isChecked(): data = data - np.mean(data)

                self.baseline_std[i] = np.std(data)
            else:
                self.baseline_std[i] = 1000.0
        
        self.is_calibrated = True
        self.lbl_calib_status.setText("Status: Calibrated")
        self.lbl_calib_status.setStyleSheet("color: green; font-weight: bold")
        self.btn_calibrate.setText("Recalibrate")
        self.btn_calibrate.setStyleSheet("background-color: #dddddd")

    def toggle_pause(self):
        self.paused = not self.paused
        self.btn_pause.setText("Resume" if self.paused else "Pause")

    def update_filters(self):
        low, high = self.spin_low.value(), self.spin_high.value()
        if low >= high: low = high - 0.5
        self.sos_band = butter(4, [low, high], btype='band', fs=SAMPLE_RATE, output='sos')
        self.notch_b, self.notch_a = iirnotch(50.0, 30.0, SAMPLE_RATE)

    def process_signal(self, data, ch_idx):
        if len(data) < 10: return data, np.zeros(len(data)), False

        # 1. Detrend
        if self.chk_detrend.isChecked(): 
            data = data - np.mean(data)

        # 2. Artifact Detection
        is_artifact = False
        if self.is_calibrated and len(data) > 0:
            limit = self.baseline_std[ch_idx] * self.artifact_thresh_sigma
            recent_segment = data[-20:] 
            
            if np.std(recent_segment) > limit:
                is_artifact = True
                
            if self.chk_clip.isChecked():
                data = np.clip(data, -limit, limit)

        # 3. Filtering
        if self.chk_notch.isChecked(): data = lfilter(self.notch_b, self.notch_a, data)
        if self.chk_bandpass.isChecked(): data = sosfilt(self.sos_band, data)
            
        # 4. Envelope
        analytic = hilbert(data)
        envelope = np.abs(analytic)
        
        win = self.spin_savgol_win.value()
        if win % 2 == 0: win += 1
        poly = self.spin_savgol_poly.value()
        if win > poly:
            try:
                envelope = savgol_filter(envelope, window_length=win, polyorder=poly)
            except: pass
            
        return data, envelope, is_artifact

    def update_loop(self):
        if self.paused or not self.receiver.running: return

        pts = int(self.spin_window.value() * SAMPLE_RATE)
        total_fetch = pts + FILTER_PADDING
        
        with self.receiver.lock:
            for i in range(min(len(self.curves_raw), len(self.receiver.buffers))):
                raw = list(self.receiver.buffers[i])
                
                if self.is_calibrating:
                    self.calibration_data[i].extend(raw[-10:])
                
                if len(raw) < total_fetch: continue
                
                process_arr = np.array(raw[-total_fetch:])
                display_slice = slice(FILTER_PADDING, None)

                self.curves_raw[i].setData(process_arr[display_slice])
                
                proc, env, is_bad = self.process_signal(process_arr, i)
                
                self.curves_proc[i].setData(proc[display_slice])
                
                if is_bad:
                    self.curves_proc[i].setPen(pg.mkPen('r', width=2))
                else:
                    self.curves_proc[i].setPen(pg.mkPen(COLORS[i % len(COLORS)], width=1))

                self.curves_env[i].setData(env[display_slice])

if __name__ == '__main__':
    receiver = EEGReceiver()
    app = QApplication(sys.argv)
    dashboard = EEGDashboard(receiver)
    dashboard.show()
    sys.exit(app.exec_())