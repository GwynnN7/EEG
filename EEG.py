import sys, threading, socket, struct, serial
from datetime import datetime
from scipy.signal import butter, filtfilt, decimate
import numpy as np
import pyqtgraph as pg
import pywt

from PyQt5.QtCore import Qt, pyqtSignal, QTimer
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QMenuBar, QMenu, QAction, QWidget,
    QVBoxLayout, QLabel, QSlider, QDialog, QHBoxLayout, QSpinBox, QSizePolicy

)

class EEGReader():
    def __init__(self):
        self.buffer = []
        self.sps = 2000

        self.HOST = '0.0.0.0'
        self.PORT = 1234
        #self.COM = '/dev/ttyUSB0'
        #self.BAUD = 250000

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((self.HOST, self.PORT))
        self.sock.listen(1)

    def start(self):
        self.conn, self.addr = self.sock.accept()
        #self.serial = serial.Serial(self.COM, self.BAUD, timeout=1)

        t = threading.Thread(target=self.read_data)
        t.daemon = True
        t.start()

    def read_data(self):
        now = datetime.now()
        while True:
            data = self.conn.recv(4)
            #data = self.serial.read(4)
            if not data:
                continue

            value = struct.unpack('f', data)[0]
            self.buffer.append(value)

            if len(self.buffer) % 4096 == 0:
                end = datetime.now()
                sec = (end - now).total_seconds()
                self.sps = int(4096 / sec)
                now = end

class SliderDialog(QDialog):
    slider_change = pyqtSignal(int)

    def __init__(self, parent = None):
        super().__init__(parent)
        self.setWindowTitle("Value Selector")
        self.resize(200, 50)
        self.layout = QVBoxLayout()

    def setValues(self, value, max, min):
        self.label = QLabel(f"Current value: {value}")
        self.slider = QSpinBox()
        self.slider.setMinimum(min)
        self.slider.setMaximum(max)
        self.slider.setValue(value)
        self.slider.valueChanged.connect(self.update_value)

        self.layout.addWidget(self.label)
        self.layout.addWidget(self.slider)
        self.setLayout(self.layout)

    def update_value(self, value):
        self.label.setText(f"Slider value: {value}")
        self.slider_change.emit(value)


class EEGViewer(QMainWindow):
    def __init__(self, reader):
        self.reader = reader
        self.values = {"buffer-size": 4096, "buffer-offset": 2048, 'filter': 4, 'smooth': 64}
        self.maxs = {"buffer-size": 10240, "buffer-offset": 8192, 'filter': 12, 'smooth': 256}
        self.mins = {"buffer-size": 16, "buffer-offset": 0, 'filter': 1, 'smooth': 2}
        self.filters = {"low-pass": True, "high-pass": True, "smooth": False, "powerline": False, "split": False}
        self.activateFilters = True

        super().__init__()

        self.setWindowTitle("EEG")
        self.setFixedSize(1800, 900)

        central = QWidget()
        self.main_layout = QHBoxLayout(central)
        self.setCentralWidget(central)

        self.left_layout = QVBoxLayout()
        self.right_layout = QVBoxLayout()
        self.main_layout.addLayout(self.left_layout)
        self.main_layout.addLayout(self.right_layout)

        self.eeg = pg.PlotWidget(title="Raw EEG")
        self.eeg.showGrid(x=True, y=True)
        self.eegGraph = self.eeg.plot(pen='y')
        self.left_layout.addWidget(self.eeg)

        self.filteredEeg = pg.PlotWidget(title="Filtered EEG")
        self.filteredEeg.showGrid(x=True, y=True)
        self.filteredEegGraph = self.filteredEeg.plot(pen='y')
        self.left_layout.addWidget(self.filteredEeg)

        self.fft = pg.PlotWidget(title="FFT")
        self.fft.showGrid(x=True, y=True)
        self.FFTGraph = self.fft.plot(pen='c')
        self.left_layout.addWidget(self.fft)

        self.wavelets = []
        self.waveletsGraphs = []

        self.waveletsLevels = ["0Hz ~ 2Hz [Drifts/Delta]",
                      "2Hz ~ 4Hz [Delta]",
                      "4Hz ~ 8Hz [Theta]",
                      "8Hz ~ 15Hz [Alpha]",
                      "15Hz ~ 31Hz [Beta]",
                      "31Hz ~ 60Hz [Gamma]",
                      "60Hz ~ 125Hz [Muscle Noise]",
                      "125Hz ~ 250Hz [Noise]",
                      "250Hz ~ 500Hz [Noise]",
                      "500Hz ~ 1000Hz [Noise]",
                      ]

        for i in range(len(self.waveletsLevels)):
            self.wavelets.append(pg.PlotWidget(title=self.waveletsLevels[i]))
            curve = self.wavelets[i].plot(pen='r')
            self.waveletsGraphs.append(curve)
            self.right_layout.addWidget(self.wavelets[i])
            self.wavelets[i].setVisible(i <= 6)

        self.timer = QTimer()
        self.timer.timeout.connect(self.update_plot)
        self.timer.start(10)

        self.init_menu()

    def init_menu(self):
        self.menubar = self.menuBar()
        self.init_settings()
        self.init_eeg_settings()
        self.init_wavelets_settings()

    def init_settings(self):
        menu = self.menubar.addMenu("Settings")

        buffersize = QAction("Buffer Size", self)
        buffersize.triggered.connect(lambda: self.open_slider_dialog('buffer-size'))
        menu.addAction(buffersize)

        bufferoffset = QAction("Buffer Offset", self)
        bufferoffset.triggered.connect(lambda: self.open_slider_dialog('buffer-offset'))
        menu.addAction(bufferoffset)

        filters = QAction("Activate Filters", self)
        filters.setCheckable(True)
        filters.setChecked(self.activateFilters)
        filters.toggled.connect(self.toggleActivateFilters)
        menu.addAction(filters)

    def toggleActivateFilters(self, value):
        self.activateFilters = value

    def init_eeg_settings(self):
        eeg_menu = self.menubar.addMenu("EEG")

        filter_order = QAction("Filter Strength", self)
        filter_order.triggered.connect(lambda: self.open_slider_dialog('filter'))
        eeg_menu.addAction(filter_order)

        smooth_order = QAction("Smooth Strength", self)
        smooth_order.triggered.connect(lambda: self.open_slider_dialog('smooth'))
        eeg_menu.addAction(smooth_order)

        lowpass = QAction("Low Pass 30Hz", self)
        lowpass.setCheckable(True)
        lowpass.setChecked(self.filters['low-pass'])
        lowpass.toggled.connect(lambda state: self.toggle_eeg_checkbox('low-pass', state))
        eeg_menu.addAction(lowpass)

        highpass = QAction("High Pass 0.5Hz", self)
        highpass.setCheckable(True)
        highpass.setChecked(self.filters['high-pass'])
        highpass.toggled.connect(lambda state: self.toggle_eeg_checkbox('high-pass', state))
        eeg_menu.addAction(highpass)

        split = QAction("Split Filters", self)
        split.setCheckable(True)
        split.setChecked(self.filters['split'])
        split.toggled.connect(lambda state: self.toggle_eeg_checkbox('split', state))
        eeg_menu.addAction(split)

        smooth = QAction("Smooth", self)
        smooth.setCheckable(True)
        smooth.setChecked(self.filters['smooth'])
        smooth.toggled.connect(lambda state: self.toggle_eeg_checkbox('smooth', state))
        eeg_menu.addAction(smooth)

        powerline = QAction("Remove 50Hz", self)
        powerline.setCheckable(True)
        powerline.setChecked(self.filters['powerline'])
        powerline.toggled.connect(lambda state: self.toggle_eeg_checkbox('powerline', state))
        eeg_menu.addAction(powerline)

    def init_wavelets_settings(self):
        wavelet_menu = self.menubar.addMenu("Wavelets")
        for i in range(len(self.waveletsLevels)):
            wavelet = QAction(self.waveletsLevels[i], self)
            wavelet.setCheckable(True)
            wavelet.setChecked(i <= 6)
            wavelet.toggled.connect(lambda state, index=i: self.toggle_wavelet_checkbox(index, state))
            wavelet_menu.addAction(wavelet)

    def toggle_wavelet_checkbox(self, level, checked):
        self.wavelets[level].setVisible(checked)
        self.resize_wavelets()

    def resize_wavelets(self):
        visible_wv = [wv for wv in self.wavelets if wv.isVisible()]

        if not visible_wv:
            return

        margins = self.layout().contentsMargins()
        spacing = self.layout().spacing()
        usable_height = self.height() - margins.top() - margins.bottom() - (spacing * (len(visible_wv) -1))
        height = usable_height // len(visible_wv)

        for wv in visible_wv:
            wv.setFixedHeight(height)

    def toggle_eeg_checkbox(self, type, checked):
        self.filters[type] = checked

    def open_slider_dialog(self, type):
        dialog = SliderDialog(self)
        dialog.setValues(self.values[type], self.maxs[type], self.mins[type])
        dialog.slider_change.connect(lambda value: self.apply_slider_value(type, value))
        dialog.exec_()

    def apply_slider_value(self, type, value):
        self.values[type] = value


    def bandpass(self, data):
        b, a = butter(self.values['filter'], [2.5, 25], btype='bandpass', analog=False, fs=self.reader.sps)
        return filtfilt(b, a, data)

    def bandstop(self, data):
        b, a = butter(self.values['filter'], [40, 60], btype='bandstop', analog=False, fs=self.reader.sps)
        return filtfilt(b, a, data)

    def lowpass(self, data):
        b, a = butter(self.values['filter'], 25, btype='lowpass', analog=False, fs=self.reader.sps)
        return filtfilt(b, a, data)

    def highpass(self, data):
        b, a = butter(self.values['filter'], 2.5, btype='highpass', analog=False, fs=self.reader.sps)
        return filtfilt(b, a, data)

    def smooth(self, data):
        data = np.array(data)
        kernel = np.ones(self.values['smooth']) / self.values['smooth']
        return np.convolve(data, kernel, mode='same')

    def update_plot(self):
        buffer = self.reader.buffer[-(self.values['buffer-size'] * 10):]

        if(len(buffer) < self.values['buffer-size'] // 4):
            return

        raw_data = buffer[-(self.values['buffer-size'] + self.values['buffer-offset']) : -self.values['buffer-offset']]
        self.eegGraph.setData(y=raw_data)

        filtered_data = buffer[:]
        if(self.filters['high-pass'] and self.filters['low-pass'] and not self.filters['split']):
            filtered_data = self.bandpass(filtered_data)
        if(self.filters['high-pass'] and self.filters['split']):
            filtered_data = self.highpass(filtered_data)
        if(self.filters['low-pass'] and self.filters['split']):
            filtered_data = self.lowpass(filtered_data)

        if(self.filters['powerline']):
            filtered_data = self.bandstop(filtered_data)

        filtered_data = filtered_data[-(self.values['buffer-size'] + self.values['buffer-offset']) : -self.values['buffer-offset']]

        if(self.filters['smooth']):
            filtered_data = self.smooth(filtered_data)

        self.filteredEegGraph.setData(y=filtered_data)

        data = filtered_data if self.activateFilters else raw_data

        if len(data) >= self.values['buffer-size']:
            fft_vals = np.fft.rfft(np.array(data) * np.hamming(self.values['buffer-size']))
            fft_freq = np.fft.rfftfreq(self.values['buffer-size'], d=1/self.reader.sps)
            fft_freq = fft_freq[fft_freq <= 60]
            fft_magnitude = np.abs(fft_vals)[:len(fft_freq)]
            self.FFTGraph.setData(fft_freq, fft_magnitude)

            waveletsLevel = pywt.dwt_max_level(self.values['buffer-size'], pywt.Wavelet('db4').dec_len)
            coeffs = pywt.wavedec(data, 'db4', level=waveletsLevel)
            visible_wv = [wv.isVisible() for wv in self.wavelets]
            for i in range(len(coeffs)):
                if(visible_wv[i] == False):
                    coeffs[i] *= 0

            max_amp = max(np.max(np.abs(c)) for c in coeffs) * 1.5
            for i, coeff in enumerate(coeffs):
                if(visible_wv[i] == False):
                    continue
                #normalized_coeffs = [c / max_amp for c in coeff]
                self.wavelets[i].setYRange(-max_amp, max_amp)
                self.waveletsGraphs[i].setData(coeff)

if __name__ == '__main__':
    reader = EEGReader()
    reader.start()

    app = QApplication([])
    viewer = EEGViewer(reader)
    viewer.show()
    app.exec_()