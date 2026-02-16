"""
Qt visualization for Inspire Hand (adapted from inspire_hand_sdk).
"""

import pyqtgraph as pg
from PyQt5 import QtCore
from PyQt5.QtWidgets import QApplication, QMainWindow, QTabWidget, QWidget, QGridLayout, QLabel, QVBoxLayout
import colorcet
import numpy as np
import time

from viz_data import data_sheet, status_codes, update_error_label


class ImageTab(QWidget):
    """Tab showing tactile sensor heatmaps."""

    def __init__(self, datas=data_sheet):
        super().__init__()
        self.layout = QVBoxLayout()
        self.setLayout(self.layout)

        self.grid_layout = QGridLayout()
        self.layout.addLayout(self.grid_layout)
        self.data_sheet = datas

        self.create_images()

    def create_images(self):
        num_cols = 4
        self.plots = []
        self.color_maps = []
        self.color_bars = []

        for i, (name, addr, length, size, var) in enumerate(self.data_sheet):
            row = i // num_cols
            col = i % num_cols

            layout_widget = pg.GraphicsLayoutWidget(show=True)
            plot_item = layout_widget.addPlot(row=0, col=0)
            plot_item.setTitle(name)
            img_item = pg.ImageItem(np.random.rand(size[0], size[1]))
            plot_item.addItem(img_item)
            self.plots.append(img_item)

            color_map = pg.ColorMap(pos=np.linspace(0, 1, 256), color=colorcet.fire[:256])
            self.color_maps.append(color_map)

            color_bar = pg.ColorBarItem(colorMap=color_map, values=(0, 1), width=5, orientation='h')
            self.color_bars.append(color_bar)
            layout_widget.addItem(color_bar, row=1, col=0)

            self.grid_layout.addWidget(layout_widget, row, col)

    def update_plot(self, data_dict):
        for i, (name, addr, length, size, var) in enumerate(self.data_sheet):
            if var in data_dict:
                self.plots[i].setImage(data_dict[var], autoLevels=True)
                max_val = np.max(data_dict[var])
                if max_val > 0:
                    self.plots[i].setLevels((0, max_val))
                    self.color_bars[i].setLevels((0, max_val))
                self.plots[i].setColorMap(self.color_maps[i])


class CurveTab(QWidget):
    """Tab showing state curves over time."""

    def __init__(self, datas=data_sheet, history_len=100):
        super().__init__()
        self.layout = QVBoxLayout()
        self.setLayout(self.layout)

        self.grid_layout = QGridLayout()
        self.layout.addLayout(self.grid_layout)
        self.data_sheet = datas
        self.history_length = history_len

        self.history = {
            'POS_ACT': [np.zeros(history_len) for _ in range(6)],
            'ANGLE_ACT': [np.zeros(history_len) for _ in range(6)],
            'FORCE_ACT': [np.zeros(history_len) for _ in range(6)],
            'CURRENT': [np.zeros(history_len) for _ in range(6)],
            'ERROR': [np.zeros(history_len) for _ in range(6)],
            'STATUS': [np.zeros(history_len) for _ in range(6)],
            'TEMP': [np.zeros(history_len) for _ in range(6)]
        }
        self.create_curves()

    def create_curves(self):
        self.error_label = QLabel("ERROR: ")
        self.status_label = QLabel("STATUS: ")

        self.layout.addWidget(self.error_label)
        self.layout.addWidget(self.status_label)

        self.plot_items = {name: pg.PlotWidget() for name in ['POS_ACT', 'ANGLE_ACT', 'FORCE_ACT', 'CURRENT']}
        for i, (name, plot_widget) in enumerate(self.plot_items.items()):
            self.grid_layout.addWidget(plot_widget, i // 2, i % 2)
            plot_widget.setTitle(name)
            plot_widget.setLabel('left', 'Value')
            plot_widget.setLabel('bottom', 'Time')
            plot_widget.setBackground((0, 0, 0))
            plot_widget.addLegend()
            plot_widget.showButtons()
            plot_widget.enableAutoRange()
            plot_widget.showGrid(x=True, y=True, alpha=0.2)

        finger_names = ['Little', 'Ring', 'Middle', 'Index', 'Thumb', 'ThumbRot']
        self.curves = {}
        for name in self.plot_items:
            self.curves[name] = [
                self.plot_items[name].plot(pen=pg.mkPen(colorcet.glasbey[i]), name=finger_names[i])
                for i in range(6)
            ]

    def update_plot(self, data_dict):
        try:
            for category in ['POS_ACT', 'ANGLE_ACT', 'FORCE_ACT', 'CURRENT']:
                if category in data_dict and data_dict[category] is not None:
                    datas = data_dict[category]
                    for i in range(min(len(datas), 6)):
                        self.history[category][i] = np.roll(self.history[category][i], -1)
                        self.history[category][i][-1] = datas[i]
                        self.curves[category][i].setData(self.history[category][i])

            if 'ERROR' in data_dict:
                err = update_error_label(data_dict['ERROR'])
                self.error_label.setText(f"ERROR: {err}")

            if 'STATUS' in data_dict:
                status_str = ', '.join([status_codes.get(int(s), f"?{s}") for s in data_dict['STATUS']])
                self.status_label.setText(f"STATUS: {status_str}")

        except Exception as e:
            print(f"Error updating plot: {e}")


class MainWindow(QMainWindow):
    """Main visualization window with tabs."""

    def __init__(self, data_handler, data=data_sheet, dt=100, name="Inspire Hand Visualizer", plot_touch=True):
        super().__init__()
        self.setWindowTitle(name)
        self.setGeometry(100, 100, 1200, 800)
        self.dt = dt
        self.data_handler = data_handler
        self.plot_touch = plot_touch

        self.tabs = QTabWidget()
        self.curve_tab = CurveTab(data)
        self.tabs.addTab(self.curve_tab, "State Curves")

        if plot_touch:
            self.image_tab = ImageTab(data)
            self.tabs.addTab(self.image_tab, "Tactile Images")

        self.setCentralWidget(self.tabs)

    def update_plot(self):
        data_dict = self.data_handler.read()
        self.curve_tab.update_plot(data_dict['states'])
        if self.plot_touch and 'touch' in data_dict:
            self.image_tab.update_plot(data_dict['touch'])

    def reflash(self):
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_plot)
        self.timer.start(self.dt)
