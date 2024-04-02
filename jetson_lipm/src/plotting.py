from random import randint

import pyqtgraph as pg
from PyQt5 import QtCore, QtWidgets
import functools 

class MainWindow(QtWidgets.QMainWindow):
    def __init__(self,data_getter_list,data_count = 10):
        super().__init__()

        # Temperature vs time dynamic plot
        self.plot_graph = pg.PlotWidget()
        self.setCentralWidget(self.plot_graph)
        self.plot_graph.setBackground("w")
        pen = pg.mkPen(color=(255, 0, 0))
        self.plot_graph.setTitle("Temperature vs Time", color="b", size="20pt")
        styles = {"color": "red", "font-size": "18px"}
        self.plot_graph.setLabel("left", "Temperature (°C)", **styles)
        self.plot_graph.setLabel("bottom", "Time (min)", **styles)
        self.plot_graph.addLegend()
        self.plot_graph.showGrid(x=True, y=True)
        self.plot_graph.setYRange(20, 40)
        self.time = list(range(data_count))

        self.data_points = []


        # Get a line reference
        self.lines = []
        for i in range(len(data_getter_list)):
            self.data_points.append([randint(20, 40) for _ in range(data_count)])
            self.lines.append(self.plot_line(
                "Temperature Sensor {0}".format(i), self.time, self.data_points[i], pen, "b"
            ));
        # Add a timer to simulate new temperature measurements


        self.updater = functools.partial(self.update_plot,data_getter_list)

        
        # pen = 
        # pen = pg.mkPen(color=(0, 0, 255))
        # self.plot_line(
        #     "Temperature Sensor 2", self.time, self.temperature, pen, "r"
        # )

    def plot_line(self, name, time, temperature, pen, brush):
        return self.plot_graph.plot(
            time,
            temperature,
            name=name,
            pen=pen,
            symbol="+",
            symbolSize=15,
            symbolBrush=brush,
        )
    def update_plot(self,data_getter_list):
        for i in range(len(data_getter_list)):
            self.data_points[i] = self.data_points[i][1:]
            self.data_points[i].append(data_getter_list[i]())
            self.lines[i].setData(self.time, self.data_points[i])



