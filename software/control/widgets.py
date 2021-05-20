# set QT_API environment variable
import os 
os.environ["QT_API"] = "pyqt5"
import qtpy

# qt libraries
from qtpy.QtCore import *
from qtpy.QtWidgets import *
from qtpy.QtGui import *

from control._def import *

import numpy as np
import pyqtgraph as pg
from collections import deque
import time

class ControlPanel(QFrame):

	signal_logging_onoff = Signal(bool,str)

	def __init__(self, main=None, *args, **kwargs):
		super().__init__(*args, **kwargs)
		self.font = QFont()
		self.font.setPixelSize(16)
		self.add_components()
		self.setFrameStyle(QFrame.Panel | QFrame.Raised)

	def add_components(self):

		self.lineEdit_experimentID = QLineEdit()

		self.btn_logging_onoff = QPushButton('Logging On/Off')
		self.btn_logging_onoff.setDefault(False)
		self.btn_logging_onoff.setCheckable(True)
		self.btn_logging_onoff.setChecked(True)

		# self.label_print = QLabel()
		# self.label_print.setFrameStyle(QFrame.Panel | QFrame.Sunken)

		grid_line2 = QHBoxLayout()
		grid_line2.addWidget(QLabel('File Prefix'))
		grid_line2.addWidget(self.lineEdit_experimentID)
		grid_line2.addWidget(self.btn_logging_onoff)

		# grid_line11 = QGridLayout()
		# grid_line11.addWidget(self.label_print,0,0,10,0)

		# for displaying stepper position and flow/pressure measurements
		self.label_ch1 = QLabel()
		self.label_ch1.setFrameStyle(QFrame.Panel | QFrame.Sunken)
		self.label_ch1.setFixedWidth(50)
		self.label_ch2 = QLabel()
		self.label_ch2.setFrameStyle(QFrame.Panel | QFrame.Sunken)
		self.label_ch2.setFixedWidth(50)
		self.label_ch3 = QLabel()
		self.label_ch3.setFrameStyle(QFrame.Panel | QFrame.Sunken)
		self.label_ch3.setFixedWidth(50)

		# self.label_print = QLabel()
		# self.label_print.setFrameStyle(QFrame.Panel | QFrame.Sunken)

		grid_line3 = QHBoxLayout()
		grid_line3.addWidget(QLabel('ch1'))
		grid_line3.addWidget(self.label_ch1)
		grid_line3.addWidget(QLabel('ch2'))
		grid_line3.addWidget(self.label_ch2)
		# grid_line3.addWidget(QLabel('temperature (degree C)'))
		# grid_line3.addWidget(self.label_ch3)

		self.entry_threshold_start_flow = QDoubleSpinBox()
		self.entry_threshold_start_flow.setMinimum(-2) 
		self.entry_threshold_start_flow.setMaximum(2) 
		self.entry_threshold_start_flow.setSingleStep(0.01)
		self.entry_threshold_start_flow.setKeyboardTracking(False)
		self.entry_threshold_start_flow.setValue(-0.07)

		self.entry_threshold_stop_flow = QDoubleSpinBox()
		self.entry_threshold_stop_flow.setMinimum(-2) 
		self.entry_threshold_stop_flow.setMaximum(2) 
		self.entry_threshold_stop_flow.setSingleStep(0.01)
		self.entry_threshold_stop_flow.setKeyboardTracking(False)
		self.entry_threshold_stop_flow.setValue(-0.12)

		grid_line4 = QHBoxLayout()
		grid_line4.addWidget(QLabel('threshold_start_flow'))
		grid_line4.addWidget(self.entry_threshold_start_flow)
		grid_line4.addWidget(QLabel('threshold_stop_flow'))
		grid_line4.addWidget(self.entry_threshold_stop_flow)

		self.grid = QGridLayout()
		self.grid.addLayout(grid_line2,2,0)
		self.grid.addLayout(grid_line3,3,0)
		self.grid.addLayout(grid_line4,4,0)
		# self.grid.addWidget(self.label_print,3,0,1,8)

		self.setLayout(self.grid)
		self.btn_logging_onoff.clicked.connect(self.logging_onoff)
		self.entry_threshold_stop_flow.valueChanged.connect(self.set_threshold_start)
		self.entry_threshold_stop_flow.valueChanged.connect(self.set_threshold_stop)

	def logging_onoff(self,state):
		self.signal_logging_onoff.emit(state,self.lineEdit_experimentID.text())

	def set_threshold_start(self,value):
		self.microcontroller.set_threshold_start(value)

	def set_threshold_stop(self,value):
		self.microcontroller.set_threshold_stop(value)



class WaveformDisplay(QFrame):

	def __init__(self, main=None, *args, **kwargs):
		super().__init__(*args, **kwargs)
		self.add_components()
		self.setFrameStyle(QFrame.Panel | QFrame.Raised)

	def add_components(self):
		# self.plotWidgets = {key: PlotWidget(title = key, color = 'b') for key in PLOTS}
		# self.plotWidgets['Airway Pressure'].plot1.setYRange(min=WAVEFORMS.PAW_MIN,max=WAVEFORMS.PAW_MAX)
		# self.plotWidgets['Flow Rate'].plot1.setYRange(min=WAVEFORMS.FLOW_MIN,max=WAVEFORMS.FLOW_MAX)
		# self.plotWidgets['Volume'].plot1.setYRange(min=WAVEFORMS.V_MIN,max=WAVEFORMS.V_MAX)

		# grid = QGridLayout() 
		# for ii, key in enumerate(PLOTS):
		# 	grid.addWidget(self.plotWidgets[key], ii, 0,1,2)
		# self.setLayout(grid)

		self.plotWidget = []
		n = 2
		for i in range(n):
			self.plotWidget.append(PlotWidget())
		layout = QGridLayout() #layout = QStackedLayout()
		for i in range(n):
			layout.addWidget(self.plotWidget[i],i,0)
		self.setLayout(layout)


class PlotWidget(pg.GraphicsLayoutWidget):
	def __init__(self, window_title='',parent=None):
		super().__init__(parent)
		self.plotWidget = self.addPlot(title = '')

	def plot(self,x,y):
		# self.plotWidget.plot(x,y,pen=(0,3),clear=True)
		self.plotWidget.plot(x,y,pen=(0,3),clear=True)

'''
class PlotWidget(QFrame):
	def __init__(self, window_title='',parent=None):
		super().__init__(parent)
		self.plotWidget = pg.PlotWidget()
		
		layout = QGridLayout()
		layout.addWidget(self.plotWidget,0,0)

		self.setLayout(layout)

		# self.plotWidget.show()
		self.p1 = self.plotWidget.plotItem
		self.p1.setLabels(left='Flow (L/mim)')
		self.p2 = pg.ViewBox()
		self.p1.showAxis('right')
		self.p1.scene().addItem(self.p2)
		self.p1.getAxis('right').linkToView(self.p2)
		self.p2.setXLink(self.p1)
		self.p1.getAxis('right').setLabel('axis2', color='#0000ff')
		self.p2.setGeometry(self.p1.vb.sceneBoundingRect())

	def plot(self,x,y):
		self.p1.plot(x,y,pen=(0,3),clear=True)

	def plot_yy(self,x,y1,y2):
		self.p1.plot(x,y1,pen=(0,3),clear=True)
		self.p2.addItem(pg.PlotCurveItem(y2, pen='b'))
'''
