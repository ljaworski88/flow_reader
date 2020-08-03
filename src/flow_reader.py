#!/usr/bin/env python3

from sf04 import *
from time import sleep, time
from collections import deque
from statistics import mean
from os import system, path
import sys
import yaml
from mainWindow import Ui_MainWindow
import numpy as np
from PyQt5 import QtCore, QtWidgets, QtGui
from PyQt5.QtWidgets import QApplication, QMainWindow, QMessageBox
from PyQt5.QtGui import QIntValidator, QDoubleValidator
import random

class StreamingPotentialApp(QMainWindow, Ui_MainWindow):
    def __init__(self, parent=None, lastSettings='settings.yaml'):
        super(StreamingPotentialApp, self).__init__(parent)
        self.setupUi(self)
        # Set the main page of the app
        self.mainTabStack.setCurrentIndex(0)
        self.graphsTab.setCurrentIndex(0)
        self.globalSettings = lastSettings

        # Initialize some dicts and lists that will be used in conversions later
        self.timeUnitsDict = {'sec' : 1,
                              'min' : 60,
                              'hr'  : 3600}
        self.volumeUnitsDict = {'L'  : 1,
                                'mL' : 1e-3,
                                'uL' : 1e-6,
                                'nL' : 1e-9,
                                'm^3': 1000}
        self.pressureUnitsDict = {'Pa'   : 1,
                                  'psi'  : 6894.76,
                                  'atm'  : 101325,
                                  'mmHg' : 133.322,
                                  'torr' : 133.322}
        self.flowRates = [''.join([volumeUnit, '/', timeUnit]) for volumeUnit in self.volumeUnitsDict.keys() for timeUnit in self.timeUnitsDict.keys()]

        # Set input validators
        self.flowSetpointLineEdit.setValidator(QDoubleValidator())
        self.errorBoundsLineEdit.setValidator(QDoubleValidator())
        self.crossSectionLineEdit.setValidator(QDoubleValidator())
        self.tissueThicknessLineEdit.setValidator(QDoubleValidator())
        self.rightTransducerSlopeLineEdit.setValidator(QDoubleValidator())
        self.rightTransducerInterceptLineEdit.setValidator(QDoubleValidator())
        self.leftTransducerSlopeLineEdit.setValidator(QDoubleValidator())
        self.leftTransducerInterceptLineEdit.setValidator(QDoubleValidator())
        self.lockTimeLineEdit.setValidator(QIntValidator())

        self.statusMessage = 'Welcome!'
        self.i2c_bus = SMBus(3)
        self.dataTimer = QtCore.QTimer(self)
        self.dataTimer.timeout.connect(self.UpdateData)
        self.timerInterval = 250
        self.lockTime = 1 * 60 # 5min * 60 sec
        self.timeData = deque([], int(self.lockTime * 1000 / self.timerInterval)) # store enough data for the lock in time
        self.unitsComboBox.addItems(self.flowRates)
        self.unitsComboBox.setCurrentText('nL/min')
        self.flowSetpointLineEdit.setText('3300')
        self.flowSetpointUnitsComboBox.addItems(self.flowRates)
        self.flowSetpointUnitsComboBox.setCurrentText('nL/min')
        self.errorBoundsLineEdit.setText('300')
        self.errorBoundsUnitsComboBox.addItem('%')
        self.errorBoundsUnitsComboBox.addItem(self.flowSetpointUnitsComboBox.currentText())
        self.errorBoundsUnitsComboBox.setCurrentText(self.flowSetpointUnitsComboBox.currentText())

        self.lockTimeUnitsComboBox.addItems(['sec', 'min', 'hr'])
        self.lockTimeUnitsComboBox.setCurrentText('min')
        self.lockTimeUnitsComboBox.currentIndexChanged.connect(self.UpdateLockTime)
        self.lockTimeLineEdit.setText('1')
        self.lockTimeLineEdit.editingFinished.connect(self.UpdateLockTime)
        self.currentReadingLCD.setSmallDecimalPoint(True)

        self.runStopButton.clicked.connect(self.RunStopData)
        self.currentlyRunning = False
        self.flowData = deque([], int(self.lockTime*1000/self.timerInterval))

        self.flowGraph.getAxis('left').setLabel('Flow ({})'.format(self.unitsComboBox.currentText()))
        self.flowGraph.getAxis('bottom').setLabel('Time (sec)')
        self.flowGraph.setYRange(-100, 8000)
        self.flowGraph.showGrid(x=True, y=True)
        self.flowCurve = self.flowGraph.plot()

        if path.isfile(self.globalSettings):
            self.LoadGlobalSettings(self.globalSettings)

    def LoadGlobalSettings(self, globalSettings):
        if path.isfile(globalSettings):
            self.globalSettings = globalSettings
            loadedSettings = yaml.load(self.globalSettings)

    def SaveGlobalSettings(self, globalSettings):
        if path.isfile(globalSettings):
            self.globalSettings = globalSettings

    def RunStopData(self):
        if self.currentlyRunning:
            self.currentlyRunning = False
            self.dataTimer.stop()
            self.runStopButton.setText('Run')
            self.timeData = deque([], int(30 * 60 * 1000 / self.timerInterval)) # store 30 min of data
            self.flowData = deque([], int(30 * 60 * 1000 / self.timerInterval)) # store 30 min of data
            self.sensorInfo.setText('Stopped')
        else:
            self.dataTimer.start(self.timerInterval)
            self.currentlyRunning = True
            reset_sensor(self.i2c_bus)
            sleep(0.5)

            self.sensor, self.serialNumber, _, _ = read_product_info(self.i2c_bus)
            self.scaleFactor, self.units, _, _ = read_scale_and_unit(self.i2c_bus)
            set_resolution(self.i2c_bus, bits=self.resolutionSettingSpinBox.value())
            set_read_data(self.i2c_bus)
            self.sensorInfo.setText('Running')
            self.runStopButton.setText('Stop')

    def UpdateData(self):
        self.timeData.append(time())
        rawFlowReading, _, _ = read_raw_data(self.i2c_bus)
        scaledFlowReading = scale_reading(rawFlowReading, self.scaleFactor)
        # scaledFlowReading = random.randrange(3000, 3500)
        self.flowData.append(scaledFlowReading)
        lowerBound = float(self.flowSetpointLineEdit.text()) - float(self.errorBoundsLineEdit.text())
        upperBound = float(self.flowSetpointLineEdit.text()) + float(self.errorBoundsLineEdit.text())
        if lowerBound < self.flowData[-1] < upperBound and len(self.flowData) == self.flowData.maxlen:
            if abs(mean(list(self.flowData)[:100]) - mean(list(self.flowData)[-100:])) < 50:
                if self.popUpDoneCheckBox.isChecked():
                    self.msg = QMessageBox()
                    self.msg.setWindowTitle('Experiment Progress')
                    self.msg.setText('All Done')
                    self.msg.setIcon(QMessageBox.Information)
                    self.msg.setStandardButtons(QMessageBox.Ok)
                    self.msg.show()
                print('All Done')
                self.RunStopData()
                return
            else:
                print('Not Yet')

        self.UpdateGraphs()

    def UpdateGraphs(self):
        if self.mainTabStack.currentIndex() == 0:
            updateGraph = {0 : self.UpdateFlowGraph,
                           1 : self.UpdatePressureGraph,
                           2 : self.UpdateCurrentGraph,
                           3 : self.UpdateVoltageGraph}
            updateGraph[self.graphsTab.currentIndex()]()

    def UpdateFlowGraph(self):
        flowVals = list(self.flowData)[-int(5 * 60 * 1000 / self.timerInterval):]
        timeVals = list(self.timeData)[-int(5 * 60 * 1000 / self.timerInterval):]
        timeVals = list(np.subtract(timeVals, timeVals[-1]))
        self.flowCurve.setData(timeVals, flowVals)
        self.currentReadingLCD.display(flowVals[-1])

    def UpdatePressureGraph(self):
        pass

    def UpdateCurrentGraph(self):
        pass

    def UpdateVoltageGraph(self):
        pass

    def UpdateLockTime(self):
        if self.timeUnitsDict[self.lockTimeUnitsComboBox.currentText()] * int(self.lockTimeLineEdit.text()) < 1 * 60: # don't allow a lock time shorter than 5 minutes
            if self.lockTimeUnitsComboBox.currentText() == 'min':
                self.lockTimeLineEdit.setText('5')
            else:
                self.lockTimeLineEdit.setText('300')
        else:
            self.lockTime = self.timeUnitsDict[self.lockTimeUnitsComboBox.currentText()] * int(self.lockTimeLineEdit.text())


def main():
    app = QApplication(sys.argv)
    form = StreamingPotentialApp()
    form.show()
    sys.exit(app.exec_())

def old_faithful():
    with SMBus(3) as bus:
        reset_sensor(bus)
        print("Sensor reset!")
        sleep(0.5)
        system('clear')

        sensor, serial_number, _, _ = read_product_info(bus)
        scale_factor, units, _, _ = read_scale_and_unit(bus)
        print('Sensor is:')
        print(sensor)
        print('Serial Number:')
        print(serial_number)
        print('Scale Factor:')
        print(scale_factor)
        print('Units:')
        print(units)
        sleep(3)

        set_read_data(bus)
        timeout_buffer = deque([], 3000)
        while True:
            raw_flow_reading, _, _ = read_raw_data(bus, True)
            scaled_flow_reading = scale_reading(raw_flow_reading, scale_factor)
            print('The current flow rate is: {} {}'.format(scaled_flow_reading, units))
            if 3100 < scaled_flow_reading < 3500:
                timeout_buffer.append((time(), scaled_flow_reading))
                if len(timeout_buffer) > 2500:
                    _, beggining_average = zip(*list(timeout_buffer)[:100])
                    beggining_average = mean(beggining_average)
                    _, ending_average = zip(*list(timeout_buffer)[-100:])
                    ending_average = mean(ending_average)
                    if abs(beggining_average-ending_average) < 100:
                        print('ALL DONE!!')
                    else:
                        print('Approaching the end')
            sleep(0.25)
            system('clear')

if __name__ == '__main__':
    main()
