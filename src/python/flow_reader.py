#!/usr/bin/env python3

from sf04_sensor.sf04 import *
import playsound
import visa
from time import sleep, time
from collections import deque
from statistics import mean
from os import system, path, remove
import sys
import yaml
from mainWindow import Ui_MainWindow
import numpy as np
from PyQt5 import QtCore, QtWidgets, QtGui
from PyQt5.QtWidgets import QApplication, QMainWindow, QMessageBox
from PyQt5.QtGui import QIntValidator, QDoubleValidator
import random

class StreamingPotentialApp(QMainWindow, Ui_MainWindow):
    def __init__(self, parent=None, lastSettings='global-settings'):
        super(StreamingPotentialApp, self).__init__(parent)
        self.setupUi(self)
        # Set the main page of the app
        self.mainTabStack.setCurrentIndex(0)
        self.graphsTab.setCurrentIndex(0)
        self.globalSettings = lastSettings
        self.visaResourceManager = visa.ResourceManager('@py')
        #TODO fix open resource name

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
        self.i2c_bus2 = SMBus(1)
        self.dataTimer = QtCore.QTimer(self)
        self.dataTimer.timeout.connect(self.UpdateData)
        self.timerInterval = 250 # update the graphs and data ever 250ms (aka 0.25s)
        self.lockTime = 5 * 60 # 5min * 60 sec
        self.timeData = deque([], int(self.lockTime * 1000 / self.timerInterval)) # store enough data for the lock in time
        self.unitsComboBox.addItems(self.flowRates)
        self.flowSetpointUnitsComboBox.addItems(self.flowRates)
        self.errorBoundsUnitsComboBox.addItem('%')
        self.errorBoundsUnitsComboBox.addItem(self.flowSetpointUnitsComboBox.currentText())

        self.lockTimeUnitsComboBox.addItems(['sec', 'min', 'hr'])
        self.lockTimeUnitsComboBox.setCurrentText('min')
        self.lockTimeUnitsComboBox.currentIndexChanged.connect(self.UpdateLockTime)
        self.lockTimeLineEdit.setText('5')
        self.lockTimeLineEdit.editingFinished.connect(self.UpdateLockTime)
        self.currentReadingLCD.setSmallDecimalPoint(True)

        self.runStopButton.clicked.connect(self.RunStopData)
        self.currentlyRunning = False
        self.flowData = deque([], int(self.lockTime * 1000 / self.timerInterval))

        self.flowGraph.getAxis('left').setLabel('Flow ({})'.format(self.unitsComboBox.currentText()))
        self.flowGraph.getAxis('bottom').setLabel('Time (sec)')
        self.flowGraph.setYRange(-100, 8000)
        self.flowGraph.showGrid(x=True, y=True)
        self.flowCurve = self.flowGraph.plot()

        if path.isfile(self.globalSettings):
            self.LoadGlobalSettings(self.globalSettings)

        self.resolutionSettingSpinBox.valueChanged.connect(self.SaveGlobalSettings)
        self.leftTransducerRadioButton.clicked.connect(self.SaveGlobalSettings)
        self.rightTransducerRadioButton.clicked.connect(self.SaveGlobalSettings)
        self.leftTransducerSlopeLineEdit.editingFinished.connect(self.SaveGlobalSettings)
        self.leftTransducerInterceptLineEdit.editingFinished.connect(self.SaveGlobalSettings)
        self.leftTransducerPressureUnitsLineEdit.editingFinished.connect(self.SaveGlobalSettings)
        self.leftTransducerSerialLineEdit.editingFinished.connect(self.SaveGlobalSettings)
        self.leftTransducerTypeLineEdit.editingFinished.connect(self.SaveGlobalSettings)
        self.rightTransducerSlopeLineEdit.editingFinished.connect(self.SaveGlobalSettings)
        self.rightTransducerInterceptLineEdit.editingFinished.connect(self.SaveGlobalSettings)
        self.rightTransducerPressureUnitsLineEdit.editingFinished.connect(self.SaveGlobalSettings)
        self.rightTransducerSerialLineEdit.editingFinished.connect(self.SaveGlobalSettings)
        self.rightTransducerTypeLineEdit.editingFinished.connect(self.SaveGlobalSettings)
        self.loadFlowReaderSettingsLineEdit.editingFinished.connect(self.SaveGlobalSettings)
        self.saveResultsNameLineEdit.editingFinished.connect(self.SaveGlobalSettings)
        self.crossSectionLineEdit.editingFinished.connect(self.SaveGlobalSettings)
        self.crossSectionUnitsComboBox.currentIndexChanged.connect(self.SaveGlobalSettings)
        self.tissueThicknessLineEdit.editingFinished.connect(self.SaveGlobalSettings)
        self.tissueThicknessUnitsComboBox.currentIndexChanged.connect(self.SaveGlobalSettings)
        self.flowSetpointLineEdit.editingFinished.connect(self.SaveGlobalSettings)
        self.flowSetpointUnitsComboBox.currentIndexChanged.connect(self.SaveGlobalSettings)
        self.averagingDepthSpinBox.valueChanged.connect(self.SaveGlobalSettings)
        self.errorBoundsLineEdit.editingFinished.connect(self.SaveGlobalSettings)
        self.errorBoundsUnitsComboBox.currentIndexChanged.connect(self.SaveGlobalSettings)
        self.lockTimeLineEdit.editingFinished.connect(self.SaveGlobalSettings)
        self.lockTimeUnitsComboBox.currentIndexChanged.connect(self.SaveGlobalSettings)
        self.soundDoneCheckBox.stateChanged.connect(self.SaveGlobalSettings)
        self.ledDoneCheckBox.stateChanged.connect(self.SaveGlobalSettings)
        self.popUpDoneCheckBox.stateChanged.connect(self.SaveGlobalSettings)
        self.unitsComboBox.currentIndexChanged.connect(self.SaveGlobalSettings)
        self.runStopButton.clicked.connect(self.SaveGlobalSettings)

    def LoadGlobalSettings(self, fileName):
        if path.isfile(fileName):
            self.globalSettingsFileName = fileName
            loadedSettings = yaml.load(self.globalSettingsFileName)

            self.resolutionSettingSpinBox.setValue(loadedSettings['Sensor']['Flow Meter'])

            self.leftTransducerRadioButton.setDown(loadedSettings['Sensor']['Pressure Transducer']['High Side-Left'])
            self.leftTransducerSlopeLineEdit.setText(loadedSettings['Sensor']['Pressure Transducer']['Left']['Slope'])
            self.leftTransducerInterceptLineEdit.setText(loadedSettings['Sensor']['Pressure Transducer']['Left']['Intercept'])
            self.leftTransducerPressureUnitsLineEdit.setText(loadedSettings['Sensor']['Pressure Transducer']['Left']['Units'])
            self.leftTransducerSerialLineEdit.setText(loadedSettings['Sensor']['Pressure Transducer']['Left']['Serial'])
            self.leftTransducerTypeLineEdit.setText(loadedSettings['Sensor']['Pressure Transducer']['Left']['Type'])

            self.rightTransducerRadioButton.setDown(loadedSettings['Sensor']['Pressure Transducer']['High Side-Right'])
            self.rightTransducerSlopeLineEdit.setText(loadedSettings['Sensor']['Pressure Transducer']['Right']['Slope'])
            self.rightTransducerInterceptLineEdit.setText(loadedSettings['Sensor']['Pressure Transducer']['Right']['Intercept'])
            self.rightTransducerPressureUnitsLineEdit.setText(loadedSettings['Sensor']['Pressure Transducer']['Right']['Units'])
            self.rightTransducerSerialLineEdit.setText(loadedSettings['Sensor']['Pressure Transducer']['Right']['Serial'])
            self.rightTransducerTypeLineEdit.setText(loadedSettings['Sensor']['Pressure Transducer']['Right']['Type'])

            self.saveResultsNameLineEdit.setText(loadedSettings['Experiment']['Results Save'])
            self.crossSectionLineEdit.setText(loadedSettings['Experiment']['Cross Section'])
            self.crossSectionUnitsComboBox.setCurrentText(loadedSettings['Experiment']['Cross Section Units'])
            self.tissueThicknessLineEdit.setText(loadedSettings['Experiment']['Tissue Thickness'])
            self.tissueThicknessUnitsComboBox.setCurrentText(loadedSettings['Experiment']['Tissue Thickness Units'])
            self.flowSetpointLineEdit.setText(loadedSettings['Experiment']['Flow Setpoint'])
            self.flowSetpointUnitsComboBox.setCurrentText(loadedSettings['Experiment']['Flow Setpoint Units'])
            self.averagingDepthSpinBox.value(loadedSettings['Experiment']['Averaging Depth'])
            self.errorBoundsLineEdit.setText(loadedSettings['Experiment']['Flow Error Bounds'])
            self.errorBoundsUnitsComboBox.setCurrentText(loadedSettings['Experiment']['Flow Error Units'])
            self.lockTimeLineEdit.setText(loadedSettings['Experiment']['Lock Time'])
            self.lockTimeUnitsComboBox.setCurrentText(loadedSettings['Experiment']['Lock Time Units'])
            self.soundDoneCheckBox.setDown(loadedSettings['Experiment']['Done Sound'])
            self.ledDoneCheckBox.setDown(loadedSettings['Experiment']['Done LED'])
            self.popUpDoneCheckBox.setDown(loadedSettings['Experiment']['Done Pop-Up'])
            self.loadExperimentSettingsLineEdit.setText(loadedSettings['Experiment']['Save'])

    def SaveGlobalSettings(self, fileName=None):
        if path.isfile(fileName):
            self.globalSettingsFileName = fileName
        self.SaveSensorSettings(0xDEADBEEF)
        self.SaveExperimentSettings(0xDEADBEEF)
        sensorSettings = yaml.load('temp-sensor-settings')
        exmperimentSettings = yaml.load('temp-experiment-settings')
        saveData = {'Sensor' : sensorSettings,
                    'Experiment' : exmperimentSettings}
        remove('temp-sensor-settings')
        remove('temp-experiment-settings')
        print(saveData)
        if fileName is None:
            with open('global-settings', 'w') as saveSensor:
                saveSensor.write(yaml.dump(saveData))

    def SaveSensorSettings(self, fileName=None):
        saveData = {'Flow Meter' : self.resolutionSettingSpinBox.value(),
                    'Pressure Transducer' : {'High Side-Left' : self.leftTransducerRadioButton.isChecked(),
                                             'High Side-Right' : self.rightTransducerRadioButton.isChecked(),
                                             'Left' : {'Slope' : self.leftTransducerSlopeLineEdit.text(),
                                                       'Intercept' : self.leftTransducerInterceptLineEdit.text(),
                                                       'Units' : self.leftTransducerPressureUnitsLineEdit.text(),
                                                       'Serial' : self.leftTransducerSerialLineEdit.text(),
                                                       'Type' : self.leftTransducerTypeLineEdit.text()},
                                             'Right' : {'Slope' : self.rightTransducerSlopeLineEdit.text(),
                                                       'Intercept' : self.rightTransducerInterceptLineEdit.text(),
                                                       'Units' : self.rightTransducerPressureUnitsLineEdit.text(),
                                                       'Serial' : self.rightTransducerSerialLineEdit.text(),
                                                       'Type' : self.rightTransducerTypeLineEdit.text()}},
                    'Save' : self.loadFlowReaderSettingsLineEdit.text()}
        if fileName is None:
            with open(self.loadFlowReaderSettingsLineEdit.text(), 'w') as saveSensor:
                saveSensor.write(yaml.dump(saveData))
        if path.isfile(fileName):
            saveData['Save'] = fileName
            with open(self.loadFlowReaderSettingsLineEdit.text(), 'w') as saveSensor:
                saveSensor.write(yaml.dump(saveData))
        if fileName is 0xDEADBEEF:
            with open('temp-sensor-settings', 'w') as saveSensor:
                saveSensor.write(yaml.dump(saveData))

    def SaveExperimentSettings(self, fileName=None):
        saveData = {'Results Save' : self.saveResultsNameLineEdit.text(),
                    'Cross Section' : self.crossSectionLineEdit.text(),
                    'Cross Section Units' : self.crossSectionUnitsComboBox.currentText(),
                    'Tissue Thickness' : self.tissueThicknessLineEdit.text(),
                    'Tissue Thickness Units' : self.tissueThicknessUnitsComboBox.currentText(),
                    'Flow Setpoint' : self.flowSetpointLineEdit.text(),
                    'Flow Setpoint Units' : self.flowSetpointUnitsComboBox.currentText(),
                    'Averaging Depth' : self.averagingDepthSpinBox.value(),
                    'Flow Error Bounds' : self.errorBoundsLineEdit.text(),
                    'Flow Error Units' : self.errorBoundsUnitsComboBox.currentText(),
                    'Lock Time' : self.lockTimeLineEdit.text(),
                    'Lock Time Units' : self.lockTimeUnitsComboBox.currentText(),
                    'Done Sound' : self.soundDoneCheckBox.isChecked(),
                    'Done LED' : self.ledDoneCheckBox.isChecked(),
                    'Done Pop-Up' : self.popUpDoneCheckBox.isChecked(),
                    'Save' : self.loadExperimentSettingsLineEdit.text()}
        if fileName is None:
            with open(self.loadExperimentSettingsLineEdit.text(), 'w') as saveSensor:
                saveSensor.write(yaml.dump(saveData))
        if path.isfile(fileName):
            saveData['Save'] = fileName
            with open(self.loadExperimentSettingsLineEdit.text(), 'w') as saveSensor:
                saveSensor.write(yaml.dump(saveData))
        if fileName is 0xDEADBEEF:
            with open('temp-experiment-settings', 'w') as saveSensor:
                saveSensor.write(yaml.dump(saveData))

    def RunStopData(self):
        if self.currentlyRunning:
            self.currentlyRunning = False
            self.dataTimer.stop()
            self.sourceMeter.close()
            self.runStopButton.setText('Run')
            self.timeData = deque([], int(self.lockTime * 1000 / self.timerInterval)) # store 30 min of data
            self.flowData = deque([], int(self.lockTime * 1000 / self.timerInterval)) # store 30 min of data
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

            self.sourceMeter = self.visaResourceManager.open_resource('ACRL::')
            self.sourceMeter.write('*RST')
            self.sourceMeter.write(":SENS:FUNC 'RES' ")
            self.sourceMeter.write(":SENS:RES:NPLC 1")
            self.sourceMeter.write(":SENS:RES:MODE AUTO")

    def UpdateData(self):
        self.timeData.append(time())
        rawFlowReading, _, _ = read_raw_data(self.i2c_bus)
        scaledFlowReading = scale_reading(rawFlowReading, self.scaleFactor)
        # scaledFlowReading = random.randrange(3000, 3500)
        self.flowData.append(scaledFlowReading)
        lowerBound = float(self.flowSetpointLineEdit.text()) - float(self.errorBoundsLineEdit.text())
        upperBound = float(self.flowSetpointLineEdit.text()) + float(self.errorBoundsLineEdit.text())
        if lowerBound < self.flowData[-1] < upperBound and len(self.flowData) == self.flowData.maxlen:
            if abs(mean(list(self.flowData)[:100]) - mean(list(self.flowData)[-100:])) < 25:
                if self.popUpDoneCheckBox.isChecked():
                    self.msg = QMessageBox()
                    self.msg.setWindowTitle('Experiment Progress')
                    self.msg.setText('All Done')
                    self.msg.setIcon(QMessageBox.Information)
                    self.msg.setStandardButtons(QMessageBox.Ok)
                    self.msg.show()
                if self.soundDoneCheckBox.isChecked():
                    playsound.playsound('done.wav')
                self.RunStopData()
                return

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
        if self.timeUnitsDict[self.lockTimeUnitsComboBox.currentText()] * int(self.lockTimeLineEdit.text()) < 5 * 60: # don't allow a lock time shorter than 5 minutes
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
