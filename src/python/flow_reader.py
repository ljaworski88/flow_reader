#!/usr/bin/env python3

from sf04_sensor.sf04 import *
from nau7802 import *
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
import serial
# import RPi.GPIO as gpio

class StreamingPotentialApp(QMainWindow, Ui_MainWindow):
    def __init__(self, parent=None, lastSettings='global-settings'):
        super(StreamingPotentialApp, self).__init__(parent)
        self.setupUi(self)
        self.setWindowTitle('Streaming Potential Calculator')
        self.setWindowIcon(QtGui.QIcon('logo.png'))
        # Set the main page of the app
        self.mainTabStack.setCurrentIndex(0)
        self.graphsTab.setCurrentIndex(0)
        self.globalSettings = lastSettings
        self.visaResourceManager = visa.ResourceManager('@py')
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
        self.thicknessUnitsDict = {'m'  : 1,
                                   'cm' : 1e-2,
                                   'mm' : 1e-3,
                                   'um' : 1e-6,
                                   'nm' : 1e-9}
        self.areaUnitsDict = {'m^2'  : 1,
                              'cm^2' : 1e-4,
                              'mm^2' : 1e-9,
                              'um^2' : 1e-36,
                              'nm^2' : 1e-81}
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
        # self.i2c_bus = SMBus(3)
        # self.i2c_bus2 = SMBus(1)
        self.dataTimer = QtCore.QTimer(self)
        self.dataTimer.timeout.connect(self.UpdateData)
        self.timerInterval = 250 # update the graphs and data ever 250ms (aka 0.25s)
        self.lockTime = 5 * 60 # 5min * 60 sec
        self.timeData = deque([], int(self.lockTime * 1000 / self.timerInterval)) # store enough data for the lock in time
        self.unitsComboBox.addItems(self.flowRates)
        self.unitsComboBox.setCurrentText('nL/min')
        self.flowSetpointUnitsComboBox.addItems(self.flowRates)
        self.errorBoundsUnitsComboBox.addItem('%')
        self.errorBoundsUnitsComboBox.addItem(self.flowSetpointUnitsComboBox.currentText())
        self.crossSectionUnitsComboBox.addItems(self.areaUnitsDict.keys())
        self.tissueThicknessUnitsComboBox.addItems(self.thicknessUnitsDict.keys())
        self.leftTransducerUnitComboBox.addItems(self.pressureUnitsDict.keys())
        self.rightTransducerUnitComboBox.addItems(self.pressureUnitsDict.keys())

        self.lockTimeUnitsComboBox.addItems(['sec', 'min', 'hr'])
        self.lockTimeUnitsComboBox.setCurrentText('min')
        self.lockTimeUnitsComboBox.currentIndexChanged.connect(self.UpdateLockTime)
        self.lockTimeLineEdit.setText('5')
        self.lockTimeLineEdit.editingFinished.connect(self.UpdateLockTime)
        self.currentReadingLCD.setSmallDecimalPoint(True)
        self.currentReadingLCD.setNumDigits(9)

        self.runStopButton.clicked.connect(self.SaveGlobalSettings)
        self.runStopButton.clicked.connect(self.RunStopData)
        self.currentlyRunning = False
        self.flowData = deque([], int(self.lockTime * 1000 / self.timerInterval))
        self.pressureData = deque([], int(self.lockTime * 1000 / self.timerInterval))
        self.voltageData = deque([], int(self.lockTime * 1000 / self.timerInterval))
        self.currentData = deque([], int(self.lockTime * 1000 / self.timerInterval))

        self.flowGraph.getAxis('left').setLabel('Flow ({})'.format(self.unitsComboBox.currentText()))
        self.flowGraph.getAxis('bottom').setLabel('Time (sec)')
        self.flowGraph.setYRange(-100, 8000)
        self.flowGraph.showGrid(x=True, y=True)
        self.flowCurve = self.flowGraph.plot()

        self.pressureGraph.getAxis('left').setLabel('Pressure ({})'.format('Pa'))
        self.pressureGraph.getAxis('bottom').setLabel('Time (sec)')
        self.pressureGraph.setYRange(-100, 800000)
        self.pressureGraph.showGrid(x=True, y=True)
        self.pressureCurve = self.pressureGraph.plot()

        self.voltageGraph.getAxis('left').setLabel('Voltage ({})'.format('V'))
        self.voltageGraph.getAxis('bottom').setLabel('Time (sec)')
        self.voltageGraph.setYRange(-5, 5)
        self.voltageGraph.showGrid(x=True, y=True)
        self.voltageCurve = self.voltageGraph.plot()

        self.currentGraph.getAxis('left').setLabel('Current ({})'.format('A'))
        self.currentGraph.getAxis('bottom').setLabel('Time (sec)')
        self.currentGraph.setYRange(-5, 5)
        self.currentGraph.showGrid(x=True, y=True)
        self.currentCurve = self.currentGraph.plot()

        if path.isfile(self.globalSettings):
            self.LoadGlobalSettings(self.globalSettings)

        self.resolutionSettingSpinBox.valueChanged.connect(self.SaveGlobalSettings)
        self.leftTransducerRadioButton.clicked.connect(self.SaveGlobalSettings)
        self.rightTransducerRadioButton.clicked.connect(self.SaveGlobalSettings)
        self.leftTransducerSlopeLineEdit.editingFinished.connect(self.SaveGlobalSettings)
        self.leftTransducerInterceptLineEdit.editingFinished.connect(self.SaveGlobalSettings)
        self.leftTransducerSerialLineEdit.editingFinished.connect(self.SaveGlobalSettings)
        self.leftTransducerUnitComboBox.currentIndexChanged.connect(self.SaveGlobalSettings)
        self.rightTransducerSlopeLineEdit.editingFinished.connect(self.SaveGlobalSettings)
        self.rightTransducerInterceptLineEdit.editingFinished.connect(self.SaveGlobalSettings)
        self.rightTransducerSerialLineEdit.editingFinished.connect(self.SaveGlobalSettings)
        self.rightTransducerUnitComboBox.currentIndexChanged.connect(self.SaveGlobalSettings)
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

    def LoadGlobalSettings(self, fileName):
        if path.isfile(fileName):
            self.globalSettingsFileName = fileName
            with open(self.globalSettingsFileName, 'r') as settings:
                loadedSettings = yaml.full_load(settings)

            self.resolutionSettingSpinBox.setValue(loadedSettings['Sensor']['Flow Meter'])

            self.leftTransducerRadioButton.setChecked(loadedSettings['Sensor']['Pressure Transducer']['High Side-Left'])
            self.leftTransducerSlopeLineEdit.setText(loadedSettings['Sensor']['Pressure Transducer']['Left']['Slope'])
            self.leftTransducerInterceptLineEdit.setText(loadedSettings['Sensor']['Pressure Transducer']['Left']['Intercept'])
            self.leftTransducerSerialLineEdit.setText(loadedSettings['Sensor']['Pressure Transducer']['Left']['Serial'])
            self.leftTransducerUnitComboBox.setCurrentText(loadedSettings['Sensor']['Pressure Transducer']['Left']['Units'])

            self.rightTransducerRadioButton.setChecked(loadedSettings['Sensor']['Pressure Transducer']['High Side-Right'])
            self.rightTransducerSlopeLineEdit.setText(loadedSettings['Sensor']['Pressure Transducer']['Right']['Slope'])
            self.rightTransducerInterceptLineEdit.setText(loadedSettings['Sensor']['Pressure Transducer']['Right']['Intercept'])
            self.rightTransducerSerialLineEdit.setText(loadedSettings['Sensor']['Pressure Transducer']['Right']['Serial'])
            self.rightTransducerUnitComboBox.setCurrentText(loadedSettings['Sensor']['Pressure Transducer']['Right']['Units'])

            self.saveResultsNameLineEdit.setText(loadedSettings['Experiment']['Results Save'])
            self.crossSectionLineEdit.setText(loadedSettings['Experiment']['Cross Section'])
            self.crossSectionUnitsComboBox.setCurrentText(loadedSettings['Experiment']['Cross Section Units'])
            self.tissueThicknessLineEdit.setText(loadedSettings['Experiment']['Tissue Thickness'])
            self.tissueThicknessUnitsComboBox.setCurrentText(loadedSettings['Experiment']['Tissue Thickness Units'])
            self.flowSetpointLineEdit.setText(loadedSettings['Experiment']['Flow Setpoint'])
            self.flowSetpointUnitsComboBox.setCurrentText(loadedSettings['Experiment']['Flow Setpoint Units'])
            self.averagingDepthSpinBox.setValue(loadedSettings['Experiment']['Averaging Depth'])
            self.errorBoundsLineEdit.setText(loadedSettings['Experiment']['Flow Error Bounds'])
            self.errorBoundsUnitsComboBox.setCurrentText(loadedSettings['Experiment']['Flow Error Units'])
            self.lockTimeLineEdit.setText(loadedSettings['Experiment']['Lock Time'])
            self.lockTimeUnitsComboBox.setCurrentText(loadedSettings['Experiment']['Lock Time Units'])
            self.soundDoneCheckBox.setChecked(loadedSettings['Experiment']['Done Sound'])
            self.ledDoneCheckBox.setChecked(loadedSettings['Experiment']['Done LED'])
            self.popUpDoneCheckBox.setChecked(loadedSettings['Experiment']['Done Pop-Up'])
            self.loadExperimentSettingsLineEdit.setText(loadedSettings['Experiment']['Save'])

    def SaveGlobalSettings(self):
        sensorSettings = self.SaveSensorSettings(0xDEADBEEF)
        exmperimentSettings = self.SaveExperimentSettings(0xDEADBEEF)
        saveData = {'Sensor' : sensorSettings,
                    'Experiment' : exmperimentSettings}
        with open('global-settings', 'w') as saveSettings:
            yaml.dump(saveData, saveSettings)

    def SaveSensorSettings(self, fileName=None):
        saveData = {'Flow Meter' : self.resolutionSettingSpinBox.value(),
                    'Pressure Transducer' : {'High Side-Left' : self.leftTransducerRadioButton.isChecked(),
                                             'High Side-Right' : self.rightTransducerRadioButton.isChecked(),
                                             'Left' : {'Slope' : self.leftTransducerSlopeLineEdit.text(),
                                                       'Intercept' : self.leftTransducerInterceptLineEdit.text(),
                                                       'Units' : self.leftTransducerUnitComboBox.currentText(),
                                                       'Serial' : self.leftTransducerSerialLineEdit.text()},
                                             'Right' : {'Slope' : self.rightTransducerSlopeLineEdit.text(),
                                                       'Intercept' : self.rightTransducerInterceptLineEdit.text(),
                                                       'Units' : self.rightTransducerUnitComboBox.currentText(),
                                                       'Serial' : self.rightTransducerSerialLineEdit.text()}},
                    'Save' : self.loadFlowReaderSettingsLineEdit.text()}
        if fileName == 0xDEADBEEF:
            return saveData
        else:
            if fileName is None:
                with open(self.loadFlowReaderSettingsLineEdit.text(), 'w') as saveSensor:
                    yaml.dump(saveData, saveSensor)
            if path.isfile(fileName):
                saveData['Save'] = fileName
                with open(self.loadFlowReaderSettingsLineEdit.text(), 'w') as saveSensor:
                    yaml.dump(saveData, saveSensor)

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
        if fileName == 0xDEADBEEF:
            return saveData
        else:
            if fileName is None:
                with open(self.loadExperimentSettingsLineEdit.text(), 'w') as saveSensor:
                    yaml.dump(saveData, saveSensor)
            if path.isfile(fileName):
                saveData['Save'] = fileName
                with open(fileName, 'w') as saveSensor:
                    yaml.dump(saveData, saveSensor)
    def LoadCailbration(self, transducer):
        pass

    def LoadSensorSettings(self):
        pass

    def LoadExperimentSettings(self):
        pass

    def RunStopData(self):
        if self.currentlyRunning:
            self.currentlyRunning = False
            self.dataTimer.stop()
            # self.sourceMeter.close()
            self.runStopButton.setText('Run')
            self.timeData = deque([], int(self.lockTime * 1000 / self.timerInterval))
            self.flowData = deque([], int(self.lockTime * 1000 / self.timerInterval))
            self.rightPressureData = deque([], int(self.lockTime * 1000 / self.timerInterval))
            self.leftPressureData = deque([], int(self.lockTime * 1000 / self.timerInterval))
            self.voltageData = deque([], int(self.lockTime * 1000 / self.timerInterval))
            self.currentData = deque([], int(self.lockTime * 1000 / self.timerInterval))
            self.sensorInfo.setText('Stopped')
        else:
            self.dataTimer.start(self.timerInterval)
            self.timeData = deque([], int(self.lockTime * 1000 / self.timerInterval))
            self.flowData = deque([], int(self.lockTime * 1000 / self.timerInterval))
            self.rightPressureData = deque([], int(self.lockTime * 1000 / self.timerInterval))
            self.leftPressureData = deque([], int(self.lockTime * 1000 / self.timerInterval))
            self.voltageData = deque([], int(self.lockTime * 1000 / self.timerInterval))
            self.currentData = deque([], int(self.lockTime * 1000 / self.timerInterval))
            self.currentlyRunning = True
            if not self.leftTransducerSlopeLineEdit.text() or not self.rightTransducerSlopeLineEdit.text():
                raise(ValueError('Both left and right transducers need calibration values'))
            if not self.leftTransducerInterceptLineEdit.text() or not self.rightTransducerInterceptLineEdit.text():
                raise(ValueError('Both left and right transducers need calibration values'))

            # Reset the pressure and flow chips on the I2C buses

            # boot_cycle(i2c_bus)
            # boot_cycle(i2c_bus2)
            # reset_sensor(self.i2c_bus)
            # self.sourceMeter = self.visaResourceManager.open_resource('ASRL/dev/ttyUSB0::INSTR')
            # self.sourceMeter.write("*RST")
            sleep(0.5)

            # Set up the pressure sensors

            # set_avdd_voltage(i2c_bus, '4.5v')
            # select_avdd_source(i2c_bus, internal_source=True)
            # set_conversion_rate(i2c_bus, conversion_rate='sps10')
            # set_gain(i2c_bus, 'x1')
            # start_reading_data(i2c_bus, start=True)
            # set_avdd_voltage(i2c_bus2, '4.5v')
            # select_avdd_source(i2c_bus2, internal_source=True)
            # set_conversion_rate(i2c_bus2, conversion_rate='sps10')
            # set_gain(i2c_bus2, 'x1')
            # start_reading_data(i2c_bus2, start=True)

            # Set up the flow sensor

            # self.sensor, self.serialNumber, _, _ = read_product_info(self.i2c_bus)
            # self.scaleFactor, self.units, _, _ = read_scale_and_unit(self.i2c_bus)
            # set_resolution(self.i2c_bus, bits=self.resolutionSettingSpinBox.value())
            # set_read_data(self.i2c_bus)

            # Set up the sourcemeter for voltage and current

            # self.sourceMeter.write(":SENS:FUNC 'RES' ")
            # self.sourceMeter.write(":SENS:RES:NPLC 1")
            # self.sourceMeter.write(":SENS:RES:MODE AUTO")
            self.sensorInfo.setText('Running')
            self.runStopButton.setText('Stop')


    def PressureDifferential(self, rawLeftTransducerReading, rawRightTransducerReading):
        leftReading = float(self.leftTransducerSlopeLineEdit) * rawLeftTransducerReading + self.leftTransducerInterceptLineEdit
        rightReading = float(self.rightTransducerSlopeLineEdit) * rawRightTransducerReading + self.rightTransducerInterceptLineEdit
        if self.leftTransducerRadioButton.isChecked():
            return leftReading - rightReading
        else:
            return rightReading - leftReading


    def UpdateData(self):
        self.timeData.append(time())
        # rawFlowReading, _, _ = read_raw_data(self.i2c_bus)
        # scaledFlowReading = scale_reading(rawFlowReading, self.scaleFactor)
        # rawLeftTransducerReading = read_load(i2c_bus)
        # rawRightTransducerReading = read_load(i2c_bus2)
        # pressureDifferentialReading = self.PressureDifferential(rawLeftTransducerReading, rawRightTransducerReading)
        # voltageReading = self.sourceMeter.query('*READ')
        # currentReading = self.sourceMeter.query('*READ')

        ## Not on Pi Testing
        scaledFlowReading = random.randrange(3000, 3500)
        pressureDifferentialReading = random.randrange(100000, 200000)
        voltageReading = random.randrange(-1, 1)
        currentReading = random.randrange(-1, 1)
        self.flowData.append(scaledFlowReading)
        self.pressureData.append(pressureDifferentialReading)
        self.voltageData.append(voltageReading)
        self.currentData.append(currentReading)
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
                if self.ledDoneCheckBox.isChecked():
                    pass
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
        pressureVals = list(self.pressureData)[-int(5 * 60 * 1000 / self.timerInterval):]
        timeVals = list(self.timeData)[-int(5 * 60 * 1000 / self.timerInterval):]
        timeVals = list(np.subtract(timeVals, timeVals[-1]))
        self.pressureCurve.setData(timeVals, pressureVals)
        self.currentReadingLCD.display(pressureVals[-1])

    def UpdateVoltageGraph(self):
        voltageVals = list(self.voltageData)[-int(5 * 60 * 1000 / self.timerInterval):]
        timeVals = list(self.timeData)[-int(5 * 60 * 1000 / self.timerInterval):]
        timeVals = list(np.subtract(timeVals, timeVals[-1]))
        self.voltageCurve.setData(timeVals, voltageVals)
        self.currentReadingLCD.display(voltageVals[-1])

    def UpdateCurrentGraph(self):
        currentVals = list(self.currentData)[-int(5 * 60 * 1000 / self.timerInterval):]
        timeVals = list(self.timeData)[-int(5 * 60 * 1000 / self.timerInterval):]
        timeVals = list(np.subtract(timeVals, timeVals[-1]))
        self.currentCurve.setData(timeVals, currentVals)
        self.currentReadingLCD.display(currentVals[-1])

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

if __name__ == '__main__':
    main()
