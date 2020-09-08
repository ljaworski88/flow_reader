#!/usr/bin/env python3

from sf04_sensor.sf04 import *
from nau7802 import *
import visa
from time import sleep, time
from collections import deque
from statistics import mean
from os import path
from subprocess import Popen
import sys
import yaml
import csv
import zipfile
from mainWindow import Ui_MainWindow
import numpy as np
from sklearn.linear_model import LinearRegression
from PyQt5 import QtCore, QtWidgets, QtGui
from PyQt5.QtWidgets import QApplication, QMainWindow, QMessageBox, QFileDialog
from PyQt5.QtGui import QIntValidator, QDoubleValidator
import pyqtgraph
import random
import serial
## Pi-start
import RPi.GPIO as gpio
## Pi-end

class StreamingPotentialApp(QMainWindow, Ui_MainWindow):
    def __init__(self, parent=None, lastSettings='global-settings'):
        super(StreamingPotentialApp, self).__init__(parent)
        self.setupUi(self)
        self.setWindowTitle('Streaming Potential Calculator')
        self.setWindowIcon(QtGui.QIcon('logo.png'))

        ## Set the main page of the app
        self.mainTabStack.setCurrentIndex(0)
        self.graphsTab.setCurrentIndex(0)
        self.globalSettings = lastSettings

        ## Initalize some library resource, state variables, and data arrays for later
        self.graphUnits = {0 : ['nL/min', 1e-9/60],
                           1 : ['Pa', 1],
                           2 : ['A', 1],
                           3 : ['V', 1]}
        self.calibrationUnits = 'Pa'
        self.r_sq = None
        self.readingToPressureSlope = None
        self.readingToPressureIntercept = None
        ## Pi-start
        gpio.setmode(gpio.BCM)
        self.doneLED = 9
        self.errorLED = 11
        gpio.setup(self.doneLED, gpio.OUT)
        gpio.setup(self.errorLED, gpio.OUT)
        ## Pi-end

        self.dataTimer = QtCore.QTimer(self)
        self.timerInterval = 250 # update the graphs and data ever 250ms (aka 0.25s)
        self.lockTime = 5 * 60 # 5min * 60 sec

        self.currentlyRunning = False
        self.logData = False

        self.timeData = deque([], int(self.lockTime * 1000 / self.timerInterval)) # store enough data for the lock in time
        self.flowData = deque([], int(self.lockTime * 1000 / self.timerInterval))
        self.pressureData = deque([], int(self.lockTime * 1000 / self.timerInterval))
        self.voltageData = deque([], int(self.lockTime * 1000 / self.timerInterval))
        self.currentData = deque([], int(self.lockTime * 1000 / self.timerInterval))
        self.calibrationAverages = []
        self.calibrationPressures = []
        ## Pi-start
        self.i2c_bus = SMBus(3)
        self.i2c_bus2 = SMBus(1)
        ## Pi-end
        self.visaResourceManager = visa.ResourceManager('@py')

        ## Initialize some dicts and lists that will be used in conversions later
        self.timeUnitsDict = {'sec' : 1,
                              'min' : 60,
                              'hr'  : 3600}

        self.volumeUnitsDict = {'L'  : 1,
                                'mL' : 1e-3,
                                'uL' : 1e-6,
                                'nL' : 1e-9,
                                'm^3': 1000}

        self.currentUnitsDict = {'A'  : 1,
                                 'mA' : 1e-3,
                                 'uA' : 1e-6,
                                 'nA' : 1e-9}

        self.voltageUnitsDict = {'V'  : 1,
                                 'mV' : 1e-3,
                                 'uV' : 1e-6,
                                 'nV' : 1e-9}

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

        ## Set input validators
        self.flowSetpointLineEdit.setValidator(QDoubleValidator())
        self.errorBoundsLineEdit.setValidator(QDoubleValidator())
        self.crossSectionLineEdit.setValidator(QDoubleValidator())
        self.tissueThicknessLineEdit.setValidator(QDoubleValidator())
        self.transducer1SlopeLineEdit.setValidator(QDoubleValidator())
        self.transducer1InterceptLineEdit.setValidator(QDoubleValidator())
        self.transducer2SlopeLineEdit.setValidator(QDoubleValidator())
        self.transducer2InterceptLineEdit.setValidator(QDoubleValidator())
        self.lockTimeLineEdit.setValidator(QIntValidator())
        self.addCalibrationDataLineEdit.setValidator(QDoubleValidator())

        ## Set Combo Box options
        self.statusMessage = 'Welcome!'
        self.unitsComboBox.addItems(self.flowRates)
        self.unitsComboBox.setCurrentText(self.graphUnits[0][0])
        self.flowSetpointUnitsComboBox.addItems(self.flowRates)
        self.errorBoundsUnitsComboBox.addItem('%')
        self.errorBoundsUnitsComboBox.addItem(self.flowSetpointUnitsComboBox.currentText())
        self.crossSectionUnitsComboBox.addItems(self.areaUnitsDict.keys())
        self.tissueThicknessUnitsComboBox.addItems(self.thicknessUnitsDict.keys())
        self.transducer2UnitComboBox.addItems(self.pressureUnitsDict.keys())
        self.transducer1UnitComboBox.addItems(self.pressureUnitsDict.keys())
        self.calibrationUnitsComboBox.addItems(self.pressureUnitsDict.keys())
        self.lockTimeUnitsComboBox.addItems(['sec', 'min', 'hr'])
        self.lockTimeUnitsComboBox.setCurrentText('min')
        self.lockTimeLineEdit.setText('5')
        self.currentReadingLCD.setSmallDecimalPoint(True)
        self.currentReadingLCD.setNumDigits(9)
        self.calibrationLCD.setSmallDecimalPoint(True)
        self.calibrationLCD.setNumDigits(9)

        ## Set-up Graphs
        self.flowGraph.getAxis('left').setLabel('Flow ({})'.format(self.graphUnits[0][0]))
        self.flowGraph.getAxis('bottom').setLabel('Time (sec)')
        self.flowGraph.setYRange((-1/6e8)/self.graphUnits[0][1], (8/6e7)/self.graphUnits[0][1])
        self.flowGraph.showGrid(x=True, y=True)
        self.flowCurve = self.flowGraph.plot()

        self.pressureGraph.getAxis('left').setLabel('Pressure ({})'.format(self.graphUnits[1][0]))
        self.pressureGraph.getAxis('bottom').setLabel('Time (sec)')
        self.pressureGraph.setYRange(-100, 800000)
        self.pressureGraph.showGrid(x=True, y=True)
        self.pressureCurve = self.pressureGraph.plot()

        self.currentGraph.getAxis('left').setLabel('Current ({})'.format(self.graphUnits[2][0]))
        self.currentGraph.getAxis('bottom').setLabel('Time (sec)')
        self.currentGraph.setYRange(-100e-3, 100e-3)
        self.currentGraph.showGrid(x=True, y=True)
        self.currentCurve = self.currentGraph.plot()

        self.voltageGraph.getAxis('left').setLabel('Voltage ({})'.format(self.graphUnits[3][0]))
        self.voltageGraph.getAxis('bottom').setLabel('Time (sec)')
        self.voltageGraph.setYRange(-100e-3, 100e-3)
        self.voltageGraph.showGrid(x=True, y=True)
        self.voltageCurve = self.voltageGraph.plot()

        self.pressureCalibrationGraph.getAxis('left').setLabel('Reading Value')
        self.pressureCalibrationGraph.getAxis('bottom').setLabel('Pressure ({})'.format(self.calibrationUnits))
        self.pressureCalibrationGraph.showGrid(x=True, y=True)
        self.pressureCalibrationGraphViewBox = self.pressureCalibrationGraph.getViewBox()
        self.pressureCalibrationGraphText = pyqtgraph.TextItem(text='Slope:\nIntercept:\nR^2:')
        self.pressureCalibrationGraphText.setParentItem(self.pressureCalibrationGraphViewBox)
        self.pressureCalibrationCurve = self.pressureCalibrationGraph.plot(symbol='o', pen=pyqtgraph.mkPen('b', width=3))
        self.pressurePredictionCurve = self.pressureCalibrationGraph.plot(pen=pyqtgraph.mkPen('r', width=2))

        if path.isfile(self.globalSettings):
            self.LoadGlobalSettings(self.globalSettings)

        ## Signal emitters
        self.dataTimer.timeout.connect(self.UpdateData)
        self.lockTimeLineEdit.editingFinished.connect(self.UpdateLockTime)
        self.lockTimeUnitsComboBox.currentIndexChanged.connect(self.UpdateLockTime)
        self.runStopButton.clicked.connect(self.RunStopData)
        self.logButton.clicked.connect(self.StartStopLogging)
        self.addCalibrationDataButton.clicked.connect(self.AddCalibrationPoint)
        self.graphsTab.currentChanged.connect(self.UpdateGraphUnits)
        self.flowSetpointUnitsComboBox.currentIndexChanged.connect(self.UpdateErrorBounds)
        self.unitsComboBox.activated.connect(self.UpdateAxis)
        self.calibrationUnitsComboBox.currentIndexChanged.connect(self.AdjustCalibration)
        self.saveCalibrationButton.clicked.connect(self.SaveCalibrationData)

        ## Save settings Signal emitters
        self.runStopButton.clicked.connect(self.SaveGlobalSettings)
        self.resolutionSettingSpinBox.valueChanged.connect(self.SaveGlobalSettings)
        self.transducer2RadioButton.clicked.connect(self.SaveGlobalSettings)
        self.transducer1RadioButton.clicked.connect(self.SaveGlobalSettings)
        self.transducer2SlopeLineEdit.editingFinished.connect(self.SaveGlobalSettings)
        self.transducer2InterceptLineEdit.editingFinished.connect(self.SaveGlobalSettings)
        self.transducer2SerialLineEdit.editingFinished.connect(self.SaveGlobalSettings)
        self.transducer2UnitComboBox.currentIndexChanged.connect(self.SaveGlobalSettings)
        self.transducer1SlopeLineEdit.editingFinished.connect(self.SaveGlobalSettings)
        self.transducer1InterceptLineEdit.editingFinished.connect(self.SaveGlobalSettings)
        self.transducer1SerialLineEdit.editingFinished.connect(self.SaveGlobalSettings)
        self.transducer1UnitComboBox.currentIndexChanged.connect(self.SaveGlobalSettings)
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

            self.transducer2RadioButton.setChecked(loadedSettings['Sensor']['Pressure Transducer']['High Side-Transducer2'])
            self.transducer2SlopeLineEdit.setText(loadedSettings['Sensor']['Pressure Transducer']['Transducer2']['Slope'])
            self.transducer2InterceptLineEdit.setText(loadedSettings['Sensor']['Pressure Transducer']['Transducer2']['Intercept'])
            self.transducer2SerialLineEdit.setText(loadedSettings['Sensor']['Pressure Transducer']['Transducer2']['Serial'])
            self.transducer2UnitComboBox.setCurrentText(loadedSettings['Sensor']['Pressure Transducer']['Transducer2']['Units'])

            self.transducer1RadioButton.setChecked(loadedSettings['Sensor']['Pressure Transducer']['High Side-Transducer1'])
            self.transducer1SlopeLineEdit.setText(loadedSettings['Sensor']['Pressure Transducer']['Transducer1']['Slope'])
            self.transducer1InterceptLineEdit.setText(loadedSettings['Sensor']['Pressure Transducer']['Transducer1']['Intercept'])
            self.transducer1SerialLineEdit.setText(loadedSettings['Sensor']['Pressure Transducer']['Transducer1']['Serial'])
            self.transducer1UnitComboBox.setCurrentText(loadedSettings['Sensor']['Pressure Transducer']['Transducer1']['Units'])

            self.saveResultsNameLineEdit.setText(loadedSettings['Experiment']['Results Save'])
            self.crossSectionLineEdit.setText(loadedSettings['Experiment']['Cross Section'])
            self.crossSectionUnitsComboBox.setCurrentText(loadedSettings['Experiment']['Cross Section Units'])
            self.tissueThicknessLineEdit.setText(loadedSettings['Experiment']['Tissue Thickness'])
            self.tissueThicknessUnitsComboBox.setCurrentText(loadedSettings['Experiment']['Tissue Thickness Units'])
            self.flowSetpointLineEdit.setText(loadedSettings['Experiment']['Flow Setpoint'])
            self.flowSetpointUnitsComboBox.setCurrentText(loadedSettings['Experiment']['Flow Setpoint Units'])
            self.averagingDepthSpinBox.setValue(loadedSettings['Experiment']['Averaging Depth'])
            self.errorBoundsLineEdit.setText(loadedSettings['Experiment']['Flow Error Bounds'])
            if loadedSettings['Experiment']['Flow Error Units'] == '%':
                self.errorBoundsUnitsComboBox.clear()
                self.errorBoundsUnitsComboBox.addItem('%')
                self.errorBoundsUnitsComboBox.addItem(self.flowSetpointUnitsComboBox.currentText())
                self.errorBoundsUnitsComboBox.setCurrentText('%')
            else:
                self.errorBoundsUnitsComboBox.clear()
                self.errorBoundsUnitsComboBox.addItem('%')
                self.errorBoundsUnitsComboBox.addItem(loadedSettings['Experiment']['Flow Error Units'])
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
                    'Pressure Transducer' : {'High Side-Transducer2' : self.transducer2RadioButton.isChecked(),
                                             'High Side-Transducer1' : self.transducer1RadioButton.isChecked(),
                                             'Transducer2' : {'Slope' : self.transducer2SlopeLineEdit.text(),
                                                       'Intercept' : self.transducer2InterceptLineEdit.text(),
                                                       'Units' : self.transducer2UnitComboBox.currentText(),
                                                       'Serial' : self.transducer2SerialLineEdit.text()},
                                             'Transducer1' : {'Slope' : self.transducer1SlopeLineEdit.text(),
                                                       'Intercept' : self.transducer1InterceptLineEdit.text(),
                                                       'Units' : self.transducer1UnitComboBox.currentText(),
                                                       'Serial' : self.transducer1SerialLineEdit.text()}},
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

    def UpdateErrorBounds(self):
        currentUnits = self.errorBoundsUnitsComboBox.currentText()
        self.errorBoundsUnitsComboBox.clear()
        self.errorBoundsUnitsComboBox.addItem('%')
        self.errorBoundsUnitsComboBox.addItem(self.flowSetpointUnitsComboBox.currentText())
        if currentUnits == '%':
            self.errorBoundsUnitsComboBox.setCurrentText('%')
        else:
            self.errorBoundsUnitsComboBox.setCurrentText(self.flowSetpointUnitsComboBox.currentText())
            oldVolumeUnit, oldTimeUnit = currentUnits.split('/')
            newVolumeUnit, newTimeUnit = self.flowSetpointUnitsComboBox.currentText().split('/')
            newErrorBound = float(self.errorBoundsLineEdit.text()) * self.timeUnitsDict[oldTimeUnit] / self.volumeUnitsDict[oldVolumeUnit] * self.volumeUnitsDict[newVolumeUnit] / self.timeUnitsDict[newTimeUnit]
            self.errorBoundsLineEdit.setText(str(int(newErrorBound)))

    def UpdateAxis(self):
        axisScale = {0 : self.AdjustFlow,
                     1 : self.AdjustPressure,
                     2 : self.AdjustCurrent,
                     3 : self.AdjustVoltage}
        axisScale[self.graphsTab.currentIndex()]()

    def AdjustFlow(self):
        self.flowCurve.clear()
        newUnits = self.unitsComboBox.currentText().split('/')
        self.graphUnits[0][1] = (self.volumeUnitsDict[newUnits[0]]) / (self.timeUnitsDict[newUnits[1]])
        self.graphUnits[0][0] = self.unitsComboBox.currentText()
        self.flowGraph.getAxis('left').setLabel('Flow ({})'.format(self.graphUnits[0][0]))
        self.flowGraph.setYRange((-1/6e8)/self.graphUnits[0][1], (8/6e7)/self.graphUnits[0][1])
        self.flowCurve = self.flowGraph.plot()

    def AdjustPressure(self):
        self.pressureCurve.clear()
        self.graphUnits[1][1] = self.pressureUnitsDict[self.unitsComboBox.currentText()]
        self.graphUnits[1][0] = self.unitsComboBox.currentText()
        self.pressureGraph.getAxis('left').setLabel('Pressure ({})'.format(self.graphUnits[1][0]))
        self.pressureGraph.setYRange(-100/self.graphUnits[1][1], 800000/self.graphUnits[1][1])
        self.pressureCurve = self.pressureGraph.plot()

    def AdjustCurrent(self):
        self.currentCurve.clear()
        self.graphUnits[2][1] = self.currentUnitsDict[self.unitsComboBox.currentText()]
        self.graphUnits[2][0] = self.unitsComboBox.currentText()
        self.currentGraph.getAxis('left').setLabel('Current ({})'.format(self.graphUnits[2][0]))
        self.currentGraph.setYRange(-100e-3/self.graphUnits[2][1], 100e-3/self.graphUnits[2][1])
        self.currentCurve = self.currentGraph.plot()

    def AdjustVoltage(self):
        self.voltageCurve.clear()
        self.graphUnits[3][1] = self.voltageUnitsDict[self.unitsComboBox.currentText()]
        self.graphUnits[3][0] = self.unitsComboBox.currentText()
        self.voltageGraph.getAxis('left').setLabel('Voltage ({})'.format(self.graphUnits[3][0]))
        self.voltageGraph.setYRange(-100e-3/self.graphUnits[3][1], 100e-3/self.graphUnits[3][1])
        self.voltageCurve = self.voltageGraph.plot()

    def AdjustErrorBounds(self):
        if self.errorBoundsUnitsComboBox.currentText() == '%':
            self.lowerBound = (100 - float(self.errorBoundsLineEdit.text())) / 100 * float(self.flowSetpointLineEdit.text())
            self.upperBound = (100 + float(self.errorBoundsLineEdit.text())) / 100 * float(self.flowSetpointLineEdit.text())
        self.lowerBound = float(self.flowSetpointLineEdit.text()) - float(self.errorBoundsLineEdit.text())
        self.upperBound = float(self.flowSetpointLineEdit.text()) + float(self.errorBoundsLineEdit.text())

    def AdjustCalibration(self):
        self.pressureCalibrationCurve.clear()
        self.pressurePredictionCurve.clear()
        self.calibrationUnits = self.calibrationUnitsComboBox.currentText()
        self.pressureCalibrationGraph.getAxis('bottom').setLabel('Pressure ({})'.format(self.calibrationUnits))

        if self.calibrationPressures:
            displayPressures = list(np.divide(self.calibrationPressures, self.pressureUnitsDict[self.calibrationUnits]))
            modelPressures = np.array(self.calibrationPressures).reshape(-1,1)
            modelPressures = modelPressures.reshape(-1,1)
            model = LinearRegression().fit(modelPressures, self.calibrationAverages)
            predictedReadings = model.predict(modelPressures) #gives predicted y vals for array x
            self.pressureCalibrationCurve.setData(displayPressures, self.calibrationAverages)
            self.pressurePredictionCurve.setData(displayPressures, predictedReadings)

    def RunStopData(self):
        if self.currentlyRunning:
            self.currentlyRunning = False
            if self.logData:
                self.StartStopLogging()
            self.dataTimer.stop()
            ## Pi-start
            self.sourceMeter.close()
            ## Pi-end
            self.runStopButton.setText('Run')
            self.runStatusLabel.setText('Stopped')
        else:
            gpio.output(self.doneLED, gpio.LOW)
            self.dataTimer.start(self.timerInterval)
            self.timeData = deque([], int(self.lockTime * 1000 / self.timerInterval))
            self.flowData = deque([], int(self.lockTime * 1000 / self.timerInterval))
            self.pressureData = deque([], int(self.lockTime * 1000 / self.timerInterval))
            self.voltageData = deque([], int(self.lockTime * 1000 / self.timerInterval))
            self.currentData = deque([], int(self.lockTime * 1000 / self.timerInterval))
            self.AdjustErrorBounds()
            self.currentlyRunning = True
            if not self.transducer2SlopeLineEdit.text() or not self.transducer1SlopeLineEdit.text():
                raise(ValueError('Both left and right transducers need calibration values'))
            if not self.transducer2InterceptLineEdit.text() or not self.transducer1InterceptLineEdit.text():
                raise(ValueError('Both left and right transducers need calibration values'))

            ## Reset the pressure and flow chips on the I2C buses

            ## Pi-start
            boot_cycle(self.i2c_bus)
            boot_cycle(self.i2c_bus2)
            reset_sensor(self.i2c_bus)
            self.sourceMeter = self.visaResourceManager.open_resource('ASRL/dev/ttyUSB0::INSTR')
            self.sourceMeter.write("*RST")
            sleep(0.5)
            ## Pi-end

            # Set up the pressure sensors

            ## Pi-start
            set_avdd_voltage(self.i2c_bus, '4.5v')
            select_avdd_source(self.i2c_bus, internal_source=True)
            set_conversion_rate(self.i2c_bus, conversion_rate='sps10')
            set_gain(self.i2c_bus, 'x1')
            start_reading_data(self.i2c_bus, start=True)
            set_avdd_voltage(self.i2c_bus2, '4.5v')
            select_avdd_source(self.i2c_bus2, internal_source=True)
            set_conversion_rate(self.i2c_bus2, conversion_rate='sps10')
            set_gain(self.i2c_bus2, 'x1')
            start_reading_data(self.i2c_bus2, start=True)
            ## Pi-end

            # Set up the flow sensor

            ## Pi-start
            self.sensor, self.serialNumber, _, _ = read_product_info(self.i2c_bus)
            self.scaleFactor, self.units, _, _ = read_scale_and_unit(self.i2c_bus)
            set_resolution(self.i2c_bus, bits=self.resolutionSettingSpinBox.value())
            set_read_data(self.i2c_bus)
            ## Pi-end

            # Set up the sourcemeter for voltage and current

            self.runStatusLabel.setText('Running')
            self.runStopButton.setText('Stop')

    def StartStopLogging(self):
        if self.logData:
            self.logData = False
            self.logButton.setText('Start Logging')
        else:
            self.logData = True
            self.logButton.setText('Stop Logging')
            try:
                with open(self.logFileLineEdit.text(), 'x') as fileHeader:
                    fileHeader.write('Time,Flow_Rate(L/s),Pressure(Pa),Current(A),Voltage(V)\n')
            except FileExistsError:
                self.msg = QMessageBox()
                self.msg.setWindowTitle('File Exists!')
                self.msg.setText('The file you are trying to write to already exists!\nInserting BREAK markers and appending new data')
                self.msg.setIcon(QMessageBox.Information)
                self.msg.setStandardButtons(QMessageBox.Ok)
                self.msg.show()
                with open(self.logFileLineEdit.text(), 'a') as fileHeader:
                    fileHeader.write('BREAK,BREAK,BREAK,BREAK,BREAK\n')


    def PressureDifferential(self):
        rawTransducer2Reading = read_load(self.i2c_bus)
        rawTransducer1Reading = read_load(self.i2c_bus2)
        transducer2Reading = float(self.transducer2SlopeLineEdit.text()) * rawTransducer2Reading + float(self.transducer2InterceptLineEdit.text())
        transducer1Reading = float(self.transducer1SlopeLineEdit.text()) * rawTransducer1Reading + float(self.transducer1InterceptLineEdit.text())
        if self.transducer2RadioButton.isChecked():
            return transducer2Reading - transducer1Reading
        else:
            return transducer1Reading - transducer2Reading

    def ReadFlow(self):
        rawFlowReading, _, _ = read_raw_data(self.i2c_bus)
        scaledFlowReading = scale_reading(rawFlowReading, self.scaleFactor)
        return scaledFlowReading * self.volumeUnitsDict['nL'] / self.timeUnitsDict['min']

    def UpdateData(self):
        timepoint = time()
        self.timeData.append(timepoint)
        ## Pi-start
        flowReading = self.ReadFlow()

        pressureDifferentialReading = self.PressureDifferential()

        instrumentResponse = self.sourceMeter.query(':meas:curr:dc?')
        currentReading, voltageReading, _, _, _ = instrumentResponse.split(',')
        currentReading = float(currentReading)
        voltageReading = float(voltageReading)
        ## Pi-end

        ## not-Pi-start
        # flowReading = random.randrange(3000, 3500)*1e-9/60
        # pressureDifferentialReading = random.randrange(100000, 200000)
        # voltageReading = random.randrange(-100, 101)/1e3
        # currentReading = random.randrange(-100, 101)/1e3
        ## not-Pi-end
        if self.logData:
            with open(self.logFileLineEdit.text(), 'a') as data:
                data.write('{},{},{},{},{}\n'.format(timepoint, flowReading, pressureDifferentialReading, currentReading, voltageReading))

        self.flowData.append(flowReading)
        self.pressureData.append(pressureDifferentialReading)
        self.voltageData.append(voltageReading)
        self.currentData.append(currentReading)
        if self.lowerBound < self.flowData[-1] < self.upperBound and len(self.flowData) == self.flowData.maxlen:
            if abs(mean(list(self.flowData)[:100]) - mean(list(self.flowData)[-100:])) < 25:
                if self.popUpDoneCheckBox.isChecked():
                    self.msg = QMessageBox()
                    self.msg.setWindowTitle('Experiment Progress')
                    self.msg.setText('All Done')
                    self.msg.setIcon(QMessageBox.Information)
                    self.msg.setStandardButtons(QMessageBox.Ok)
                    self.msg.show()
                if self.soundDoneCheckBox.isChecked():
                    Popen(['vlc', 'done.mp3'])
                if self.ledDoneCheckBox.isChecked():
                    ## Pi-start
                    gpio.output(self.doneLED, gpio.HIGH)
                    ## Pi-end
                    pass
                self.RunStopData()
                if self.logData:
                    self.StartStopLogging()
                return
        self.UpdateGraphs()

    def UpdateGraphs(self):
        if self.mainTabStack.currentIndex() == 0:
            updateGraph = {0 : self.UpdateFlowGraph,
                           1 : self.UpdatePressureGraph,
                           2 : self.UpdateCurrentGraph,
                           3 : self.UpdateVoltageGraph}
            updateGraph[self.graphsTab.currentIndex()]()

    def UpdateGraphUnits(self):
        self.unitsComboBox.clear()
        newUnitOptions = {0 : self.flowRates,
                          1 : self.pressureUnitsDict.keys(),
                          2 : self.currentUnitsDict.keys(),
                          3 : self.voltageUnitsDict.keys()}
        self.unitsComboBox.addItems(newUnitOptions[self.graphsTab.currentIndex()])
        self.unitsComboBox.setCurrentText(self.graphUnits[self.graphsTab.currentIndex()][0])

    def UpdateFlowGraph(self):
        flowVals = list(self.flowData)[-int(5 * 60 * 1000 / self.timerInterval):]
        flowVals = list(np.divide(flowVals, self.graphUnits[0][1]))
        timeVals = list(self.timeData)[-int(5 * 60 * 1000 / self.timerInterval):]
        timeVals = list(np.subtract(timeVals, timeVals[-1]))
        self.flowCurve.setData(timeVals, flowVals)
        self.currentReadingLCD.display(flowVals[-1])

    def UpdatePressureGraph(self):
        pressureVals = list(self.pressureData)[-int(5 * 60 * 1000 / self.timerInterval):]
        pressureVals = list(np.divide(pressureVals, self.graphUnits[1][1]))
        timeVals = list(self.timeData)[-int(5 * 60 * 1000 / self.timerInterval):]
        timeVals = list(np.subtract(timeVals, timeVals[-1]))
        self.pressureCurve.setData(timeVals, pressureVals)
        self.currentReadingLCD.display(pressureVals[-1])

    def UpdateCurrentGraph(self):
        currentVals = list(self.currentData)[-int(5 * 60 * 1000 / self.timerInterval):]
        currentVals = list(np.divide(currentVals, self.graphUnits[2][1]))
        timeVals = list(self.timeData)[-int(5 * 60 * 1000 / self.timerInterval):]
        timeVals = list(np.subtract(timeVals, timeVals[-1]))
        self.currentCurve.setData(timeVals, currentVals)
        self.currentReadingLCD.display(currentVals[-1])

    def UpdateVoltageGraph(self):
        voltageVals = list(self.voltageData)[-int(5 * 60 * 1000 / self.timerInterval):]
        voltageVals = list(np.divide(voltageVals, self.graphUnits[3][1]))
        timeVals = list(self.timeData)[-int(5 * 60 * 1000 / self.timerInterval):]
        timeVals = list(np.subtract(timeVals, timeVals[-1]))
        self.voltageCurve.setData(timeVals, voltageVals)
        self.currentReadingLCD.display(voltageVals[-1])

    def UpdateLockTime(self):
        if self.timeUnitsDict[self.lockTimeUnitsComboBox.currentText()] * int(self.lockTimeLineEdit.text()) < 5 * 60: # don't allow a lock time shorter than 5 minutes
            if self.lockTimeUnitsComboBox.currentText() == 'min':
                self.lockTimeLineEdit.setText('5')
            else:
                self.lockTimeLineEdit.setText('300')
        else:
            self.lockTime = self.timeUnitsDict[self.lockTimeUnitsComboBox.currentText()] * int(self.lockTimeLineEdit.text())

    def AddCalibrationPoint(self):
        ## Pi-start
        if self.transducer1CalibrationRadioButton.isChecked():
            bus = self.i2c_bus2
        else:
            bus = self.i2c_bus

        readings = []
        boot_cycle(bus)
        sleep(0.5)
        set_avdd_voltage(bus, '4.5v')
        select_avdd_source(bus, internal_source=True)
        set_conversion_rate(bus, conversion_rate='sps10')
        set_gain(bus, 'x1')
        start_reading_data(bus, start=True)
        sleep(0.1)
        gpio.output(self.doneLED, gpio.LOW)
        for x in range(100):
            while not checkDataReady(bus):
                if gpio.input(self.errorLED):
                    gpio.output(self.errorLED, gpio.LOW)
                else:
                    gpio.output(self.errorLED, gpio.HIGH)
                sleep(0.025)
            readings.append(read_load(bus))
        gpio.output(self.errorLED, gpio.LOW)
        gpio.output(self.doneLED, gpio.HIGH)
        ## Pi-end
        ## not-Pi-start
        # readings = []
        # for x in range(100):
            # readings.append(random.randrange(3000, 3500))
        ## not-Pi-end
        self.calibrationAverages.append(mean(readings))
        self.calibrationLCD.display(self.calibrationAverages[-1])
        self.calibrationPressures.append(float(self.addCalibrationDataLineEdit.text()) * self.pressureUnitsDict[self.calibrationUnits])
        displayPressures = list(np.divide(self.calibrationPressures, self.pressureUnitsDict[self.calibrationUnits]))
        modelPressures = np.array(self.calibrationPressures).reshape(-1,1)
        modelPressures = modelPressures.reshape(-1,1)
        model = LinearRegression().fit(modelPressures, self.calibrationAverages)
        self.r_sq = model.score(modelPressures, self.calibrationAverages)
        predictedReadings = model.predict(modelPressures) #gives predicted y vals for array x
        intercept = model.intercept_
        slope = float(model.coef_[0]) #Get the slope value as a number, sklearn presents it as a 1d array with one element
        self.pressureCalibrationCurve.setData(displayPressures, self.calibrationAverages)
        self.pressurePredictionCurve.setData(displayPressures, predictedReadings)
        self.pressureCalibrationGraphText.setText('Slope: {}\nIntecept: {}\nR^2: {}'.format(slope, intercept, self.r_sq))
        self.pressureCalibrationGraphText.setPos(0,0)
        print('Slope: {}\nIntecept: {}\nR^2: {}'.format(slope, intercept, self.r_sq))
        if slope == float(0):
            self.readingToPressureSlope = 0.0
            self.readingToPressureIntercept = 0.0
            self.r_rq = 0.0
        else:
            self.readingToPressureSlope = 1/slope
            self.readingToPressureIntercept = -intercept/slope

    def SaveCalibrationData(self):
        if not self.saveCalibrationLineEdit.text():
            saveFileLocation = QFileDialog.getSaveFileName(self)
        else:
            saveFileLocation = self.saveCalibrationLineEdit.text()
        with zipfile.ZipFile(saveFileLocation, mode='w') as calFile:
            with calFile.open(b'sensor_parameters.yaml', 'w') as parameters:
                saveData = {'slope' : self.readingToPressureSlope,
                            'intercept' : self.readingToPressureIntercept,
                            'serial'    : self.serialNumberLineEdit.text(),
                            'r_sq'      : self.r_sq}
                yaml.dump(saveData, parameters)
            with calFile.open(b'calibration_points.csv', 'w', newline='') as rawPoints:
                calPointsWriter = csv.writer(rawPoints, delimiter=',')
                for reading, pressure in zip(self.calibrationAverages, self.calibrationPressures):
                    calPointsWriter.writerow([reading, pressure])


def main():
    app = QApplication(sys.argv)
    form = StreamingPotentialApp()
    form.show()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()


