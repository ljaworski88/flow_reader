# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'flow_reader.ui'
#
# Created by: PyQt5 UI code generator 5.15.0
#
# WARNING: Any manual changes made to this file will be lost when pyuic5 is
# run again.  Do not edit this file unless you know what you are doing.


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(625, 776)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.gridLayout_5 = QtWidgets.QGridLayout(self.centralwidget)
        self.gridLayout_5.setObjectName("gridLayout_5")
        self.mainTabStack = QtWidgets.QTabWidget(self.centralwidget)
        self.mainTabStack.setObjectName("mainTabStack")
        self.graphicsTab = QtWidgets.QWidget()
        self.graphicsTab.setToolTipDuration(5)
        self.graphicsTab.setObjectName("graphicsTab")
        self.verticalLayout_3 = QtWidgets.QVBoxLayout(self.graphicsTab)
        self.verticalLayout_3.setObjectName("verticalLayout_3")
        self.graphTabVerticalLayout = QtWidgets.QVBoxLayout()
        self.graphTabVerticalLayout.setObjectName("graphTabVerticalLayout")
        self.graphsTab = QtWidgets.QTabWidget(self.graphicsTab)
        self.graphsTab.setTabPosition(QtWidgets.QTabWidget.North)
        self.graphsTab.setObjectName("graphsTab")
        self.flowGraphTab = QtWidgets.QWidget()
        self.flowGraphTab.setObjectName("flowGraphTab")
        self.gridLayout_4 = QtWidgets.QGridLayout(self.flowGraphTab)
        self.gridLayout_4.setObjectName("gridLayout_4")
        self.flowGraph = PlotWidget(self.flowGraphTab)
        self.flowGraph.setObjectName("flowGraph")
        self.gridLayout_4.addWidget(self.flowGraph, 0, 0, 1, 1)
        self.graphsTab.addTab(self.flowGraphTab, "")
        self.pressureGraphTab = QtWidgets.QWidget()
        self.pressureGraphTab.setObjectName("pressureGraphTab")
        self.gridLayout_3 = QtWidgets.QGridLayout(self.pressureGraphTab)
        self.gridLayout_3.setObjectName("gridLayout_3")
        self.pressureGraph = PlotWidget(self.pressureGraphTab)
        self.pressureGraph.setObjectName("pressureGraph")
        self.gridLayout_3.addWidget(self.pressureGraph, 0, 0, 1, 1)
        self.graphsTab.addTab(self.pressureGraphTab, "")
        self.currentGraphTab = QtWidgets.QWidget()
        self.currentGraphTab.setObjectName("currentGraphTab")
        self.gridLayout = QtWidgets.QGridLayout(self.currentGraphTab)
        self.gridLayout.setObjectName("gridLayout")
        self.currentGraph = PlotWidget(self.currentGraphTab)
        self.currentGraph.setObjectName("currentGraph")
        self.gridLayout.addWidget(self.currentGraph, 0, 0, 1, 1)
        self.graphsTab.addTab(self.currentGraphTab, "")
        self.voltageGraphTab = QtWidgets.QWidget()
        self.voltageGraphTab.setObjectName("voltageGraphTab")
        self.gridLayout_2 = QtWidgets.QGridLayout(self.voltageGraphTab)
        self.gridLayout_2.setObjectName("gridLayout_2")
        self.voltageGraph = PlotWidget(self.voltageGraphTab)
        self.voltageGraph.setObjectName("voltageGraph")
        self.gridLayout_2.addWidget(self.voltageGraph, 0, 0, 1, 1)
        self.graphsTab.addTab(self.voltageGraphTab, "")
        self.graphTabVerticalLayout.addWidget(self.graphsTab)
        self.loggingLayout = QtWidgets.QHBoxLayout()
        self.loggingLayout.setObjectName("loggingLayout")
        self.logFileLineEdit = QtWidgets.QLineEdit(self.graphicsTab)
        self.logFileLineEdit.setObjectName("logFileLineEdit")
        self.loggingLayout.addWidget(self.logFileLineEdit)
        self.logButton = QtWidgets.QPushButton(self.graphicsTab)
        self.logButton.setObjectName("logButton")
        self.loggingLayout.addWidget(self.logButton)
        self.graphTabVerticalLayout.addLayout(self.loggingLayout)
        self.infoTickerHorizontalLayout = QtWidgets.QHBoxLayout()
        self.infoTickerHorizontalLayout.setObjectName("infoTickerHorizontalLayout")
        self.descriptionsLayout = QtWidgets.QVBoxLayout()
        self.descriptionsLayout.setObjectName("descriptionsLayout")
        self.sensorInfo = QtWidgets.QLabel(self.graphicsTab)
        self.sensorInfo.setObjectName("sensorInfo")
        self.descriptionsLayout.addWidget(self.sensorInfo)
        self.infoTickerHorizontalLayout.addLayout(self.descriptionsLayout)
        self.unitsLayout = QtWidgets.QVBoxLayout()
        self.unitsLayout.setObjectName("unitsLayout")
        self.currentReadingLCD = QtWidgets.QLCDNumber(self.graphicsTab)
        self.currentReadingLCD.setObjectName("currentReadingLCD")
        self.unitsLayout.addWidget(self.currentReadingLCD)
        self.unitsComboBox = QtWidgets.QComboBox(self.graphicsTab)
        self.unitsComboBox.setObjectName("unitsComboBox")
        self.unitsLayout.addWidget(self.unitsComboBox)
        self.infoTickerHorizontalLayout.addLayout(self.unitsLayout)
        self.runCalculateVerticalLayout = QtWidgets.QVBoxLayout()
        self.runCalculateVerticalLayout.setObjectName("runCalculateVerticalLayout")
        self.runStopButton = QtWidgets.QPushButton(self.graphicsTab)
        self.runStopButton.setObjectName("runStopButton")
        self.runCalculateVerticalLayout.addWidget(self.runStopButton)
        self.calculateResultButton = QtWidgets.QPushButton(self.graphicsTab)
        self.calculateResultButton.setObjectName("calculateResultButton")
        self.runCalculateVerticalLayout.addWidget(self.calculateResultButton)
        self.infoTickerHorizontalLayout.addLayout(self.runCalculateVerticalLayout)
        self.graphTabVerticalLayout.addLayout(self.infoTickerHorizontalLayout)
        self.verticalLayout_3.addLayout(self.graphTabVerticalLayout)
        self.mainTabStack.addTab(self.graphicsTab, "")
        self.sensorSettingsTab = QtWidgets.QWidget()
        self.sensorSettingsTab.setObjectName("sensorSettingsTab")
        self.verticalLayout_2 = QtWidgets.QVBoxLayout(self.sensorSettingsTab)
        self.verticalLayout_2.setObjectName("verticalLayout_2")
        self.sensorSettingsVerticalLayout = QtWidgets.QVBoxLayout()
        self.sensorSettingsVerticalLayout.setObjectName("sensorSettingsVerticalLayout")
        self.sensorSettingsLine = QtWidgets.QFrame(self.sensorSettingsTab)
        self.sensorSettingsLine.setFrameShape(QtWidgets.QFrame.HLine)
        self.sensorSettingsLine.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.sensorSettingsLine.setObjectName("sensorSettingsLine")
        self.sensorSettingsVerticalLayout.addWidget(self.sensorSettingsLine)
        self.flowMeterSettingLabel = QtWidgets.QLabel(self.sensorSettingsTab)
        self.flowMeterSettingLabel.setAutoFillBackground(False)
        self.flowMeterSettingLabel.setStyleSheet("background-color: rgb(0, 0, 0);\n"
"color: rgb(238, 238, 236);")
        self.flowMeterSettingLabel.setAlignment(QtCore.Qt.AlignCenter)
        self.flowMeterSettingLabel.setObjectName("flowMeterSettingLabel")
        self.sensorSettingsVerticalLayout.addWidget(self.flowMeterSettingLabel)
        self.flowMeterSettingsFormLayout = QtWidgets.QFormLayout()
        self.flowMeterSettingsFormLayout.setObjectName("flowMeterSettingsFormLayout")
        self.resolutionSettingHorizontalLayout = QtWidgets.QHBoxLayout()
        self.resolutionSettingHorizontalLayout.setObjectName("resolutionSettingHorizontalLayout")
        self.resolutionSettingSpinBox = QtWidgets.QSpinBox(self.sensorSettingsTab)
        self.resolutionSettingSpinBox.setMinimum(9)
        self.resolutionSettingSpinBox.setMaximum(16)
        self.resolutionSettingSpinBox.setProperty("value", 16)
        self.resolutionSettingSpinBox.setObjectName("resolutionSettingSpinBox")
        self.resolutionSettingHorizontalLayout.addWidget(self.resolutionSettingSpinBox)
        self.resolutionSettingLabel = QtWidgets.QLabel(self.sensorSettingsTab)
        self.resolutionSettingLabel.setObjectName("resolutionSettingLabel")
        self.resolutionSettingHorizontalLayout.addWidget(self.resolutionSettingLabel)
        self.flowMeterSettingsFormLayout.setLayout(0, QtWidgets.QFormLayout.LabelRole, self.resolutionSettingHorizontalLayout)
        self.flowMeterResolutionSettingLabel = QtWidgets.QLabel(self.sensorSettingsTab)
        self.flowMeterResolutionSettingLabel.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.flowMeterResolutionSettingLabel.setObjectName("flowMeterResolutionSettingLabel")
        self.flowMeterSettingsFormLayout.setWidget(0, QtWidgets.QFormLayout.FieldRole, self.flowMeterResolutionSettingLabel)
        self.sensorSettingsVerticalLayout.addLayout(self.flowMeterSettingsFormLayout)
        self.pressureTransducerSettingsLabel = QtWidgets.QLabel(self.sensorSettingsTab)
        self.pressureTransducerSettingsLabel.setStyleSheet("background-color: rgb(0, 0, 0);\n"
"color: rgb(238, 238, 236);")
        self.pressureTransducerSettingsLabel.setAlignment(QtCore.Qt.AlignCenter)
        self.pressureTransducerSettingsLabel.setObjectName("pressureTransducerSettingsLabel")
        self.sensorSettingsVerticalLayout.addWidget(self.pressureTransducerSettingsLabel)
        self.pressureTransducerSettingsFormLayout = QtWidgets.QFormLayout()
        self.pressureTransducerSettingsFormLayout.setObjectName("pressureTransducerSettingsFormLayout")
        self.transducerHighSideButtonHorizontalLayout = QtWidgets.QHBoxLayout()
        self.transducerHighSideButtonHorizontalLayout.setObjectName("transducerHighSideButtonHorizontalLayout")
        self.leftTransducerRadioButton = QtWidgets.QRadioButton(self.sensorSettingsTab)
        self.leftTransducerRadioButton.setToolTipDuration(5)
        self.leftTransducerRadioButton.setChecked(True)
        self.leftTransducerRadioButton.setObjectName("leftTransducerRadioButton")
        self.transducerHighSideButtonHorizontalLayout.addWidget(self.leftTransducerRadioButton)
        self.rightTransducerRadioButton = QtWidgets.QRadioButton(self.sensorSettingsTab)
        self.rightTransducerRadioButton.setToolTipDuration(5)
        self.rightTransducerRadioButton.setObjectName("rightTransducerRadioButton")
        self.transducerHighSideButtonHorizontalLayout.addWidget(self.rightTransducerRadioButton)
        self.pressureTransducerSettingsFormLayout.setLayout(0, QtWidgets.QFormLayout.LabelRole, self.transducerHighSideButtonHorizontalLayout)
        self.highPressureSideSelectionLabel = QtWidgets.QLabel(self.sensorSettingsTab)
        self.highPressureSideSelectionLabel.setLayoutDirection(QtCore.Qt.RightToLeft)
        self.highPressureSideSelectionLabel.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.highPressureSideSelectionLabel.setObjectName("highPressureSideSelectionLabel")
        self.pressureTransducerSettingsFormLayout.setWidget(0, QtWidgets.QFormLayout.FieldRole, self.highPressureSideSelectionLabel)
        self.sensorSettingsVerticalLayout.addLayout(self.pressureTransducerSettingsFormLayout)
        self.verticalLayout_2.addLayout(self.sensorSettingsVerticalLayout)
        self.transducerSettingsTab = QtWidgets.QTabWidget(self.sensorSettingsTab)
        self.transducerSettingsTab.setObjectName("transducerSettingsTab")
        self.leftTransducerSettingsTab = QtWidgets.QWidget()
        self.leftTransducerSettingsTab.setObjectName("leftTransducerSettingsTab")
        self.verticalLayout = QtWidgets.QVBoxLayout(self.leftTransducerSettingsTab)
        self.verticalLayout.setObjectName("verticalLayout")
        self.leftTransducerSettingsVeritcalLayout = QtWidgets.QVBoxLayout()
        self.leftTransducerSettingsVeritcalLayout.setObjectName("leftTransducerSettingsVeritcalLayout")
        self.leftTransducerCalibrationHorizontalLayout = QtWidgets.QHBoxLayout()
        self.leftTransducerCalibrationHorizontalLayout.setObjectName("leftTransducerCalibrationHorizontalLayout")
        self.leftTransducerCalibrationFormLayout = QtWidgets.QFormLayout()
        self.leftTransducerCalibrationFormLayout.setObjectName("leftTransducerCalibrationFormLayout")
        self.leftTransducerSlopeLineEdit = QtWidgets.QLineEdit(self.leftTransducerSettingsTab)
        self.leftTransducerSlopeLineEdit.setToolTipDuration(5)
        self.leftTransducerSlopeLineEdit.setObjectName("leftTransducerSlopeLineEdit")
        self.leftTransducerCalibrationFormLayout.setWidget(0, QtWidgets.QFormLayout.LabelRole, self.leftTransducerSlopeLineEdit)
        self.leftTransducerSlopeLabel = QtWidgets.QLabel(self.leftTransducerSettingsTab)
        self.leftTransducerSlopeLabel.setObjectName("leftTransducerSlopeLabel")
        self.leftTransducerCalibrationFormLayout.setWidget(0, QtWidgets.QFormLayout.FieldRole, self.leftTransducerSlopeLabel)
        self.leftTransducerInterceptLineEdit = QtWidgets.QLineEdit(self.leftTransducerSettingsTab)
        self.leftTransducerInterceptLineEdit.setObjectName("leftTransducerInterceptLineEdit")
        self.leftTransducerCalibrationFormLayout.setWidget(1, QtWidgets.QFormLayout.LabelRole, self.leftTransducerInterceptLineEdit)
        self.leftTransducerInerceptLabel = QtWidgets.QLabel(self.leftTransducerSettingsTab)
        self.leftTransducerInerceptLabel.setObjectName("leftTransducerInerceptLabel")
        self.leftTransducerCalibrationFormLayout.setWidget(1, QtWidgets.QFormLayout.FieldRole, self.leftTransducerInerceptLabel)
        self.leftTransducerPressureUnitsLineEdit = QtWidgets.QLineEdit(self.leftTransducerSettingsTab)
        self.leftTransducerPressureUnitsLineEdit.setObjectName("leftTransducerPressureUnitsLineEdit")
        self.leftTransducerCalibrationFormLayout.setWidget(2, QtWidgets.QFormLayout.LabelRole, self.leftTransducerPressureUnitsLineEdit)
        self.leftTransducerPressureUnitsLabel = QtWidgets.QLabel(self.leftTransducerSettingsTab)
        self.leftTransducerPressureUnitsLabel.setObjectName("leftTransducerPressureUnitsLabel")
        self.leftTransducerCalibrationFormLayout.setWidget(2, QtWidgets.QFormLayout.FieldRole, self.leftTransducerPressureUnitsLabel)
        self.leftTransducerCalibrationHorizontalLayout.addLayout(self.leftTransducerCalibrationFormLayout)
        self.leftTransducerIDFormLayout = QtWidgets.QFormLayout()
        self.leftTransducerIDFormLayout.setObjectName("leftTransducerIDFormLayout")
        self.leftTransducerSerialLineEdit = QtWidgets.QLineEdit(self.leftTransducerSettingsTab)
        self.leftTransducerSerialLineEdit.setToolTipDuration(5)
        self.leftTransducerSerialLineEdit.setObjectName("leftTransducerSerialLineEdit")
        self.leftTransducerIDFormLayout.setWidget(0, QtWidgets.QFormLayout.LabelRole, self.leftTransducerSerialLineEdit)
        self.leftTransducerSerialLabel = QtWidgets.QLabel(self.leftTransducerSettingsTab)
        self.leftTransducerSerialLabel.setObjectName("leftTransducerSerialLabel")
        self.leftTransducerIDFormLayout.setWidget(0, QtWidgets.QFormLayout.FieldRole, self.leftTransducerSerialLabel)
        self.leftTransducerTypeLineEdit = QtWidgets.QLineEdit(self.leftTransducerSettingsTab)
        self.leftTransducerTypeLineEdit.setObjectName("leftTransducerTypeLineEdit")
        self.leftTransducerIDFormLayout.setWidget(1, QtWidgets.QFormLayout.LabelRole, self.leftTransducerTypeLineEdit)
        self.leftTransducerTypeLabel = QtWidgets.QLabel(self.leftTransducerSettingsTab)
        self.leftTransducerTypeLabel.setObjectName("leftTransducerTypeLabel")
        self.leftTransducerIDFormLayout.setWidget(1, QtWidgets.QFormLayout.FieldRole, self.leftTransducerTypeLabel)
        self.leftTransducerCalibrationHorizontalLayout.addLayout(self.leftTransducerIDFormLayout)
        self.leftTransducerSettingsVeritcalLayout.addLayout(self.leftTransducerCalibrationHorizontalLayout)
        self.loadLeftTransducerSettingsButton = QtWidgets.QPushButton(self.leftTransducerSettingsTab)
        self.loadLeftTransducerSettingsButton.setObjectName("loadLeftTransducerSettingsButton")
        self.leftTransducerSettingsVeritcalLayout.addWidget(self.loadLeftTransducerSettingsButton)
        self.verticalLayout.addLayout(self.leftTransducerSettingsVeritcalLayout)
        self.transducerSettingsTab.addTab(self.leftTransducerSettingsTab, "")
        self.rightTransducerSettingsTab = QtWidgets.QWidget()
        self.rightTransducerSettingsTab.setObjectName("rightTransducerSettingsTab")
        self.horizontalLayout_2 = QtWidgets.QHBoxLayout(self.rightTransducerSettingsTab)
        self.horizontalLayout_2.setObjectName("horizontalLayout_2")
        self.rightTransducerSettingsVerticalLayout = QtWidgets.QVBoxLayout()
        self.rightTransducerSettingsVerticalLayout.setObjectName("rightTransducerSettingsVerticalLayout")
        self.rightTransducerCalibrationHorizontalLayout = QtWidgets.QHBoxLayout()
        self.rightTransducerCalibrationHorizontalLayout.setObjectName("rightTransducerCalibrationHorizontalLayout")
        self.rightTransducerCalibrationFormLayout = QtWidgets.QFormLayout()
        self.rightTransducerCalibrationFormLayout.setObjectName("rightTransducerCalibrationFormLayout")
        self.rightTransducerSlopeLineEdit = QtWidgets.QLineEdit(self.rightTransducerSettingsTab)
        self.rightTransducerSlopeLineEdit.setToolTipDuration(3)
        self.rightTransducerSlopeLineEdit.setObjectName("rightTransducerSlopeLineEdit")
        self.rightTransducerCalibrationFormLayout.setWidget(0, QtWidgets.QFormLayout.LabelRole, self.rightTransducerSlopeLineEdit)
        self.rightTransducerSlopeLabel = QtWidgets.QLabel(self.rightTransducerSettingsTab)
        self.rightTransducerSlopeLabel.setObjectName("rightTransducerSlopeLabel")
        self.rightTransducerCalibrationFormLayout.setWidget(0, QtWidgets.QFormLayout.FieldRole, self.rightTransducerSlopeLabel)
        self.rightTransducerInterceptLineEdit = QtWidgets.QLineEdit(self.rightTransducerSettingsTab)
        self.rightTransducerInterceptLineEdit.setToolTipDuration(3)
        self.rightTransducerInterceptLineEdit.setObjectName("rightTransducerInterceptLineEdit")
        self.rightTransducerCalibrationFormLayout.setWidget(1, QtWidgets.QFormLayout.LabelRole, self.rightTransducerInterceptLineEdit)
        self.rightTransducerInerceptLabel = QtWidgets.QLabel(self.rightTransducerSettingsTab)
        self.rightTransducerInerceptLabel.setObjectName("rightTransducerInerceptLabel")
        self.rightTransducerCalibrationFormLayout.setWidget(1, QtWidgets.QFormLayout.FieldRole, self.rightTransducerInerceptLabel)
        self.rightTransducerPressureUnitsLineEdit = QtWidgets.QLineEdit(self.rightTransducerSettingsTab)
        self.rightTransducerPressureUnitsLineEdit.setObjectName("rightTransducerPressureUnitsLineEdit")
        self.rightTransducerCalibrationFormLayout.setWidget(2, QtWidgets.QFormLayout.LabelRole, self.rightTransducerPressureUnitsLineEdit)
        self.rightTransducerPressureUnitsLabel = QtWidgets.QLabel(self.rightTransducerSettingsTab)
        self.rightTransducerPressureUnitsLabel.setObjectName("rightTransducerPressureUnitsLabel")
        self.rightTransducerCalibrationFormLayout.setWidget(2, QtWidgets.QFormLayout.FieldRole, self.rightTransducerPressureUnitsLabel)
        self.rightTransducerCalibrationHorizontalLayout.addLayout(self.rightTransducerCalibrationFormLayout)
        self.rightTransducerIDFormLayout = QtWidgets.QFormLayout()
        self.rightTransducerIDFormLayout.setObjectName("rightTransducerIDFormLayout")
        self.rightTransducerSerialLineEdit = QtWidgets.QLineEdit(self.rightTransducerSettingsTab)
        self.rightTransducerSerialLineEdit.setToolTipDuration(3)
        self.rightTransducerSerialLineEdit.setObjectName("rightTransducerSerialLineEdit")
        self.rightTransducerIDFormLayout.setWidget(0, QtWidgets.QFormLayout.LabelRole, self.rightTransducerSerialLineEdit)
        self.rightTransducerSerialLabel = QtWidgets.QLabel(self.rightTransducerSettingsTab)
        self.rightTransducerSerialLabel.setObjectName("rightTransducerSerialLabel")
        self.rightTransducerIDFormLayout.setWidget(0, QtWidgets.QFormLayout.FieldRole, self.rightTransducerSerialLabel)
        self.rightTransducerTypeLineEdit = QtWidgets.QLineEdit(self.rightTransducerSettingsTab)
        self.rightTransducerTypeLineEdit.setObjectName("rightTransducerTypeLineEdit")
        self.rightTransducerIDFormLayout.setWidget(1, QtWidgets.QFormLayout.LabelRole, self.rightTransducerTypeLineEdit)
        self.rightTransducerTypeLabel = QtWidgets.QLabel(self.rightTransducerSettingsTab)
        self.rightTransducerTypeLabel.setObjectName("rightTransducerTypeLabel")
        self.rightTransducerIDFormLayout.setWidget(1, QtWidgets.QFormLayout.FieldRole, self.rightTransducerTypeLabel)
        self.rightTransducerCalibrationHorizontalLayout.addLayout(self.rightTransducerIDFormLayout)
        self.rightTransducerSettingsVerticalLayout.addLayout(self.rightTransducerCalibrationHorizontalLayout)
        self.loadRightTransducerSettingsButton = QtWidgets.QPushButton(self.rightTransducerSettingsTab)
        self.loadRightTransducerSettingsButton.setObjectName("loadRightTransducerSettingsButton")
        self.rightTransducerSettingsVerticalLayout.addWidget(self.loadRightTransducerSettingsButton)
        self.horizontalLayout_2.addLayout(self.rightTransducerSettingsVerticalLayout)
        self.transducerSettingsTab.addTab(self.rightTransducerSettingsTab, "")
        self.verticalLayout_2.addWidget(self.transducerSettingsTab)
        self.loadSettingsHorizontalLayout = QtWidgets.QHBoxLayout()
        self.loadSettingsHorizontalLayout.setObjectName("loadSettingsHorizontalLayout")
        self.loadFlowReaderSettingsLineEdit = QtWidgets.QLineEdit(self.sensorSettingsTab)
        self.loadFlowReaderSettingsLineEdit.setObjectName("loadFlowReaderSettingsLineEdit")
        self.loadSettingsHorizontalLayout.addWidget(self.loadFlowReaderSettingsLineEdit)
        self.loadFlowReaderSettingsButton = QtWidgets.QPushButton(self.sensorSettingsTab)
        self.loadFlowReaderSettingsButton.setObjectName("loadFlowReaderSettingsButton")
        self.loadSettingsHorizontalLayout.addWidget(self.loadFlowReaderSettingsButton)
        self.verticalLayout_2.addLayout(self.loadSettingsHorizontalLayout)
        self.saveSensorSettingsButton = QtWidgets.QPushButton(self.sensorSettingsTab)
        self.saveSensorSettingsButton.setObjectName("saveSensorSettingsButton")
        self.verticalLayout_2.addWidget(self.saveSensorSettingsButton)
        self.mainTabStack.addTab(self.sensorSettingsTab, "")
        self.experimentSettingsTab = QtWidgets.QWidget()
        self.experimentSettingsTab.setObjectName("experimentSettingsTab")
        self.verticalLayout_12 = QtWidgets.QVBoxLayout(self.experimentSettingsTab)
        self.verticalLayout_12.setObjectName("verticalLayout_12")
        self.experimentsSettingsVerticalLayout = QtWidgets.QVBoxLayout()
        self.experimentsSettingsVerticalLayout.setObjectName("experimentsSettingsVerticalLayout")
        self.saveResultsHorizontalLayout = QtWidgets.QHBoxLayout()
        self.saveResultsHorizontalLayout.setObjectName("saveResultsHorizontalLayout")
        self.saveResultsNameLineEdit = QtWidgets.QLineEdit(self.experimentSettingsTab)
        self.saveResultsNameLineEdit.setToolTipDuration(3)
        self.saveResultsNameLineEdit.setObjectName("saveResultsNameLineEdit")
        self.saveResultsHorizontalLayout.addWidget(self.saveResultsNameLineEdit)
        self.saveLocationPushButton = QtWidgets.QPushButton(self.experimentSettingsTab)
        self.saveLocationPushButton.setObjectName("saveLocationPushButton")
        self.saveResultsHorizontalLayout.addWidget(self.saveLocationPushButton)
        self.experimentsSettingsVerticalLayout.addLayout(self.saveResultsHorizontalLayout)
        self.experimentSettingsFormLayout = QtWidgets.QFormLayout()
        self.experimentSettingsFormLayout.setObjectName("experimentSettingsFormLayout")
        self.crossSectionVerticalLayout = QtWidgets.QVBoxLayout()
        self.crossSectionVerticalLayout.setObjectName("crossSectionVerticalLayout")
        self.crossSectionLineEdit = QtWidgets.QLineEdit(self.experimentSettingsTab)
        self.crossSectionLineEdit.setObjectName("crossSectionLineEdit")
        self.crossSectionVerticalLayout.addWidget(self.crossSectionLineEdit)
        self.crossSectionUnitsComboBox = QtWidgets.QComboBox(self.experimentSettingsTab)
        self.crossSectionUnitsComboBox.setObjectName("crossSectionUnitsComboBox")
        self.crossSectionVerticalLayout.addWidget(self.crossSectionUnitsComboBox)
        self.experimentSettingsFormLayout.setLayout(0, QtWidgets.QFormLayout.LabelRole, self.crossSectionVerticalLayout)
        self.crossSectionLabel = QtWidgets.QLabel(self.experimentSettingsTab)
        self.crossSectionLabel.setObjectName("crossSectionLabel")
        self.experimentSettingsFormLayout.setWidget(0, QtWidgets.QFormLayout.FieldRole, self.crossSectionLabel)
        self.tissueThicknessLabel = QtWidgets.QLabel(self.experimentSettingsTab)
        self.tissueThicknessLabel.setObjectName("tissueThicknessLabel")
        self.experimentSettingsFormLayout.setWidget(1, QtWidgets.QFormLayout.FieldRole, self.tissueThicknessLabel)
        self.flowSetpointVerticalLayout = QtWidgets.QVBoxLayout()
        self.flowSetpointVerticalLayout.setObjectName("flowSetpointVerticalLayout")
        self.flowSetpointLineEdit = QtWidgets.QLineEdit(self.experimentSettingsTab)
        self.flowSetpointLineEdit.setObjectName("flowSetpointLineEdit")
        self.flowSetpointVerticalLayout.addWidget(self.flowSetpointLineEdit)
        self.flowSetpointUnitsComboBox = QtWidgets.QComboBox(self.experimentSettingsTab)
        self.flowSetpointUnitsComboBox.setObjectName("flowSetpointUnitsComboBox")
        self.flowSetpointVerticalLayout.addWidget(self.flowSetpointUnitsComboBox)
        self.experimentSettingsFormLayout.setLayout(2, QtWidgets.QFormLayout.LabelRole, self.flowSetpointVerticalLayout)
        self.flowSetpointLabel = QtWidgets.QLabel(self.experimentSettingsTab)
        self.flowSetpointLabel.setObjectName("flowSetpointLabel")
        self.experimentSettingsFormLayout.setWidget(2, QtWidgets.QFormLayout.FieldRole, self.flowSetpointLabel)
        self.averagingDepthSpinBox = QtWidgets.QSpinBox(self.experimentSettingsTab)
        self.averagingDepthSpinBox.setMinimum(1)
        self.averagingDepthSpinBox.setMaximum(100)
        self.averagingDepthSpinBox.setObjectName("averagingDepthSpinBox")
        self.experimentSettingsFormLayout.setWidget(3, QtWidgets.QFormLayout.LabelRole, self.averagingDepthSpinBox)
        self.averagingVerticalLayout = QtWidgets.QVBoxLayout()
        self.averagingVerticalLayout.setObjectName("averagingVerticalLayout")
        self.flowRateLabel = QtWidgets.QLabel(self.experimentSettingsTab)
        self.flowRateLabel.setObjectName("flowRateLabel")
        self.averagingVerticalLayout.addWidget(self.flowRateLabel)
        self.averagingDepthLabel = QtWidgets.QLabel(self.experimentSettingsTab)
        self.averagingDepthLabel.setObjectName("averagingDepthLabel")
        self.averagingVerticalLayout.addWidget(self.averagingDepthLabel)
        self.experimentSettingsFormLayout.setLayout(3, QtWidgets.QFormLayout.FieldRole, self.averagingVerticalLayout)
        self.errorBoundsVerticalLayout = QtWidgets.QVBoxLayout()
        self.errorBoundsVerticalLayout.setObjectName("errorBoundsVerticalLayout")
        self.errorBoundsLineEdit = QtWidgets.QLineEdit(self.experimentSettingsTab)
        self.errorBoundsLineEdit.setObjectName("errorBoundsLineEdit")
        self.errorBoundsVerticalLayout.addWidget(self.errorBoundsLineEdit)
        self.errorBoundsUnitsComboBox = QtWidgets.QComboBox(self.experimentSettingsTab)
        self.errorBoundsUnitsComboBox.setObjectName("errorBoundsUnitsComboBox")
        self.errorBoundsVerticalLayout.addWidget(self.errorBoundsUnitsComboBox)
        self.experimentSettingsFormLayout.setLayout(4, QtWidgets.QFormLayout.LabelRole, self.errorBoundsVerticalLayout)
        self.errorBoundsLabel = QtWidgets.QLabel(self.experimentSettingsTab)
        self.errorBoundsLabel.setObjectName("errorBoundsLabel")
        self.experimentSettingsFormLayout.setWidget(4, QtWidgets.QFormLayout.FieldRole, self.errorBoundsLabel)
        self.lockTimeVerticalLayout = QtWidgets.QVBoxLayout()
        self.lockTimeVerticalLayout.setObjectName("lockTimeVerticalLayout")
        self.lockTimeLineEdit = QtWidgets.QLineEdit(self.experimentSettingsTab)
        self.lockTimeLineEdit.setObjectName("lockTimeLineEdit")
        self.lockTimeVerticalLayout.addWidget(self.lockTimeLineEdit)
        self.lockTimeUnitsComboBox = QtWidgets.QComboBox(self.experimentSettingsTab)
        self.lockTimeUnitsComboBox.setObjectName("lockTimeUnitsComboBox")
        self.lockTimeVerticalLayout.addWidget(self.lockTimeUnitsComboBox)
        self.experimentSettingsFormLayout.setLayout(5, QtWidgets.QFormLayout.LabelRole, self.lockTimeVerticalLayout)
        self.lockTimeLabel = QtWidgets.QLabel(self.experimentSettingsTab)
        self.lockTimeLabel.setObjectName("lockTimeLabel")
        self.experimentSettingsFormLayout.setWidget(5, QtWidgets.QFormLayout.FieldRole, self.lockTimeLabel)
        self.tissueThicknessVerticalLayout = QtWidgets.QVBoxLayout()
        self.tissueThicknessVerticalLayout.setObjectName("tissueThicknessVerticalLayout")
        self.tissueThicknessLineEdit = QtWidgets.QLineEdit(self.experimentSettingsTab)
        self.tissueThicknessLineEdit.setObjectName("tissueThicknessLineEdit")
        self.tissueThicknessVerticalLayout.addWidget(self.tissueThicknessLineEdit)
        self.tissueThicknessUnitsComboBox = QtWidgets.QComboBox(self.experimentSettingsTab)
        self.tissueThicknessUnitsComboBox.setObjectName("tissueThicknessUnitsComboBox")
        self.tissueThicknessVerticalLayout.addWidget(self.tissueThicknessUnitsComboBox)
        self.experimentSettingsFormLayout.setLayout(1, QtWidgets.QFormLayout.LabelRole, self.tissueThicknessVerticalLayout)
        self.experimentsSettingsVerticalLayout.addLayout(self.experimentSettingsFormLayout)
        self.finishedOptionVerticalLayout = QtWidgets.QVBoxLayout()
        self.finishedOptionVerticalLayout.setObjectName("finishedOptionVerticalLayout")
        self.soundDoneCheckBox = QtWidgets.QCheckBox(self.experimentSettingsTab)
        self.soundDoneCheckBox.setObjectName("soundDoneCheckBox")
        self.finishedOptionVerticalLayout.addWidget(self.soundDoneCheckBox)
        self.ledDoneCheckBox = QtWidgets.QCheckBox(self.experimentSettingsTab)
        self.ledDoneCheckBox.setObjectName("ledDoneCheckBox")
        self.finishedOptionVerticalLayout.addWidget(self.ledDoneCheckBox)
        self.popUpDoneCheckBox = QtWidgets.QCheckBox(self.experimentSettingsTab)
        self.popUpDoneCheckBox.setChecked(True)
        self.popUpDoneCheckBox.setObjectName("popUpDoneCheckBox")
        self.finishedOptionVerticalLayout.addWidget(self.popUpDoneCheckBox)
        self.experimentsSettingsVerticalLayout.addLayout(self.finishedOptionVerticalLayout)
        self.loadExperimentSettingsHorizontalLayout = QtWidgets.QHBoxLayout()
        self.loadExperimentSettingsHorizontalLayout.setObjectName("loadExperimentSettingsHorizontalLayout")
        self.loadExperimentSettingsLineEdit = QtWidgets.QLineEdit(self.experimentSettingsTab)
        self.loadExperimentSettingsLineEdit.setObjectName("loadExperimentSettingsLineEdit")
        self.loadExperimentSettingsHorizontalLayout.addWidget(self.loadExperimentSettingsLineEdit)
        self.loadExperimentSettingsButton = QtWidgets.QPushButton(self.experimentSettingsTab)
        self.loadExperimentSettingsButton.setObjectName("loadExperimentSettingsButton")
        self.loadExperimentSettingsHorizontalLayout.addWidget(self.loadExperimentSettingsButton)
        self.experimentsSettingsVerticalLayout.addLayout(self.loadExperimentSettingsHorizontalLayout)
        self.saveExperimentSettingsButton = QtWidgets.QPushButton(self.experimentSettingsTab)
        self.saveExperimentSettingsButton.setObjectName("saveExperimentSettingsButton")
        self.experimentsSettingsVerticalLayout.addWidget(self.saveExperimentSettingsButton)
        self.verticalLayout_12.addLayout(self.experimentsSettingsVerticalLayout)
        self.mainTabStack.addTab(self.experimentSettingsTab, "")
        self.gridLayout_5.addWidget(self.mainTabStack, 0, 0, 1, 1)
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtWidgets.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 625, 22))
        self.menubar.setObjectName("menubar")
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QtWidgets.QStatusBar(MainWindow)
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)
        self.mainTabStack.setCurrentIndex(2)
        self.graphsTab.setCurrentIndex(0)
        self.transducerSettingsTab.setCurrentIndex(1)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow"))
        self.graphicsTab.setToolTip(_translate("MainWindow", "Main Window with Live Graphs"))
        self.graphsTab.setTabText(self.graphsTab.indexOf(self.flowGraphTab), _translate("MainWindow", "Flow"))
        self.graphsTab.setTabText(self.graphsTab.indexOf(self.pressureGraphTab), _translate("MainWindow", "Pressure"))
        self.graphsTab.setTabText(self.graphsTab.indexOf(self.currentGraphTab), _translate("MainWindow", "Current"))
        self.graphsTab.setTabText(self.graphsTab.indexOf(self.voltageGraphTab), _translate("MainWindow", "Voltage"))
        self.logButton.setText(_translate("MainWindow", "Log"))
        self.sensorInfo.setText(_translate("MainWindow", "sensorInfo"))
        self.runStopButton.setText(_translate("MainWindow", "Run"))
        self.calculateResultButton.setText(_translate("MainWindow", "Calculate Result"))
        self.mainTabStack.setTabText(self.mainTabStack.indexOf(self.graphicsTab), _translate("MainWindow", "Graph"))
        self.flowMeterSettingLabel.setText(_translate("MainWindow", "<html><head/><body><p><span style=\" font-weight:600;\">Flow Meter</span></p></body></html>"))
        self.resolutionSettingLabel.setText(_translate("MainWindow", "bits"))
        self.flowMeterResolutionSettingLabel.setText(_translate("MainWindow", "Resolution"))
        self.pressureTransducerSettingsLabel.setText(_translate("MainWindow", "<html><head/><body><p><span style=\" font-weight:600;\">Pressure Transducer</span></p></body></html>"))
        self.leftTransducerRadioButton.setToolTip(_translate("MainWindow", "The LEFT transducer is connected to the high pressure side."))
        self.leftTransducerRadioButton.setText(_translate("MainWindow", "Left"))
        self.rightTransducerRadioButton.setToolTip(_translate("MainWindow", "The RIGHT transducer is connected to the high pressure side."))
        self.rightTransducerRadioButton.setText(_translate("MainWindow", "Right"))
        self.highPressureSideSelectionLabel.setText(_translate("MainWindow", "High Pressure Side"))
        self.leftTransducerSlopeLineEdit.setToolTip(_translate("MainWindow", "Slope of left transducer in [pressure units]/mV"))
        self.leftTransducerSlopeLabel.setText(_translate("MainWindow", "Slope"))
        self.leftTransducerInterceptLineEdit.setToolTip(_translate("MainWindow", "Intercept of left transducer in mV"))
        self.leftTransducerInerceptLabel.setText(_translate("MainWindow", "Intercept"))
        self.leftTransducerPressureUnitsLabel.setText(_translate("MainWindow", "Pressure Units"))
        self.leftTransducerSerialLineEdit.setToolTip(_translate("MainWindow", "Left transducer serial number"))
        self.leftTransducerSerialLabel.setText(_translate("MainWindow", "Serial Number"))
        self.leftTransducerTypeLabel.setText(_translate("MainWindow", "Transducer Type"))
        self.loadLeftTransducerSettingsButton.setText(_translate("MainWindow", "Load Cal File"))
        self.transducerSettingsTab.setTabText(self.transducerSettingsTab.indexOf(self.leftTransducerSettingsTab), _translate("MainWindow", "Left Transducer"))
        self.rightTransducerSlopeLineEdit.setToolTip(_translate("MainWindow", "Slope of right transducer in Pa/mV"))
        self.rightTransducerSlopeLabel.setText(_translate("MainWindow", "Slope"))
        self.rightTransducerInterceptLineEdit.setToolTip(_translate("MainWindow", "Intercept of left transducer in mV"))
        self.rightTransducerInerceptLabel.setText(_translate("MainWindow", "Intercept"))
        self.rightTransducerPressureUnitsLabel.setText(_translate("MainWindow", "Pressure Units"))
        self.rightTransducerSerialLineEdit.setToolTip(_translate("MainWindow", "Right transducer serial number"))
        self.rightTransducerSerialLabel.setText(_translate("MainWindow", "Serial Number"))
        self.rightTransducerTypeLabel.setText(_translate("MainWindow", "Transducer Type"))
        self.loadRightTransducerSettingsButton.setText(_translate("MainWindow", "Load Cal File"))
        self.transducerSettingsTab.setTabText(self.transducerSettingsTab.indexOf(self.rightTransducerSettingsTab), _translate("MainWindow", "Right Transducer"))
        self.loadFlowReaderSettingsButton.setText(_translate("MainWindow", "Load Settings"))
        self.saveSensorSettingsButton.setText(_translate("MainWindow", "Save Settings"))
        self.mainTabStack.setTabText(self.mainTabStack.indexOf(self.sensorSettingsTab), _translate("MainWindow", "Sensor Settings"))
        self.saveResultsNameLineEdit.setToolTip(_translate("MainWindow", "The filename for the results"))
        self.saveResultsNameLineEdit.setText(_translate("MainWindow", "result.csv"))
        self.saveLocationPushButton.setText(_translate("MainWindow", "Save Results Location"))
        self.crossSectionLabel.setText(_translate("MainWindow", "Cross-Sectional Area"))
        self.tissueThicknessLabel.setText(_translate("MainWindow", "Tissue Thickness"))
        self.flowSetpointLabel.setText(_translate("MainWindow", "Flow Rate Setpoint"))
        self.flowRateLabel.setText(_translate("MainWindow", "Flow Rate"))
        self.averagingDepthLabel.setText(_translate("MainWindow", "Averaging Depth"))
        self.errorBoundsLabel.setText(_translate("MainWindow", "Setpoint Error Bounds"))
        self.lockTimeLabel.setText(_translate("MainWindow", "Lock In Time"))
        self.soundDoneCheckBox.setText(_translate("MainWindow", "Play Sound When Done"))
        self.ledDoneCheckBox.setText(_translate("MainWindow", "Turn On LED When Done"))
        self.popUpDoneCheckBox.setText(_translate("MainWindow", "Pop-Up with Results When Done"))
        self.loadExperimentSettingsButton.setText(_translate("MainWindow", "Load Settings"))
        self.saveExperimentSettingsButton.setText(_translate("MainWindow", "Save Settings"))
        self.mainTabStack.setTabText(self.mainTabStack.indexOf(self.experimentSettingsTab), _translate("MainWindow", "Experiment Settings"))
from pyqtgraph import PlotWidget


if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    MainWindow = QtWidgets.QMainWindow()
    ui = Ui_MainWindow()
    ui.setupUi(MainWindow)
    MainWindow.show()
    sys.exit(app.exec_())