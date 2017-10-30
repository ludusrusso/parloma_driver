#! /usr/bin/env python
# -*- coding: utf-8 -*-

'''
Copyright (C) 2014 Politecnico di Torino

This program is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 2 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License along
with this program; if not, write to the Free Software Foundation, Inc.,
51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.

This software is developed within the PARLOMA project, which aims
at developing a communication system for deablinf people (www.parloma.com)
The PARLOMA project is developed with the Turin node of AsTech laboraroies
network of Italian CINI (Consorzio Interuniversitario Nazionale di Informatica)

Contributors:
    Giuseppe Airò Farulla (giuseppe.airofarulla@polito.it)
	Ludovico O. Russo (ludovico.russo@polito.it)
'''

## @package hand_widget
# Documentation for file hand_widget.py
#
# Code within this file is to draw and manage the GUI used to easily edit the XML file
# concerning all the valid signs that can be executed by the robotic actuator.

# Import required Python code.
import sys
from PyQt4 import QtGui, QtCore
import signal

## @brief Documentation for class HandWidget
#
# This class extends QtGui.QWidget and is in charge of managing the GUI within the file hand_widget.py.
class HandWidget(QtGui.QWidget):

    ## @brief Documentation for function __init__
    #
    # Constructor for HandWidget class. It retrieves the dictionary with all the valid joints,
    # with their codes and their minimum feasible position, and unmask abort signal 
    # (so that it is possible to abort by pressing CTRL-C).
    # @param implemented_joints_minPos_dict
    def __init__(self, implemented_joints_minPos_dict):
        # PyQT init
        self.app = QtGui.QApplication(sys.argv)
        super(HandWidget, self).__init__()
        self.minPos_dict = implemented_joints_minPos_dict

        # Unmask CTRL-C abort signal
        signal.signal(signal.SIGINT, signal.SIG_DFL)
        self._initUI()

    ## @brief Documentation for function run
    #
    # PyQT main loop.
    def run(self):
        sys.exit(self.app.exec_())

    ## @brief Documentation for function _initUI
    #
    # This function initialites the layout, creates all the widgets and addes them to the layout.
    def _initUI(self):
        # GUI init
        vbox = QtGui.QVBoxLayout()

        # init and fill sliders, that are shown in raising code order
        self.slds = []
        i = 0
        while True:
            for joint in self.minPos_dict:
                if self.minPos_dict[joint][1] != i:
                    continue

                # init sliders, lcd and name labels
                sld = QtGui.QSlider(QtCore.Qt.Horizontal, self)
                sld.setMinimum(100*self.minPos_dict[joint][0])
                sld.setMaximum(100)
                lcd = QtGui.QLCDNumber(self)
                sld.valueChanged.connect(lcd.display)
                jointLabel = QtGui.QLineEdit(joint, self)
                jointLabel.setReadOnly(True)
                font = jointLabel.font()
                font.setFamily('Courier')
                font.setPointSize(8)

                hbox = QtGui.QHBoxLayout()
                hbox.addWidget(jointLabel)
                hbox.addWidget(sld)
                hbox.addWidget(lcd)   
                self.slds.append(sld)
                vbox.addLayout(hbox)

                i += 1
            if i == len(self.minPos_dict):
                break

        self.setLayout(vbox)

        # init signs combobox with 'default' configuration (fake sign)
        self.comboBox = QtGui.QComboBox()
        vbox.addWidget(self.comboBox)
        self.connect(self.comboBox, QtCore.SIGNAL('activated(int)'), self.change_config)
        self.configs = []
        self.dyn = []
        self.add_config('default', len(self.slds)*[0])

        hbox = QtGui.QHBoxLayout()

        # init 'message send' and 'sign save' buttons
        self.bsend = QtGui.QPushButton('Send',self)
        hbox.addWidget(self.bsend)
        self.bsave = QtGui.QPushButton('Save',self)
        self.bsave.setEnabled(False)
        hbox.addWidget(self.bsave)
        
        # init signlabel for adding new signs
        self.signLabel = QtGui.QLineEdit('Sign Name', self)
        #TODO clear signLabel on click event
        #self.connect(self.signLabel, QtCore.SIGNAL('mousePressEvent()'), self.erase_signLabel)
        self.connect(self.signLabel, QtCore.SIGNAL('textEdited(QString)'), self.enable_badds)
        self.signLabel.setReadOnly(False)
        font = self.signLabel.font()
        font.setFamily('Courier')
        font.setPointSize(8)
        hbox.addWidget(self.signLabel)

        # init 'add sign' and 'add row' buttons
        self.badds = QtGui.QPushButton('Add Sign',self)
        self.badds.setEnabled(False)
        hbox.addWidget(self.badds)
        self.baddr = QtGui.QPushButton('Add Row',self)
        self.baddr.setEnabled(False)
        hbox.addWidget(self.baddr)

        # init checkbox for 'dynamic' attribute
        dyn = QtGui.QLineEdit('Dynamic', self)
        dyn.setReadOnly(True)
        font = dyn.font()
        font.setFamily('Courier')
        font.setPointSize(8)
        hbox.addWidget(dyn)
        self.checkbox = QtGui.QCheckBox(self)
        self.checkbox.setEnabled(False)
        hbox.addWidget(self.checkbox)

        # set layout, geometry and show
        vbox.addLayout(hbox)
        self.setGeometry(300, 300, 650, 350)
        self.setWindowTitle('Hand Calibration')
        self.show()

    #def erase_signLabel(self):
    #    self.signLabel.clear()

    ## @brief Documentation for function enable_badds
    #
    # This function manages the 'add sign' button which is enabled
    # only when valid text is written on the signLabel.
    # @param index
    def enable_badds(self, index):
        if len(self.signLabel.text()) > 0:
            self.badds.setEnabled(True)
        else:
            self.badds.setEnabled(False)

    ## @brief Documentation for function change_config
    #
    # This function manages the 'save sign' and 'add row' buttons and the checkbox state.
    # @param index
    def change_config(self, index):
        self.set_scrolls(self.configs[index])
        self.checkbox.setChecked(self.dyn[index])
        if index > 0:
            self.bsave.setEnabled(True)
            self.baddr.setEnabled(True)
            self.checkbox.setEnabled(True)
        else:
            self.bsave.setEnabled(False)
            self.baddr.setEnabled(False)
            self.checkbox.setEnabled(False)

    ## @brief Documentation for function add_config
    #
    # This function fills the combobox with new valid items (i.e. sign rows), which are
    # by default treated as static.
    # @param name
    # @param scrolls_positions
    # @param dyn
    def add_config(self, name, scrolls_positions, dyn=False):
        if len(scrolls_positions) == len(self.slds):
            self.comboBox.addItem(name)
            self.configs.append([float(sld*100.) for sld in scrolls_positions])
            self.dyn.append(dyn)

    ## @brief Documentation for function connect_sliders
    #
    # This function connects all the sliders with the proper callbacks,
    # so that a new command is sent to the robot any time the slider configuration changes.
    # @param signal
    def connect_sliders(self, signal):
        for sld in self.slds:
            self.connect(sld, QtCore.SIGNAL('valueChanged(int)'), signal)

    ## @brief Documentation for function connect_button_send
    #
    # This function connects the 'send sign' button with the proper callback.
    # @param signal
    def connect_button_send(self, signal):
        self.connect(self.bsend, QtCore.SIGNAL('clicked()'), signal)

    ## @brief Documentation for function connect_button_save
    #
    # This function connects the 'save sign' button with the proper callback.
    # @param signal
    def connect_button_save(self, signal):
        self.connect(self.bsave, QtCore.SIGNAL('clicked()'), signal)

    ## @brief Documentation for function connect_button_adds
    #
    # This function connects the 'add sign' button with the proper callback.
    # @param signal
    def connect_button_adds(self, signal):
        self.connect(self.badds, QtCore.SIGNAL('clicked()'), signal)

    ## @brief Documentation for function connect_button_addr
    #
    # This function connects the 'add row' button with the proper callback.
    # @param signal
    def connect_button_addr(self, signal):
        self.connect(self.baddr, QtCore.SIGNAL('clicked()'), signal)

    ## @brief Documentation for function connect_checkbox
    #
    # This function connects the checkbox with the proper callback.
    # @param signal
    def connect_checkbox(self, signal):
        self.connect(self.checkbox, QtCore.SIGNAL('clicked()'), signal)

    ## @brief Documentation for function get_scrolls
    #
    # @returns This function retrieves and returns sliders position, reported in range [0;1] or [-1;1].
    def get_scrolls(self):
        # Get sliders values
        values = [float(sld.value()/100.) for sld in self.slds]
        return values

    ## @brief Documentation for function set_scrolls
    #
    # This function sets sliders position, reported in range [0;100] or [-100;100].
    def set_scrolls(self, values):
        # Set sliders values
        if isinstance(values, list):
            if len(values) == len(self.slds):
                for i in range(0,len(self.slds)):
                    self.slds[i].setValue(values[i])
        elif isinstance(values, int):
            for i in range(0,len(self.slds)):
                self.slds[i].setValue(values)

    ## @brief Documentation for function get_combobox
    #
    # @returns This function retrieves and returns combobox currently highlighted sign.
    def get_combobox(self):
        return str(self.comboBox.currentText())
