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

## @package calibration
# Documentation for file calibration.py
#
# Code within this file is tomanage the HandWidget GUI used to easily edit the XML file
# concerning all the valid signs that can be executed by the robotic actuator.

# Import required Python code.
import roslib; roslib.load_manifest('parloma_driver')
import rospy
import sys
from serial_bridge.msg import generic_serial

from hand_widget import HandWidget
from parloma_driver.modules import ParserCommands
from parloma_driver.modules import ParserSigns
from parloma_driver.modules import ParserRobot

## @brief Documentation for class HandCalibrator
#
# This class manages HandWidget for GUI creation, user interaction, XML parsing, ROS messages.
class HandCalibrator:

    ## @brief Documentation for function __init__
    #
    # Constructor for HandCalibrator class. It retrieves the XML files, initializes the parsers and the ROS topics.
    # In addition, it initializes the GUI widget items and link them to the proper callbacks.
    def __init__(self):
        # get parameters (output topic and XML files)
        self.output_topic = rospy.get_param('serial_topic', '/serial_topic');
        self.xml_hand = rospy.get_param('xml_hand', 'robot_hand.xml')
        self.xml_signs = rospy.get_param('xml_signs', 'signs2pose.xml')
        self.xml_commands = rospy.get_param('xml_commands', 'commands_list.xml')

        # self.xml_hand = '/home/beppe/Scrivania/PARLOMA_ROS/robotcontrol/ros/parloma_driver/xml/robot_hand.xml'
        # self.xml_signs = '/home/beppe/Scrivania/PARLOMA_ROS/robotcontrol/ros/parloma_driver/xml/signs2pose.xml'
        # self.xml_commands = '/home/beppe/Scrivania/PARLOMA_ROS/robotcontrol/ros/parloma_driver/xml/commands_list.xml'

        # init XML parsers and topics
        self.ps = ParserSigns(self.xml_signs)
        self.pc = ParserCommands(self.xml_commands)
        self.pr = ParserRobot(self.xml_hand)
        self.serial_pub = rospy.Publisher(self.output_topic, generic_serial, queue_size=10)
        # Parsing signs alphabet and joints list
        alphabet = self.ps.parse_alphabet()
        all_joints = self.ps.parse_joints()
        implemented_joints_dict = self.pr.parse_implemented_joints()
        self.implemented_joints_minPos_dict = self.pr.parse_implemented_joints_minPos()
        self.implemented_jointsN = len(self.implemented_joints_minPos_dict)
        cmds = self.ps.parse_signs_rows(alphabet)
        #print self.implemented_joints_minPos_dict
        #print self.implemented_jointsN

        # init the widget
        self.widget = HandWidget(self.implemented_joints_minPos_dict)
        # TODO set_scrolls for default position from getting the list of current motor positions - fingertips feedback
        self.widget.set_scrolls(0)
        for sign in cmds.keys():
            cmd_dict = cmds[sign]
            for k in cmd_dict:
                cmd = []
                for c in cmd_dict[k]:
                    cmd.append(cmd_dict[k][c])
                    self.widget.add_config(sign + " - %s"%k, cmd, self.ps.parse_dynamic(sign)==True)
        # connect widget items with proper callbacks
        self.widget.connect_button_send(self.send_sign)
        self.widget.connect_button_save(self.save_sign)
        self.widget.connect_button_adds(self.add_sign)
        self.widget.connect_button_addr(self.add_row)
        #self.widget.connect_sliders(self.send_sign)
        self.widget.connect_checkbox(self.toggle_dyn)

        rospy.loginfo(rospy.get_caller_id() + " Node Initialized")
        self.widget.run()

    ## @brief Documentation for function toggle_dyn
    #
    # This function toggle the dynamic boolean associated to a valid selected sign.
    def toggle_dyn(self):
        # Toggle dynamic bool info for a given sign
        s = self.widget.get_combobox()

        if s == "default":
            return

        sign = str(s.split(' - ')[0])
        self.ps.toggle_dynamic(sign)

    # Old legacy function
    #def sign_callback(self, sign):
    #    # Perform a given sign (if valid)
    #    cmds = self.ps.parse([sign.data])
    #    if cmds is None or len(cmds) == 0:
    #        self.send_rest()
    #    else:
    #        for cmd in cmds:
    #            msg = generic_serial()
    #            msg.msg = [self.pc.parse(['set_all_motors'])[0]]
    #            for c in cmd:
    #                msg.msg.append(cmd[c])
    #            self.serial_pub.publish(msg)
    #            rospy.sleep(0.5)

    ## @brief Documentation for function send_sign
    #
    # This function controls robot motors using sliders positions.
    # It is called whenever the 'send sign' button is pressed or sliders configuration changes.
    def send_sign(self):
        msg = generic_serial()
        scrolls_list = self.widget.get_scrolls()
        scrolls_dict = {}

        # Sliders are sorted by raising codes
        for joint in self.pr.parse_implemented_joints_minPos():
            scrolls_dict[joint] = scrolls_list[self.implemented_joints_minPos_dict[joint][1]]

        # Uncomment following lines to manually link sliders and motors
        #scrolls_dict['thumb_MCP'] = scrolls_list[0]
        #scrolls_dict['index_MCP'] = scrolls_list[1]
        #scrolls_dict['middle_MCP'] = scrolls_list[3]
        #scrolls_dict['ring_MCP'] = scrolls_list[4]
        #scrolls_dict['pinky_MCP'] = scrolls_list[4]
        #scrolls_dict['thumb_MCP_A'] = scrolls_list[5]
        #scrolls_dict['index_MCP_A'] = scrolls_list[6]
        #scrolls_dict['middle_MCP_A'] = scrolls_list[7]
        #scrolls_dict['wrist'] = scrolls_list[8]

        # send ROS message
        msg.msg = [self.pc.parse('set_all_motors')] + self.pr.parse(scrolls_dict).values()
        #msg.msg = [self.pc.parse('set_all_motors')] + self.widget.get_scrolls()

        self.serial_pub.publish(msg)

    ## @brief Documentation for function send_sign
    #
    # This function controls robot motors using sliders positions.
    # It is called whenever the 'save sign' button is pressed.
    def save_sign(self):
        # Save a given sign (if valid) from sliders positions
        s = self.widget.get_combobox()

        if s == "default":
            return

        sign = str(s.split(' - ')[0])
        row = int(s.split(' - ')[1])

        print "Saving data for ---%s---%s---"%(sign,row)

        scrolls_list = self.widget.get_scrolls()
        scrolls_dict = {}

        # Sliders are sorted by raising codes
        for joint in self.pr.parse_implemented_joints_minPos():
            scrolls_dict[joint] = scrolls_list[self.implemented_joints_minPos_dict[joint][1]]

        # Uncomment following lines to manually link sliders and motors
        #scrolls_dict['thumb_MCP'] = scrolls_list[0]
        #scrolls_dict['index_MCP'] = scrolls_list[1]
        #scrolls_dict['middle_MCP'] = scrolls_list[2]
        #scrolls_dict['ring_MCP'] = scrolls_list[3]
        #scrolls_dict['pinky_MCP'] = scrolls_list[4]
        #scrolls_dict['thumb_MCP_A'] = scrolls_list[5]
        #scrolls_dict['index_MCP_A'] = scrolls_list[6]
        #scrolls_dict['middle_MCP_A'] = scrolls_list[7]
        #scrolls_dict['wrist'] = scrolls_list[8]

        # print "Before editing"
        # print self.ps.parse_sign_row(sign, row)
        self.ps.edit_sign(sign, row, scrolls_dict)
        # print "After editing"
        # print self.ps.parse_sign_row(sign, row)

    ## @brief Documentation for function add_sign
    #
    # This function tries to add a new sign and updates the widget GUI, if succeed.
    # It is called whenever the 'add sign' button is pressed.
    def add_sign(self):
        # Add new sign (default: dynamic=False, 1 row, all motors set to 0)
        sign = str(self.widget.signLabel.text())
        print "Adding new sign " + str(sign)
        # Try to add the new sign
        cmds = self.ps.add_signs(sign, False)
        if cmds is not None:
            # if succeed, update the widget
            self.widget.badds.setEnabled(False)
            self.widget.signLabel.clear()
            self.widget.add_config(sign + " - %s"%0, self.implemented_jointsN*[0.])

    ## @brief Documentation for function add_row
    #
    # This function tries to add a new row and updates the widget GUI, if succeed.
    # It is called whenever the 'add row' button is pressed.
    def add_row(self):
        # Add a new row for a given sign (if valid, default: 1 row, all motors set to 0)
        s = self.widget.get_combobox()
        if s == "default":
            return
        sign = str(s.split(' - ')[0])
        print "Adding new row for sign " + str(sign)

        # Try to add the new row
        rowN = self.ps.add_row(sign) - 1
        if rowN >= 0:
            # if succeed, update the widget
            cmds = self.ps.parse_sign_rows(sign, False)
            self.widget.add_config(sign + " - %s"%rowN, self.implemented_jointsN*[0.])

    ## @brief Documentation for function send_rest
    #
    # This function performs hard-coded rest sign.
    def send_rest(self):
        msg = generic_serial()
        msg.msg = [241, 180, 180, 180, 180, 180, 70, 90 ,50 ,120]
        self.serial_pub.publish(msg)


# main
if __name__ == '__main__':
    rospy.init_node('arduino_hand_calibrator', anonymous=True)
    try:
        ne = HandCalibrator()
    except rospy.ROSInterruptException:
        pass
