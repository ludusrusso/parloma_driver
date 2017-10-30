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
	Ludovico O. Russo (ludovico.russo@polito.it)
	Giuseppe Airò Farulla (giuseppe.airofarulla@polito.it)
'''

## @package sign2command
# Documentation for file sign2command.py
#
# This ROS node parses single letter and number of the sentences
# it receives from the client node (as soon at it receives them).
# Then, it publishes on the 'serial_publisher' topic the commands
# needed to control the robotic actuator, to let it assume the
# desired shapes.

# Import required Python code.
import roslib
import rospy
import sys

from serial_bridge.msg import generic_serial
from std_msgs.msg import String

from parloma_driver.modules import ParserSigns
from parloma_driver.modules import ParserRobot
from parloma_driver.modules import ParserCommands

## @brief Documentation for class HandDriver
#
# This class is in charge of managing the ROS node within the file sign2command.py.
class HandDriver:

    ## @brief Documentation for function __init__
    #
    # Constructor for HandDriver class. It retrieves the XML files, initializes the parsers and the ROS topics.
    def __init__(self):

	self.OLD_SIGN = ""

        # get parameters
        self.output_topic = rospy.get_param('serial_topic', '/serial_topic');

        xml_hand = rospy.get_param('xml_hand', 'robot_hand.xml')
        self.xml_signs = rospy.get_param('xml_signs', 'signs2pose.xml')
        xml_commands = rospy.get_param('xml_commands', 'commands_list.xml')

        self.ps = ParserSigns(self.xml_signs)
        self.pc = ParserCommands(xml_commands)
        self.pr = ParserRobot(xml_hand)

        # get parameters
        self._input_topic = rospy.get_param('signs_topic', '/signs_topic');
        self._output_topic = rospy.get_param('serial_topic', '/serial_topic');

        # init topics
        rospy.Subscriber(self._input_topic, String, self.sign_callback)
        self._serial_pub = rospy.Publisher(self._output_topic, generic_serial, queue_size=10)
        rospy.loginfo(rospy.get_caller_id() + " Node Initialized")
        rospy.spin()

    ## @brief Documentation for function sign_callback
    #
    # Callback function to retrieve and publish the commands, needed to control the robotic actuator,
    # to let it reproduce the input sign.
    # @param StrSign
    def sign_callback(self, StrSign):

        sign = str(StrSign)
	print "[sign2command.py] REC " + sign
	#print "AAA:::" + sign[-4:]
	if sign[-4:] == "REST":
            self.OLD_SIGN = ""
            print "[sign2command.py] HARD CODED REST "
	    self._send_rest()
	    return

        sign2 = sign[-1]

	if sign2==self.OLD_SIGN:
            print "[sign2command.py] OLD SIGN " + sign2
	    return
        
	print "[sign2command.py] NEW SIGN " + sign2
        self.OLD_SIGN = sign2
        # ps = ParserSigns(self.xml_signs)
        s = self.ps.parse_sign_rows(str(sign2), False)

        cmds = self.pr.parse(s.values())
        print cmds

        if cmds == None or len(cmds) == 0:
            print "[sign2command.py] Sign " + sign2 + " not found!"
            self._send_rest()
        else:
            for cmd in cmds:
                msg = generic_serial()
                msg.msg = [self.pc.parse('set_all_motors')]
                for c in range(len(cmd.keys())):
                    msg.msg.append(cmd[c])
                self._serial_pub.publish(msg)
                rospy.sleep(0.2)

    ## @brief Documentation for function _send_rest
    #
    # Test function to send rest position
    def _send_rest(self):
        msg = generic_serial()
        msg.msg = [241, 180, 180, 180, 180, 180, 70, 90, 90, 120]
        self._serial_pub.publish(msg)


# main
if __name__ == '__main__':
    rospy.init_node('hand_driver', anonymous=True)
    try:
        ne = HandDriver()
    except rospy.ROSInterruptException:
        pass
