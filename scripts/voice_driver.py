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

'''
# For a possible future, but this feature is not implemented yet
The service as soon as receives from the client node 
the sentence will convert each letter to the upper case
and then transposes letter by letter in the corrispondent
in Morse code. In addition to letters and numbers 
it is possible to convert also some symbols. 
The Morse code is executed with both the Buzzer
and a led.
'''

## @package voice_driver
# Documentation for file voice_driver.py
#
# This ROS node requires 'espeak' to be installed to work properly.
# It pronounces every single letter and number of the sentences
# it receives from the client node (as soon at it receives them).

# Import required Python code.
import os
import rospy
from std_msgs.msg import String

## @brief Documentation for class VoiceNode
#
# This class is in charge of managing the ROS node within the file voice_driver.py.
class VoiceNode:

    ## @brief Documentation for function sign_callback
    #
    # Callback function to pronounce (Italian spelling) a valid string letter by letter.
    # @param data
    def sign_callback(self, data):
        print data.data
        if self.lastPh == data.data:
            return
        self.lastPh = data.data
        for letter in data.data:
            os.system('espeak -v it '+letter)

    ## @brief Documentation for function __init__
    #
    # Constructor for the VoiceNode class. It launces the 'voice_driver' node,
    # reads necessary parameters and links the sign_callback function as callback
    # for the input_topic (expected message is a String).
    def __init__(self):

        # Identifies the null String to not be pronounced
        self.lastPh = ''
        
        print 'Ready to convert'
        rospy.init_node('voice_driver')
        self.input_topic = rospy.get_param("input_topic", '/signs_topic')
        self.subs = rospy.Subscriber(self.input_topic, String, self.sign_callback)
        rospy.spin()

# main
if __name__=='__main__':
    nh = VoiceNode()
