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

## @package parloma_driver
# Documentation for file modules.py
#
# This script parses the XML file that lists all the commands, all the signs,
# all the joints implemented to control a given robotic actuator.
#
# Error codes are defined as None, {}, [], or integer less than 0.

# Import required Python code.
import xml.etree.ElementTree as ET
import numpy as np
import sys

# Error codes
NON_VALID = -1

## @brief Documentation for class ParserCommands
#
# Class invoked when controlling a robotic actuator.
# This Parser retrieve the integer code(s) for a command (a list of commands).
class ParserCommands:

    ## @brief Documentation for function __init__
    #
    # Constructor for the ParserCommands class. This function tries to read,
    # and to parse, the root of the XML with the codes to send to the motors.
    # @param XML_COMMAND
    def __init__(self, XML_COMMAND):

        # Try to access the XML file
        self.XML_COMMAND = XML_COMMAND
        try:
            self._tree = ET.parse(self.XML_COMMAND)
            self._root = self._tree.getroot()
        except:
            # Valid XML file?!
            self._tree = None
            self._root = None
            print 'Unexpected error:', sys.exc_info()[0]
            print self.XML_COMMAND

        # If valid XML, try to retrieve the commands list
        self._commands_list = []
        try:
            tmp_list = self._root.find('list')
            for child in tmp_list:
                self._commands_list.append(child.text)
        except Exception as e:
            # Correct XML file?!
            print e

    ## @brief Documentation for function _parse
    #
    # This function should NOT be invoked directly.
    # @param cmd
    # @returns This function returns from a single command, its numeric code.
    def _parse(self, cmd):
        if self._root is None:
            # XML not parsed correctly
            return NON_VALID

        if cmd not in self._commands_list:
            # Command non valid
            code = NON_VALID
        else:
            # If valid XML, try to retrieve the codes list
            try:
                _codes = self._root.find('codes')
                # Fill, and return, the dictionary with valid pairs (command, code as int)
                code = int(_codes.find(cmd).text)
            except Exception as e:
                # Correct XML file?!
                print e
                return NON_VALID

        return code

    ## @brief Documentation for function parse
    #
    # @param cmds
    # @returns This function returns a dictionary with pairs (command, code as int) or a single code,
    # as cmds may be a list of commands or a single command respectively.
    def parse(self, cmds):

        if isinstance(cmds, list):
            # If cmds is a list
            # Create the dictionary to store pairs command - code
            cmds_dict = {}

            # Loop over all the commands
            for cmd in cmds:
                cmds_dict[cmd] = self._parse(cmd)
            return cmds_dict
            #return cmds_dict.values()
        else:
            # Parsing a single command
            return self._parse(cmds)
#end class ParserCommands


## @brief Documentation for class ParserRobot
#
# This class retrieves the relative poses a robot has to assume given a generic poses in percentage.
class ParserRobot:

    ## @brief Documentation for function __init__
    #
    # Constructor for the ParserRobot class. It tries to read, and parse, the root
    # of the XML file describing the robot architecture.
    # @param XML_ROBOT
    def __init__(self, XML_ROBOT):
        self.XML_ROBOT = XML_ROBOT

        # Try to access the XML file
        try:
            self._tree_robot = ET.parse(self.XML_ROBOT)
            self._root_robot = self._tree_robot.getroot()
        except:
            # Valid XML file?!
            self._tree_robot = None
            self._root_robot = None
            print 'Unexpected error:', sys.exc_info()[0]

        # TODO should be generalized to obtain self.all_valid_joints_Element_list
        self._right_hand_joints_list = []
        self._right_hand_joints_reduced_dict = {}
        self._right_hand_joints_Element_dict = {}

        hands = self._root_robot.find('hands')
        right_hand = hands.find('right')

        for child in right_hand:
            if str(child.find('implemented').text) == 'True':
                    self._right_hand_joints_list.append(child.attrib['name_id'])
                    self._right_hand_joints_Element_dict[child.attrib['name_id']] = child
                    self._right_hand_joints_reduced_dict[str(child.attrib['name_id'])] = [int(child.find('minvalue').text), int(child.find('code').text)]

    ## @brief Documentation for function parse_implemented_joints
    #
    # @returns This function returns a dictionary with pairs {motor_name, motor_node within the XML} (only for implemented limbs).
    def parse_implemented_joints(self):
        return self._right_hand_joints_Element_dict

    ## @brief Documentation for function parse_implemented_joints_minPos
    #
    # @returns This function returns a dictionary with pairs {motor_name, [motor minimum value, motor code]} (only for implemented limbs).
    def parse_implemented_joints_minPos(self):
        return self._right_hand_joints_reduced_dict

    ## @brief Documentation for function _parse
    #
    # Pose must be a dictionary with pairs {joint, percentage}.
    # This method should NOT be invoked directly.
    # @param pose
    # @returns This function returns from a single pose, a dictionary with pairs {motor_code, motor_pose} (only for implemented robots)
    # that can be used to control the robot.
    def _parse(self, pose):
        if self._root_robot is None:
            return {}

        if isinstance(pose, dict) == False:
            print "WARNING Invalid input!"
            return {}

        tmp_poses_dict = {}

        for j in pose.keys():
            if j not in self._right_hand_joints_list:
                # Not implemented joint
                continue
            _el = self._right_hand_joints_Element_dict[j]
            tmp_implemented = str(_el.find('implemented').text)
            tmp_code = int(_el.find('code').text)
            if tmp_implemented == 'False' or tmp_code == NON_VALID:
                # Consistency error - Check the XML
                print "WARNING joint " + str(j) + "should be implemented but it is not!"
                continue
            tmp_minrange = float(_el.find('minrange').text)
            tmp_maxrange = float(_el.find('maxrange').text)
            tmp_restposition = float(_el.find('restposition').text)

            # Converting pose in working range for the joint:
            # minrange - maxrange when there is no rest position
            # otherwise, rest position - maxrange when val pose is in 0 - +1
            # otherwise, minrange - rest position
            val = float(pose[j])

            if tmp_restposition == -1.:
                b = tmp_minrange
                c = tmp_maxrange
                if val >= 0.:
                    a = val
                else:
                    a = val + 1
            else:
                if val >= 0.:
                    a = val
                    b = tmp_maxrange
                    c = tmp_restposition
                else:
                    a = val + 1
                    b = tmp_restposition
                    c = tmp_minrange

            cmd_num = a * (b - c) + c

            # Map int(round(cmd_num)) to cmd[int(child.find('code').text)]
            # to have pairs (motor_code_as_int, motor_pose) in the dictionaries
            tmp_poses_dict[tmp_code] = int(round(cmd_num))

        return tmp_poses_dict

    ## @brief Documentation for function parse
    #
    # Each pose must be a dictionary with pairs {joint, percentage}.
    # @param poses
    # @returns This function returns, from a single pose or a list of poses, a dictionary with pairs {motor_code, motor_pose}
    # (only for implemented robots) that can be used to control the robot.
    def parse(self, poses):
        if isinstance(poses, dict):
            # Parsing a single pose
            return self._parse(poses)
        else:
            # Parsing a liste of poses
            tmp_list_poses = []
            for pose in poses:
                tmp_list_poses.append(self._parse(pose))
            return tmp_list_poses

#end class ParserRobot


## @brief Documentation for class ParserSigns
#
# This Parser retrieves the alphabet and the poses for each sign (a list of signs).
# In addition, it can modify existing signs and add new ones.
class ParserSigns:

    ## @brief Documentation for function __init__
    #
    # Constructor for the ParserSigns class. It tries to read, and parse, the XML file
    # listing all the valid signs in the alphabet with their shapes and properties.
    # @param XML_SIGNS
    def __init__(self, XML_SIGNS):
        self.XML_SIGNS = XML_SIGNS
        # Header string used for automatically rewriting XML_SIGNS
        self.XML_HEADER = "\
<?xml version=\'1.0\' encoding=\'UTF-8\'?> \n \
<!-- This xml exposes handshapes for letters from LIS alphabet. --> \n \
<!-- This file is automatically generated. --> \n \
"

        # Try to access the XML file
        try:
            self._tree_signs = ET.parse(self.XML_SIGNS)
            self._root_signs = self._tree_signs.getroot()
            self._update_alphabet()
        except:
            # Valid XML file?!
            self._tree_signs = None
            self._root_signs = None
            print 'Unexpected error:', sys.exc_info()[0]

        # If valid XML files, try to retrieve the joints list and the alphabet
        # all_joint_list is the list of all the valid joints
        # all_joints_dict is the dictionary of pairs {valid_joint, min_value}
        # min_value is 0 or 1 for flexion/extension joints or ad/abduction joints, respectively
        # (max value is always 1) (values are in percentage)
        self.all_joints_list = []
        self.all_joints_dict = {}

        try:
            tmp_joints = self._root_signs.find('joints')
            for child in tmp_joints:
                self.all_joints_list.append(child.text)
                # Comment this row below to have only the implemented joints
                #if child.text in self._right_hand_joints_list:
                self.all_joints_dict[child.text] = child.attrib['min']
        except Exception as e:
            # Correct XML file?!
            print e

    ## @brief Documentation for function _update_alphabet
    #
    # This function updates (or creates) the alphabet listing all the valid signs.
    def _update_alphabet(self):
        self.alphabet = [sign.attrib['name_id'] for sign in self._root_signs.find('alphabet')]

    ## @brief Documentation for function parse_alphabet
    #
    # @returns This function returns the alphabet of valid signs, useful especially for graphic XML editor.
    def parse_alphabet(self):
        if self._root_signs is None:
            return []
        else:
            return self.alphabet

    ## @brief Documentation for function _parse_sign_rowsN
    #
    # This method should NOT be invoked directly.
    # @param sign
    #
    # @returns This function returns the number of rows (i.e. intermediate poses to assume) for a given sign for graphic XML editor.
    def _parse_sign_rowsN(self, sign):
        if self._root_signs is None or sign not in self.alphabet:
            return -1
        else:
            for s in self._root_signs.find('alphabet'):
                if s.attrib['name_id'] == sign:
                    return int(s.find('rowsN').text)
            # Should never arrive here
            return -1

    ## @brief Documentation for function parse_signs_rowsN
    #
    # GUI should asks for rowsN sign by sign for all the signs in the alphabet.
    # For legacy reason, below method accepts also a list of signs.
    # @param signs
    # @returns This function returns the number of rows (i.e. intermediate poses to assume) for a given sign (or a list of signs).
    def parse_signs_rowsN(self, signs):
        if isinstance(signs, str) == True:
            return self._parse_sign_rowsN(signs)
        else:
            # Parsing a list
            tmp_rows_list = []
            for sign in signs:
                tmp_rows_list.append(self._parse_sign_rowsN(sign))
            return tmp_rows_list

    ## @brief Documentation for function parse_joints
    #
    # This dictionary contains pairs {name of the joint: minimum feasible value}.
    # @returns This function returns a dictionary containing all the joints the GUI needs to implement.
    def parse_joints(self):
        return self.all_joints_dict

    ## @brief Documentation for function _add_sign
    #
    # This method should NOT be invoked directly
    # @param sign
    # @param dynamic
    # @returns This function adds a new sign (1 row, all the joints set to 0) and returns the new alphabet.
    def _add_sign(self, sign, dynamic):
        if self._root_signs is None:
            return []

        if sign in self.alphabet:
            print 'WARNING: Sign ' + str(sign) + ' already existing!'
        else:
            alph = self._root_signs.find('alphabet')
            _el = ET.Element('sign', {'name_id':str(sign), 'dynamic':str(dynamic), 'type_id':'matrix'})
            _el_rowsN = ET.Element('rowsN')
            _el_rowsN.text = str(1)
            _el.append(_el_rowsN)
            _el_row = ET.Element('row', {'row_id':'0','type_id':'array'})
            for joint in self.all_joints_dict:
                _el_row_joint = ET.Element(joint)
                _el_row_joint.text = str(0)
                _el_row.append(_el_row_joint)
            _el.append(_el_row)
            alph.append(_el)

            # Saves the new sign in the xml
            out_file = open(self.XML_SIGNS,'w')
            out_file.write(self.XML_HEADER)
            out_file.write('\n')
            out_file.write(ET.tostring(self._root_signs))
            out_file.close()

            # Updates and return alphabet
            self._update_alphabet()

        return self.alphabet

    ## @brief Documentation for function add_signs
    #
    # GUI should add sign by sign for all the signs to add.
    # For legacy reason, below method accepts also a list of signs.
    # @param signs
    # @param dynamic
    # @returns This function adds a new sign (1 row, all the joints set to 0) or a set of signs
    # (from the dictionary dynamic={sign:boolean dyn}) and returns the new alphabet.
    def add_signs(self, signs, dynamic):
        if isinstance(signs, str) == True:
            return self._add_sign(signs, dynamic)
        elif isinstance(dynamic, dict) == True:
            # Parsing a list
            # dynamic must be a dictionary with pairs {sign, bool}
            for sign in signs:
                alph = self._add_sign(sign, dynamic[sign])
            return alph
        else:
            # Invalid inputs
            print "WARNING: One or more inputs are invalid!"
            return self.alphabet

    ## @brief Documentation for function add_row
    #
    # @param sign
    # @returns This function adds a new row for a sign (all the joints set to 0) and returns the new row count.
    def add_row(self, sign):
        if self._root_signs is None:
            return -1

        if sign not in self.alphabet:
            print 'WARNING: Sign ' + str(sign) + '  NOT existing!'
            return -1
        else:
            for s in self._root_signs.find('alphabet'):
                if s.attrib['name_id'] == sign:
                    _el_rowsN = s.find('rowsN')
                    tmp_row = int(_el_rowsN.text)
                    _el_rowsN.text = str(1 + tmp_row)
                    _el_row = ET.Element('row', {'row_id':str(tmp_row), 'type_id':'array'})
                    for joint in self.all_joints_dict:
                        _el_row_joint = ET.Element(joint)
                        _el_row_joint.text = str(0)
                        _el_row.append(_el_row_joint)
                    s.append(_el_row)

                    # Saves the new sign in the xml
                    out_file = open(self.XML_SIGNS,'w')
                    out_file.write(self.XML_HEADER)
                    out_file.write('\n')
                    out_file.write(ET.tostring(self._root_signs))
                    out_file.close()

                    return self.parse_signs_rowsN(sign)

            # Should never arrive here
            return -1

    ## @brief Documentation for function parse_dynamic
    #
    # @param sign
    # @returns This function returns the value of the dynamic attribute for a given sign.
    def parse_dynamic(self, sign):
        if self._root_signs is None:
            return None

        if isinstance(sign, str) == False:
            print 'WARNING: Invalid input value!'
            return None

        if sign not in self.alphabet:
            print 'WARNING: Sign ' + str(sign) + '  does NOT exist!'
            return None
        else:
            for s in self._root_signs.find('alphabet'):
                if s.attrib['name_id'] == sign:
                    if str(s.attrib['dynamic']) == 'True':
                        return True
                    else:
                        return False

            # Should never arrive here
            return -1

    ## @brief Documentation for function toggle_dynamic
    #
    # @param sign
    # @returns This function toggles and returns the new value of the dynamic attribute for a given sign.
    def toggle_dynamic(self, sign):
        if self._root_signs is None:
            return -1

        if isinstance(sign, str) == False:
            print 'WARNING: Invalid input value!'
            return -1

        if sign not in self.alphabet:
            print 'WARNING: Sign ' + str(sign) + '  does NOT exist!'
            return -1
        else:
            for s in self._root_signs.find('alphabet'):
                if s.attrib['name_id'] == sign:
                    if str(s.attrib['dynamic']) == 'True':
                        s.attrib['dynamic'] = 'False'
                    else:
                        s.attrib['dynamic'] = 'True'

                    # Saves the new sign in the xml
                    out_file = open(self.XML_SIGNS,'w')
                    out_file.write(self.XML_HEADER)
                    out_file.write('\n')
                    out_file.write(ET.tostring(self._root_signs))
                    out_file.close()

                    # Succeed
                    return 0

            # Should never arrive here
            return -1

    ## @brief Documentation for function edit_sign
    #
    # Overwrites a single row (pose_idx) of a single sign with percentages described in dictionary
    # (percentages_dict) of pairs {joint, new_percentage}.
    # @param sign
    # @param pose_idx
    # @param percentages_dict
    # @returns This function returns only error or success (integer 0) code.
    def edit_sign(self, sign, pose_idx, percentages_dict):
        if self._root_signs is None:
            return -1

        if isinstance(sign, str) == False:
            # Input sign is not valid
            print 'Input ' + str(sign) + '  is not valid.'
            return -1
        if isinstance(pose_idx, int) == False:
            # Input pose_idx is not valid
            print 'Input ' + str(pose_idx) + '  is not valid.'
            return -1
        if isinstance(percentages_dict, dict) == False:
            # Input pose_idx is not valid
            print 'Input ' + str(percentages_dict) + '  is not valid.'
            return -1

        if sign not in self.alphabet:
            print 'WARNING sign ' + str(sign) + '  is not existing.'
            return -1

        for s in self._root_signs.find('alphabet'):
            if s.attrib['name_id'] == sign:
                break

        if s.attrib['name_id'] != sign:
            # Correct XML file and valid sign?! Consistency error!!
            return -1

        size = int(s.find('rowsN').text)
        if pose_idx < 0 or pose_idx >= size:
            # Row index out of bounds
            print 'For sign ' + str(sign) + '  out of bounds row ' + str(pose_idx)
            return -1

        # Search for the desired row
        rows_list = []
        for row_child in s:
            if row_child.tag == 'row':
                rows_list.append(row_child)
        # When not found, cannot edit the row
        found = False
        row = ''
        for rr in rows_list:
            row = rr
            if int(row.attrib['row_id']) == pose_idx:
                # Desired row found
                found = True
                break
        if found == False:
            # Desired row not found. Exit with error
            print 'For sign ' + str(sign) + '  the row_id ' + str(r) + '  is missing!'
            return -1

        file_modified = False

        # Search for the desired joint(s)
        for joint in percentages_dict.keys():
            j = row.find(joint)

            tmp_j = joint in self.all_joints_list

            if j is None or tmp_j is False:
                # Consistency error - Check the XML
                print 'For sign ' + str(sign) + '  and row_id ' + str(pose_idx) + '  the joint ' + str(joint) + '  does not exist.'
                # print 'WARNING: Coherence error!'
                # print 'GUI asks for editing ' + str(joint) + '  which is not implemented.'
                continue

            # If new value is coherent with joint limits, accept it and overwrite the old value
            if percentages_dict[joint] <= 1 and percentages_dict[joint] >= float(self.all_joints_dict[joint]):
                j.text = str(percentages_dict[joint])
                file_modified = True
            else:
                print 'For sign ' + str(sign) + '  and row_id ' + str(pose_idx) + '  and joint ' + str(joint) + ' \
                     the value ' + str(percentages_dict[joint]) + '  is not within valid bounds.'
                continue

        # If everything was OK, write the new xml and succeed
        if file_modified:
            out_file = open(self.XML_SIGNS,'w')
            out_file.write(self.XML_HEADER)
            out_file.write('\n')
            out_file.write(ET.tostring(self._root_signs))
            out_file.close()

        return 0

    ## @brief Documentation for function parse_sign_row
    #
    # @param sign
    # @param pose_idx
    # @returns This function retrieves for a given sign and a given row (pose_idx) all the poses in percentages,
    # returning a dictionary with pairs {motor_code as int/string, motor_pose}.
    def parse_sign_row(self, sign, pose_idx):
        if self._root_signs is None:
            return {}

        if sign not in self.alphabet:
            print "Sign " + str(sign) + " does not exist!"
            return {}

        if isinstance(sign, str) == False or isinstance(pose_idx, int) == False:
            print "One or more input values are not valid"
            return

        for s in self._root_signs.find('alphabet'):
            if s.attrib['name_id'] == sign:
                break

        if s.attrib['name_id'] != sign:
            # Correct XML file and valid sign?! Consistency error!!
            return {}

        # Is the sign dynamic or not?
        # dyn = s.attrib['dynamic']

        child = ''
        all_cmds_dict = {}

        # size is the number of rows in the matrix mat for the sign s
        # Please refer to the xml file for more detailed comments
        size = int(s.find('rowsN').text)
        #mat = np.zeros((1, len(self.all_joints_list)), dtype=np.float)

        # Tries to explore the rows with crescent ID -> a missing row_id should be a mistake?!
        rows_list = []
        for row_child in s:
            if row_child.tag == 'row':
                rows_list.append(row_child)

        # A non-found row is an empty node
        found = False
        row = ''
        for rr in rows_list:
            row = rr
            if int(row.attrib['row_id']) == pose_idx:
                # Desired row found
                found = True
                break
        if found == False:
            # Desired row not found
            print 'For sign ' + str(sign) + '  the row_id ' + str(pose_idx) + '  is missing!'
            return {}

        # Checking row consistency and saving the desired wor (if valid)
        if len(row) != len(self.all_joints_list):
            print 'ERROR: For sign ' + str(sign) + '  the row_id ' + str(r) + '  has incorrect length ' + str(len(row))
            return {}
        for child in row:
            try:
                #mat[0, self.all_joints_list.index(child.tag)] = float(child.text)
                all_cmds_dict[child.tag] = float(child.text)
            except Exception as e:
                print 'ERROR: For sign ' + str(sign) + '  the row_id ' + str(r) + '  encountered error: ' + str(e)
                continue

        return all_cmds_dict

    ## @brief Documentation for function parse_sign_rows
    #
    # For sending commands to control the robot it is necessary to parse all the rows of the sign.
    # This function retrieves for a given sign all the poses in percentages.
    # @param sign
    # @param kinematic
    # @returns a dictionaries of dictionaries {pose_idx, {motor_code as int/string, motor_pose}}
    def parse_sign_rows(self, sign, kinematic):

        if self._root_signs is None:
            return {}

        if sign not in self.alphabet:
            print "Sign " + str(sign) + " does not exist!"
            return {}

        if isinstance(sign, str) == False:
            print "One or more input values are not valid"
            return

        for s in self._root_signs.find('alphabet'):
            if s.attrib['name_id'] == sign:
                break

        if s.attrib['name_id'] != sign:
            # Correct XML file and valid sign?! Consistency error!!
            return {}

        # Is the sign dynamic or not?
        dyn = s.attrib['dynamic']

        child = ''
        all_cmds_dict = {}

        # size is the number of rows in the matrix mat for the sign s
        # Please refer to the xml file for more detailed comments
        size = int(s.find('rowsN').text)
        mat = np.zeros((size, len(self.all_joints_list)), dtype=np.float)

        # Tries to explore the rows with crescent ID -> a missing row_id should be a mistake?!
        rows_list = []
        for row_child in s:
            if row_child.tag == 'row':
                rows_list.append(row_child)

        for r in range(size):
            # A non-found row is an empty node
            found = False
            row = ''
            for rr in rows_list:
                row = rr
                if int(row.attrib['row_id']) == r:
                    # Desired row found
                    found = True
                    break
            if found == False:
                # Desired row not found
                print 'For sign ' + str(sign) + '  the row_id ' + str(r) + '  is missing!'
                return {}

            # Checking row consistency and saving the desired wor (if valid)
            if len(row) != len(self.all_joints_list):
                print 'ERROR: For sign ' + str(sign) + '  the row_id ' + str(r) + '  has incorrect length ' + str(len(row))
                continue
            for child in row:
                try:
                    mat[r, self.all_joints_list.index(child.tag)] = float(child.text)
                except Exception as e:
                    print 'ERROR: For sign ' + str(sign) + '  the row_id ' + str(r) + '  encountered error: ' + str(e)
                    continue

        child = ''

        if str(kinematic) == 'True' and str(dyn) == 'False':
            # Considering only the final pose for a static sign when fingers anti-entanglement is implemented by the robot
            tmp_row_cmds_dict = {}
            for j in self.all_joints_list:
                arr = mat[-1]
                val = arr[self.all_joints_list.index(j)]
                tmp_row_cmds_dict[j] = val
            all_cmds_dict[0] = tmp_row_cmds_dict
        else:
            for cmd_row in range(mat.shape[0]):
                tmp_row_cmds_dict = {}
                for j in self.all_joints_list:
                    val = mat[cmd_row, self.all_joints_list.index(j)]
                    tmp_row_cmds_dict[j] = val
                all_cmds_dict[cmd_row] = tmp_row_cmds_dict

        return all_cmds_dict

    ## @brief Documentation for function parse_signs_rows
    #
    # Signs should be parsed one by one.
    # For legacy reason, this method accepts also a list of signs.
    # @param signs
    # @param kinematic
    # @returns This function returns a dictionaries of dictionaries {pose_idx, {motor_code as int/string, motor_pose}}
    # (or a list of those) for a given sign (or a list of input signs).
    def parse_signs_rows(self, signs, kinematic=False):
        if isinstance(signs, str) == True:
            return self.parse_sign_rows(signs, kinematic)
        else:
            # Parsing a list
            tmp_all_signs_dict = {}
            for sign in signs:
                tmp_all_signs_dict[sign] = self.parse_sign_rows(sign, kinematic)
            return tmp_all_signs_dict

#end class ParserSigns


# main
def main():
    #p = ParserSigns('robot_hand_Bulga.xml', 'signs2pose.xml')
    #print p.parse_joints()
    #print p.parse(None)
    #print p.edit_sign('R', 3, {'thumb_MCP':0.8, 'thumb_MCP_A':-1})
    #print p.parse('R')

    #return

    pc = ParserCommands('../../xml/commands_list.xml')
    #print pc.parse(['set_all_motors', 'set_one_motor', 'l'])

    pr = ParserRobot('../../xml/robot_hand.xml')
    pr.parse([])

    ps = ParserSigns('../../xml/signs2pose.xml')

    #print ps.edit_sign('A', 1, {'thumb_PIP':1, 'thumb_MCP':1})
    print ps.add_signs('50', 'True')
    #print ps.edit_sign('A', 1, {'thumb_PIP':1, 'thumb_MCP':1})

    #return
    print ps.parse_alphabet()
    print ps.parse_joints()
    print ps.parse_signs_rowsN('PIPPO')
    #print ps.parse_sign_row('A',0)
    print ps.parse_sign_rows('R', True)
    print '#'
    print ps.parse_sign_rows('R', False)

if __name__ == '__main__':
    if len(sys.argv) != 1:
        print 'Usage: python script_name'
    else:
        main()
