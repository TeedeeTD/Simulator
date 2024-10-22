#!/usr/bin/env python3

# Copyright (c) 2019, Bosch Engineering Center Cluj and BFMC organizers
# All rights reserved.

# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:

# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.

# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.

# 3. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE

import json
from pynput import keyboard
import logging
import rospy
import math

import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from RcBrainThread import RcBrainThread
from std_msgs.msg import String
import datetime

from bblab_lane_navigation import BBLabLaneNavigation

import time
import sys


class RemoteControlTransmitterProcess():
    # ===================================== INIT==========================================
    def __init__(self):
        """Run on the PC. It forwards the commans from the user via KeboardListenerThread to the RcBrainThread.
        The RcBrainThread converts them into actual commands and sends them to the remote via a socket connection.

        """
        self.dirKeys   = ['w', 'a', 's', 'd']
        self.paramKeys = ['t','g','y','h','u','j','i','k', 'r', 'p']
        self.pidKeys = ['z','x','v','b','n','m']

        self.allKeys = self.dirKeys + self.paramKeys + self.pidKeys
        self.bridge = CvBridge()
        self.cv_image = np.zeros((640, 320))
        self.image_sub = rospy.Subscriber("/automobile/rcCar/camera_follow/image_raw", Image, self.callback)
        self.rcBrain   =  RcBrainThread()

        rospy.init_node('EXAMPLEnode', anonymous=False)
        self.publisher = rospy.Publisher('/automobile/command', String, queue_size=1)
        
        self.last_time = 0
        
    def callback(self, data):
        """
        :param data: sensor_msg array containing the image in the Gazsbo format
        :return: nothing but sets [cv_image] to the usefull image that can be use in opencv (numpy array)
        """
        self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")

    # ===================================== RUN ==========================================
    
    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
                self.key = sys.stdin.read(1)
        else:
                self.key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
    
    def run(self):
        """Apply initializing methods and start the threads. 
        """
        receiv_key=0
        last_key=''
        rospy.loginfo("start")
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.getKey()
            if self.key in self.allKeys:
                receiv_key=1
                
                if(self.key=='q'):
                    self.key='space'
                last_key=self.key
                keyMsg = 'p.' + str(self.key)
                # rospy.loginfo("%s",keyMsg)
                self._send_command(keyMsg)
            else:
                if(self.key==''):
                    if(receiv_key==1):
                        receiv_key=0
                        # if(last_key=='a' or last_key=='d'):
                        #     continue
                        # else:
                        keyMsg_2 = 'r.' + str(last_key)
                        self._send_command(keyMsg_2)
                        # rospy.loginfo("%s",keyMsg_2)
                continue
            if (self.key == '\x03'):
                break
            # rospy.loginfo("not freeze")
            rate.sleep()
        # with keyboard.Listener(on_press = self.keyPress, on_release = self.keyRelease) as listener: 
        #     listener.join()
        # ===================================== KEY PRESS ====================================
    def keyPress(self,key):
        """Processing the key pressing

        Parameters
        ----------
        key : pynput.keyboard.Key
            The key pressed
        """
        try:
            if key.char in self.allKeys:
                keyMsg = 'p.' + str(key.char)

                self._send_command(keyMsg)

        except: pass

    # ===================================== KEY RELEASE ==================================
    def keyRelease(self, key):
        """Processing the key realeasing.

        Parameters
        ----------
        key : pynput.keyboard.Key
            The key realeased.

        """
        if key == keyboard.Key.esc:                        #exit key
            self.publisher.publish('{"action":"3","steerAngle":0.0}')
            return False
        try:
            if key.char in self.allKeys:
                keyMsg = 'r.'+str(key.char)

                self._send_command(keyMsg)

        except: pass
    # ===================================== SEND COMMAND =================================
    def _send_command(self, key):
        """Transmite the command to the remotecontrol receiver.

        Parameters
        ----------
        inP : Pipe
            Input pipe.
        """
        command = self.rcBrain.getMessage(key)
        if command is not None:

            command = json.dumps(command)
            rospy.loginfo("%s",command)
            self.publisher.publish(command)
        
    def shutdown(self):
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        
    def auto_(self):
        
        data_speed = {}
        data_steer ={}
        rate=rospy.Rate(10)
        speed=18.0
        steer=0.0
        data_speed['action']        =  '1'
        data_speed['speed']    =  float(speed/100.0)
        data_steer['action']        =  '2'
        data_steer['steerAngle']    =  float(steer)
        while not rospy.is_shutdown():
            if self.cv_image is not None:
                steel_recv = self.img_proccessor(self.cv_image)
                # New function to process image here
                print('steer angle: %f' % steel_recv)
            data_speed['speed']    =  float(speed/100.0)
            if steel_recv is not None:
                data_steer['steerAngle'] = float(steel_recv)
            else:
                data_steer['steerAngle'] = 0.0  # hoặc bất kỳ giá trị nào phù hợp khác
            command_sp=json.dumps(data_speed)
            command_st=json.dumps(data_steer)

            rospy.loginfo("sp:%f",speed)
            rospy.loginfo("st:%f",steer)
            self.publisher.publish(command_sp)
            self.publisher.publish(command_st)
            cv2.waitKey(1)
            rate.sleep()
        cv2.destroyAllWindows()

    def img_proccessor(self, frame_image):
        self.last_time = time.time()
        result_img, det_angle = bblab_lane_navigation_.navigate_lane(frame_image)
        print(det_angle)
        steel_recv_ = det_angle - 90
        print(steel_recv_)
        
        return steel_recv_
        

if __name__ == '__main__':
    bblab_lane_navigation_ = BBLabLaneNavigation()

    input_mean = 127.5
    input_std = 127.5
    #steel_recv = 0
    try:
        nod = RemoteControlTransmitterProcess()
        nod.auto_()
    except rospy.ROSInterruptException:
        pass
