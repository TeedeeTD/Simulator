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
# from pynput import keyboard
from RcBrainThread import RcBrainThread
from std_msgs.msg import String
import sys
import tty
import termios
import rospy
import select

class RemoteControlTransmitterProcess():
    # ===================================== INIT==========================================
    def __init__(self):
        """Run on the PC. It forwards the commans from the user via KeboardListenerThread to the RcBrainThread. 
        The RcBrainThread converts them into actual commands and sends them to the remote via a socket connection.
        
        """
        self.dirKeys   = ['w', 'a', 's', 'd','q']
        self.paramKeys = ['t','g','y','h','u','j','i','k', 'r', 'p']
        self.pidKeys = ['z','x','v','b','n','m']

        self.allKeys = self.dirKeys + self.paramKeys + self.pidKeys
        
        self.rcBrain   =  RcBrainThread()   
        self.settings = termios.tcgetattr(sys.stdin)
        self.key = None
        rospy.init_node('EXAMPLEnode', anonymous=False)     
        self.publisher = rospy.Publisher('/automobile/command', String, queue_size=1)

    # ===================================== RUN ==========================================
   
    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            self.key = sys.stdin.read(1)
        else:
            self.key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
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
    def shutdown(self):
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
    # ===================================== KEY RELEASE ==================================
    def keyRelease(self, key):
        """Processing the key realeasing.
        
        Parameters
        ----------
        key : pynput.keyboard.Key
            The key realeased. 
        
        """ 
        # if key == keyboard.Key.esc:                        #exit key      
        #     self.publisher.publish('{"action":"3","steerAngle":0.0}')   
        #     return False
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
            # rospy.loginfo("cmd:%s",command)
            self.publisher.publish(command)  
            
if __name__ == '__main__':
    try:

        nod = RemoteControlTransmitterProcess()
        nod.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        nod.shutdown()