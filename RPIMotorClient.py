# Based on https://gist.github.com/claymcleod/028386b860b75e4f5472

from __future__ import print_function

import os
import random
import logging
import time
import pprint

import grpc
import pygame
import threading

import rpi_motor_pb2
import rpi_motor_pb2_grpc

do_poll = True
v_x = 0.0
v_y = 0.0
v_t = 0.0

class PS4Controller(object):
    global do_poll, v_x, v_y, v_t
    """Class representing the PS4 controller. Pretty straightforward functionality."""

    controller = None
    axis_data = None
    button_data = None
    hat_data = None

    def init(self):
        """Initialize the joystick components"""
        
        pygame.init()
        pygame.joystick.init()
        self.controller = pygame.joystick.Joystick(0)
        self.controller.init()

    def listen(self):
        global do_poll, v_x, v_y, v_t
        """Listen for events to happen"""
        
        if not self.axis_data:
            self.axis_data = {}

        if not self.button_data:
            self.button_data = {}
            for i in range(self.controller.get_numbuttons()):
                self.button_data[i] = False

        if not self.hat_data:
            self.hat_data = {}
            for i in range(self.controller.get_numhats()):
                self.hat_data[i] = (0, 0)

        while do_poll:
            for event in pygame.event.get():
                if event.type == pygame.JOYAXISMOTION:
                    self.axis_data[event.axis] = round(event.value,2)
                elif event.type == pygame.JOYBUTTONDOWN:
                    self.button_data[event.button] = True
                elif event.type == pygame.JOYBUTTONUP:
                    self.button_data[event.button] = False
                elif event.type == pygame.JOYHATMOTION:
                    self.hat_data[event.hat] = event.value

                # Insert your code on what you would like to happen for each event here!
                # In the current setup, I have the state simply printing out to the screen.
                
                if self.hat_data[0][0] == 1:
                    v_y = 0.1
                elif self.hat_data[0][0] == -1:
                    v_y = -0.1
                else:
                    v_y = 0.0

                if self.hat_data[0][1] == 1:
                    v_x = 0.1
                elif self.hat_data[0][1] == -1:
                    v_x = -0.1
                else:
                    v_x = 0.0

                if self.button_data[4]==1 and self.button_data[5] == 0:
                    v_t = -0.2 
                elif self.button_data[4]==0 and self.button_data[5] == 1:
                    v_t = 0.2  
                else:
                    v_t = 0.0

                #os.system('clear')
                pprint.pprint(self.button_data)
                pprint.pprint(self.axis_data)
                pprint.pprint(self.hat_data)

                time.sleep(0.025)

def mainThread():
    global do_poll, v_x, v_y, v_t

    port = 50201
    channel = grpc.insecure_channel('192.168.1.11:'+str(port))
    client = rpi_motor_pb2_grpc.RPIMotorStub(channel)

    req = rpi_motor_pb2.StateRequest()

    while do_poll:
        req.vel_x = v_x
        req.vel_y = v_y
        req.vel_t = v_t
        res = client.SetState(req)

        time.sleep(0.05)

    channel.close()


def run():
    global do_poll
    
    ps4 = PS4Controller()
    ps4.init()
    gp = threading.Thread(target=ps4.listen)
    gp.start()
    
    mn = threading.Thread(target=mainThread)
    mn.start()

    while True:
        try:
            time.sleep(1)
        except KeyboardInterrupt:
            print("byebye")
            do_poll = False
            break
        finally:
            print("...")

    gp.join()
    mn.join()


if __name__ == '__main__':
    logging.basicConfig()
    run()
