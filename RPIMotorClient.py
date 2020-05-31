from __future__ import print_function

import random
import logging
import time

import grpc

import rpi_motor_pb2
import rpi_motor_pb2_grpc

def run():
    port = 50201
    channel = grpc.insecure_channel('192.168.1.9:'+str(port))
    client = rpi_motor_pb2_grpc.RPIMotorStub(channel)

    req = rpi_motor_pb2.StateRequest()

    req.vel_x = 0.01
    req.vel_y = 0.01
    req.vel_t = 0.0
    client.SetState(req)

    time.sleep(10)

    req.vel_x = 0.00
    req.vel_y = 0.00
    req.vel_t = 0.0
    client.SetState(req)

    time.sleep(1)

    channel.close()


if __name__ == '__main__':
    logging.basicConfig()
    run()
