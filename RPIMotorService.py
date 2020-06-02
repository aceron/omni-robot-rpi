import RPi.GPIO as GPIO
import time
import threading
import numpy as np

from concurrent import futures
import logging

import grpc

import rpi_motor_pb2
import rpi_motor_pb2_grpc

class RPIMotorServiceImpl(rpi_motor_pb2_grpc.RPIMotorServicer):

    def __init__(self):
        GPIO.setmode(GPIO.BOARD)

        self.new_cmd = False
        self.chan_list = [33, 35, 36, 37, 38, 40]
        self.enc_list =  [13, 15, 16, 18, 29, 31]

        self.enc = np.array([0, 0, 0], dtype=np.float64)
        self.prev_enc = [0, 0, 0]
        self.vel = np.array([0, 0, 0], dtype=np.float64)
        self.prev_vel = np.array([0, 0, 0], dtype=np.float64)
        self.enc_last = ["00", "00", "00"]
        self.states = {"0001":1, "0010":-1, "0100":-1, "0111":1, "1000":1, "1011":-1, "1101":-1, "1110":1}
        self.err_hist = []

        self.p_gain = np.array([42.5, 42.5, 42.5], dtype=np.float64)
        self.i_gain = np.array([0.0001, 0.0001, 0.0001], dtype=np.float64)
        self.d_gain = np.array([0.000001, 0.000001, 0.000001], dtype=np.float64)
        self.cut_vel = np.array([0.055, 0.055, 0.055], dtype=np.float64)

        self.ppr = 4*80*1 # 4 pulses per motor rev., 80 motor rev. = 1 wheel rev.
        self.duty = np.array([0, 0, 0], dtype=np.float64)
        self.w = np.array([0, 0, 0], dtype=np.float64)

        self.r = 0.0240
        self.R = 0.1041
        self.theta = 0.0*np.pi/180.0
        self.a = np.array([self.theta, self.theta+120.0*np.pi/180.0, self.theta+240.0*np.pi/180.0], dtype=np.float64)
        self.pwm = []
        self.freq = 200.0

        self.can_display = True
        self.can_control = True

        self.v_x = 0.00
        self.v_y = 0.00
        self.v_t = 0.00

        GPIO.setup(self.chan_list, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self.enc_list, GPIO.IN, pull_up_down=GPIO.PUD_UP)

        GPIO.add_event_detect(13, GPIO.BOTH, callback=self.encoder_0_cbk)
        GPIO.add_event_detect(15, GPIO.BOTH, callback=self.encoder_0_cbk)
        GPIO.add_event_detect(16, GPIO.BOTH, callback=self.encoder_1_cbk)
        GPIO.add_event_detect(18, GPIO.BOTH, callback=self.encoder_1_cbk)
        GPIO.add_event_detect(29, GPIO.BOTH, callback=self.encoder_2_cbk)
        GPIO.add_event_detect(31, GPIO.BOTH, callback=self.encoder_2_cbk)

        self.pwm.append(GPIO.PWM(38, self.freq))
        self.pwm.append(GPIO.PWM(40, self.freq))
        self.pwm.append(GPIO.PWM(36, self.freq))
        self.pwm.append(GPIO.PWM(37, self.freq))
        self.pwm.append(GPIO.PWM(33, self.freq))
        self.pwm.append(GPIO.PWM(35, self.freq))

        for idx in range(0, 3):
            self.err_hist.append([])
            self.pwm[idx*2].start(0.0)
            self.pwm[idx*2+1].start(0.0)

        self.dp = threading.Thread(target=self.display_stats)
        self.dp.start()
        self.ct = threading.Thread(target=self.control_loop)
        self.ct.start()

        print("GPIO initialized")

    def encoder_0_cbk(self, chan):
        curr = str(GPIO.input(13)) + str(GPIO.input(15))
        key = self.enc_last[0] + curr
        if key in self.states:
            drctn = self.states[key]
            self.enc_last[0] = curr
            self.enc[0] += float(drctn)

    def encoder_1_cbk(self, chan):
        curr = str(GPIO.input(16)) + str(GPIO.input(18))
        key = self.enc_last[1] + curr
        if key in self.states:
            drctn = self.states[key]
            self.enc_last[1] = curr
            self.enc[1] += float(drctn)

    def encoder_2_cbk(self, chan):
        curr = str(GPIO.input(29)) + str(GPIO.input(31))
        key = self.enc_last[2] + curr
        if key in self.states:
            drctn = self.states[key]
            self.enc_last[2] = curr
            self.enc[2] += float(drctn)

    def display_stats(self):
        while(self.can_display):
            print("---")
            print("command: " + str(self.w))
            print("pulses: " + str(self.enc))
            print("speed: " + str(self.vel))
            print("duty: " + str(self.duty))
            time.sleep(0.25)

    def control_loop(self):
        stop_count = 0
        while(self.can_control):
            if self.new_cmd:
                self.new_cmd = False
                stop_count = 0
            else:
                stop_count += 1

            if stop_count > 50:
               stop_count = 50
               self.v_x = 0.0
               self.v_y = 0.0
               self.v_t = 0.0

            for idx in range(0, 3):
                self.prev_vel[idx] = self.vel[idx]
                self.vel[idx] = (self.prev_vel[idx] + 2*np.pi*(self.enc[idx] - self.prev_enc[idx])/self.ppr)/2.0
                if self.vel[idx] < self.cut_vel[idx] and self.vel[idx] > -self.cut_vel[idx]:
                    self.vel[idx] = 0.0
                self.prev_enc[idx] = self.enc[idx]
                if len(self.err_hist[idx]) > 20:
                    self.err_hist[idx].pop(0)
                self.err_hist[idx].append(self.vel[idx] - self.w[idx])
                p_err = self.err_hist[idx][len(self.err_hist[idx])-1]
                i_err = np.sum(self.err_hist[idx])
                d_err = (self.err_hist[idx][len(self.err_hist[idx])-1] - self.err_hist[idx][len(self.err_hist[idx])-2])/2.0

                output = p_err*self.p_gain[idx] + i_err*self.i_gain[idx] + d_err*self.d_gain[idx]
                self.duty[idx] = output
                
                if self.duty[idx] > 100.0:
                        self.duty[idx] = 100.0
                elif self.duty[idx] < -100.0:
                        self.duty[idx] = -100.0

                if self.duty[idx] == 0.0:
                    self.pwm[idx*2].ChangeDutyCycle(0.0)
                    self.pwm[idx*2+1].ChangeDutyCycle(0.0)
                elif self.duty[idx] < 0.0:
                    self.pwm[idx*2].ChangeDutyCycle(0.0)
                    self.pwm[idx*2+1].ChangeDutyCycle(-self.duty[idx])
                else:
                    self.pwm[idx*2].ChangeDutyCycle(self.duty[idx])
                    self.pwm[idx*2+1].ChangeDutyCycle(0.0)
            
            time.sleep(0.01)

    def SetState(self, request, context):
        result = rpi_motor_pb2.StateReply()

        try:
            self.v_x = request.vel_x
            self.v_y = request.vel_y
            self.v_t = request.vel_t

            self.w = (-np.sin(self.a)*np.cos(self.a)*self.v_x + np.cos(self.a)*np.cos(self.theta)*self.v_y + self.R*self.v_t)/self.r
            self.new_cmd = True
            #self.w[0] = (-np.sin(self.a[0])*np.cos(self.a[0])*self.v_x + np.cos(self.a[0])*np.cos(self.theta)*self.v_y + self.R*self.v_t)/self.r
            #self.w[1] = (-np.sin(self.a[1])*np.cos(self.a[1])*self.v_x + np.cos(self.a[1])*np.cos(self.theta)*self.v_y + self.R*self.v_t)/self.r
            #self.w[2] = (-np.sin(self.a[2])*np.cos(self.a[2])*self.v_x + np.cos(self.a[2])*np.cos(self.theta)*self.v_y + self.R*self.v_t)/self.r

            #print(self.w)

            result.res = True
        except Exception as e:
            print("ERROR: " + str(e))
            result.res = False

        return result

    def __del__(self):
        self.can_display = False
        self.can_control = False
        self.dp.join()
        self.ct.join()
        for mtr in self.pwm:
            mtr.stop()

        GPIO.remove_event_detect(13)
        GPIO.remove_event_detect(15)
        GPIO.remove_event_detect(16)
        GPIO.remove_event_detect(18)
        GPIO.remove_event_detect(29)
        GPIO.remove_event_detect(31)

        time.sleep(1)

        GPIO.cleanup()

        print("GPIO destroyed")

def serve():
    port = 50201
    server = grpc.server(futures.ThreadPoolExecutor(max_workers=3))
    rpi_motor_pb2_grpc.add_RPIMotorServicer_to_server(RPIMotorServiceImpl(), server)
    server.add_insecure_port('[::]:'+str(port))
    server.start()
    print("RPIMotor service started at " + str(port))
    server.wait_for_termination()

if __name__ == '__main__':
    logging.basicConfig()
    serve()
