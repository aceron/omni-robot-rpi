import RPi.GPIO as GPIO
import time
import math
import threading

GPIO.setmode(GPIO.BOARD)

chan_list = [33, 35, 36, 37, 38, 40]
enc_list =  [13, 15, 16, 18, 29, 31]

GPIO.setup(chan_list, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(enc_list, GPIO.IN, pull_up_down=GPIO.PUD_UP)

enc = [0, 0, 0]
prev_enc = [0, 0, 0]
vel = [0, 0, 0]
enc_last = ["00", "00", "00"]
states = {"0001":1, "0010":-1, "0100":-1, "0111":1, "1000":1, "1011":-1, "1101":-1, "1110":1}

can_display = True
can_control = True

v_x = 0.2
v_y = 0.2
v_t = 0.0

ppr = 4*80*1 # 4 pulses per motor rev., 80 motor rev. = 1 wheel rev.
duty = [0.0, 0.0, 0.0]

w = [0.0, 0.0, 0.0]

r = 0.0240
R = 0.1041
theta = 0.0 #15.0*math.pi/180.0
a = [theta, theta+120.0*math.pi/180.0, theta+240.0*math.pi/180.0]


freq = 200.0

pwm = []
pwm.append(GPIO.PWM(38, freq))
pwm.append(GPIO.PWM(40, freq))
pwm.append(GPIO.PWM(36, freq))
pwm.append(GPIO.PWM(37, freq))
pwm.append(GPIO.PWM(33, freq))
pwm.append(GPIO.PWM(35, freq))

for idx in range(0, 3):
		pwm[idx*2].start(0.0)
		pwm[idx*2+1].start(0.0)

w[0] = (-math.sin(a[0])*math.cos(a[0])*v_x + math.cos(a[0])*math.cos(theta)*v_y + R*v_t)/r
w[1] = (-math.sin(a[1])*math.cos(a[1])*v_x + math.cos(a[1])*math.cos(theta)*v_y + R*v_t)/r
w[2] = (-math.sin(a[2])*math.cos(a[2])*v_x + math.cos(a[2])*math.cos(theta)*v_y + R*v_t)/r

print(w)

def encoder_0_cbk(chan):
        global enc, enc_last, states
        curr = str(GPIO.input(13)) + str(GPIO.input(15))
        key = enc_last[0] + curr
        if key in states:
                drctn = states[key]
                enc_last[0] = curr
                enc[0] += drctn

def encoder_1_cbk(chan):
        global enc, enc_last, states
        curr = str(GPIO.input(16)) + str(GPIO.input(18))
        key = enc_last[1] + curr
        if key in states:
                drctn = states[key]
                enc_last[1] = curr
                enc[1] += drctn
				
def encoder_2_cbk(chan):
        global enc, enc_last, states
        curr = str(GPIO.input(29)) + str(GPIO.input(31))
        key = enc_last[2] + curr
        if key in states:
                drctn = states[key]
                enc_last[2] = curr
                enc[2] += drctn

GPIO.add_event_detect(13, GPIO.BOTH, callback=encoder_0_cbk)
GPIO.add_event_detect(15, GPIO.BOTH, callback=encoder_0_cbk)
GPIO.add_event_detect(16, GPIO.BOTH, callback=encoder_1_cbk)
GPIO.add_event_detect(18, GPIO.BOTH, callback=encoder_1_cbk)
GPIO.add_event_detect(29, GPIO.BOTH, callback=encoder_2_cbk)
GPIO.add_event_detect(31, GPIO.BOTH, callback=encoder_2_cbk)


def display_stats():
        global can_display, enc, vel, w
        while(can_display):
                print("command: " + str(w))
                print("pulses: " + str(enc))
                print("speed: " + str(vel))
                time.sleep(0.25)


def control_loop():
        global v_x, v_y, v_t, enc, vel, prev_enc, ppr, duty, w
        while(can_control):
                for idx in range(0, 3):
                        vel[idx] = 2*math.pi*(enc[idx] - prev_enc[idx])/ppr
                        prev_enc[idx] = enc[idx]
                        err = vel[idx] - w[idx]
                        if err > 0.0:
                                duty[idx] -= 5.0
                        elif err < 0.0:
                                duty[idx] += 5.0
								
                        if duty[idx] > 100.0:
                                duty[idx] = 100.0
                        elif duty[idx] < -100.0:
                                duty[idx] = -100.0

						if duty[idx] == 0.0:
								pwm[idx*2].ChangeDutyCycle(0.0)
								pwm[idx*2+1].ChangeDutyCycle(0.0)
						elif duty[idx] < 0.0:
								pwm[idx*2].ChangeDutyCycle(0.0)
								pwm[idx*2+1].ChangeDutyCycle(-duty[idx])
						else:
								pwm[idx*2].ChangeDutyCycle(duty[idx])
								pwm[idx*2+1].ChangeDutyCycle(0.0)
                time.sleep(0.025)
				
dp = threading.Thread(target=display_stats)
dp.start()
ct = threading.Thread(target=control_loop)
ct.start()

time.sleep(10)

can_display = False
can_control = False
dp.join()
ct.join()
for mtr in pwm:
        mtr.stop()
GPIO.cleanup()