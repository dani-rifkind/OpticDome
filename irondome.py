import RPi.GPIO as GPIO
import cv2
import numpy as np
import time
import threading
GPIO.setmode(GPIO.BCM)
global run
global motor_thread
global on_target
global motorX
global motorY
global motorPower
global x
global y
global w
global h
global mid_x
global mid_y
laser = 26
cap = cv2.VideoCapture(0)

width, height = 550, 320
cap.set(3, width)
cap.set(4, height)
xCenter, yCenter = int(width / 2), int(height / 2) 
  
_, frame = cap.read()
class Pin:
    def __init__(self, pin):
        self.pin = pin
        GPIO.setup(pin, GPIO.OUT)
        self.state = False
        GPIO.output(pin, GPIO.LOW)
    def on(self):
        if not self.state:
            GPIO.output(self.pin, GPIO.HIGH)
            self.state = True
    def off(self):
        if self.state:
            GPIO.output(self.pin, GPIO.LOW)
            self.state = False
class PulsePin(Pin):
    def __init__(self, pin):
        super().__init__(pin=pin)
        self.doing = False
    def on(self):
        if self.doing:
            return False
        self.doing = True
        pulse = threading.Thread(target=self.helper)
        pulse.start()
        return True 
    def helper(self):
        super().on()
        time.sleep(0.4)
        super().off()
        self.doing = False
class LimitedPulsePin(PulsePin):
    def __init__(self, pin):
        super().__init__(pin=pin)
        self.count = 0
        
    def on(self):
        if self.count < 3:
            if super().on():
                self.count += 1
    def reset_counter(self):
        self.count = 0
class Nema:
    motors = []
    def __init__(self, dire_given, step_given, e_given=0):
        self.dire = dire_given
        self.step = step_given
        self.e = e_given
        self.count = 0
        Nema.motors.append(self)
        # Setup for the enable if it is being used
        GPIO.setup(self.dire, GPIO.OUT)
        GPIO.setup(self.step, GPIO.OUT)
        GPIO.output(self.dire, GPIO.LOW)
        GPIO.output(self.step, GPIO.LOW)
        # Output for the enable if it is neing used
    def do_step(self, direction, amount=0):
        # Change enable if it is being used
        self.count += 1 if direction else -1
        GPIO.output(self.dire, GPIO.HIGH if direction else GPIO.LOW)
        GPIO.output(self.step, GPIO.HIGH)
        time.sleep(0.0005)
        GPIO.output(self.step, GPIO.LOW)
        time.sleep(0.0025 if x <= xCenter <= x + w and y <= yCenter <= y + h else 0.005) # The speed control.
        # Change enable if it is being used

    @staticmethod
    def reset():
        for motor in Nema.motors:
            while motor.count < 0:
                motor.do_step(True)
            while motor.count > 0:
                motor.do_step(False)
    
def checkCenteredObject():
    l, r = mid_x
    t, b = mid_y
    return l <= xCenter <= r and t <= yCenter <= b
def move_motors():
    while not on_target:
        if run:
            l, r = mid_x
            t, b = mid_y
            if not l <= xCenter <= r:
                motorX.do_step(xCenter > l)
            if not t <= yCenter <= b:
                motorY.do_step(yCenter < t)
def check_state(*args):
    global run
    run = GPIO.input(25)
def check(c):
    rect = cv2.boundingRect(c)
    return rect[2] > 20 and rect[3] > 20
def main():
    global x
    global y
    global w
    global h
    global mid_x
    global mid_y
    global on_target
    global motor_thread
    laserPin = LimitedPulsePin(laser)
    buzzerPin = PulsePin(13)
    redPin = Pin(19)
    greenPin = Pin(24)
    
    greenPin.on()
    redPin.off()
    x, y, w, h = 0, 0, 0, 0
    low_red = np.array([0,50,50])
    high_red = np.array([10,255,255])
    low_green = np.array([124,252,10])
    high_green = np.array([255,255,150])    
    while True:
        _, frame = cap.read()
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        if run:
            motorPower.on()
            redPin.on()
            greenPin.off()
            red_mask = cv2.inRange(hsv_frame, low_red, high_red)
            green_mask = cv2.inRange(hsv_frame, low_green, high_green)
            contours_g, _ = cv2.findContours(green_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            contours, _ = cv2.findContours(red_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            contours += contours_g
            contours = sorted(contours, key=lambda x: cv2.contourArea(x), reverse=True)
            contours = list(filter(lambda x: check(x), contours))

            if contours != []:
                x, y, w, h = cv2.boundingRect(contours[0])
                mid_x = [((x + x + w) / 2) - (w / 3), ((x + x + w) / 2) + (w / 3)]
                mid_y = [((y + y + h) / 2) - (h / 3), ((y + y + h) / 2) + (h / 3)]
                if checkCenteredObject():
                    on_target = True
                    laserPin.on()
                    buzzerPin.on()
                else:
                    if not motor_thread.is_alive():
                        on_target = False
                        laserPin.reset_counter()

                        motor_thread = threading.Thread(target=move_motors)
                        motor_thread.start()
                cv2.line(frame, (x, y), (x + w, y + h), (0, 0, 255) if (x <= xCenter <= (x + w)) and (y <= yCenter <= (y+ h)) else (255, 0, 0), 3)
                cv2.line(frame, (x + w, y), (x, y + h), (0, 0, 255) if (x <= xCenter <= (x + w)) and (y <= yCenter <= (y+ h)) else (255, 0, 0), 3)
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255) if (x <= xCenter <= (x + w)) and (y <= yCenter <= (y+ h)) else (255, 0, 0), 3)
            else:
                 on_target = True
                 Nema.reset()
        else:
            motorPower.off()
            greenPin.on()
            redPin.off()
        cv2.line(frame, (0, yCenter), (int(width / 2) - 20, yCenter), (0, 0, 0), 2)
        cv2.line(frame, (int(width / 2) + 20, yCenter), (width, yCenter), (0, 0, 0), 2)
        cv2.line(frame, (xCenter, 0), (xCenter, int(height / 2) - 20), (0, 0, 0), 2)
        cv2.line(frame, (xCenter, int(height / 2) + 20), (xCenter, height), (0, 0, 0), 2)                        
        cv2.imshow("Frame", frame)
        key = cv2.waitKey(1)
        if key == 27:
            motorPower.off()
            print("Unpluged")
            GPIO.cleanup()
            print("I cleaned up")
            break
            
    
    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    run  = False
    GPIO.setup(25, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.add_event_detect(25, GPIO.BOTH, callback=check_state)
    motorX = Nema(22, 23)#, 24)
    motorY = Nema(20 , 21)#, 4)
    motorPower = Pin(6)
   # motorPower.on()
    on_target = True
    x = 0
    y = 0
    w = 0
    h = 0
    mid_x = []
    mid_y = []
    motor_thread = threading.Thread(target=move_motors)

    try:
        #server_thread = threading.Thread(target=app_run)
        #server_thread.start()
#        switch_thread = threading.Thread(target=check_state)
#        switch_thread.start()
        main()
        
    except Exception as e:
        print(e)
    finally:
        motorPower.off()
        print("Unpluged")
        GPIO.cleanup()
        print("I cleaned up")
        








