import time
import os
import math
try:
    from robot_hat import Pin, ADC, PWM, Servo, fileDB
    from robot_hat import Grayscale_Module, Ultrasonic
    from robot_hat.utils import reset_mcu, run_command
except ImportError:
    from sim_robot_hat import Pin, ADC, PWM, Servo, fileDB
    from sim_robot_hat import Grayscale_Module, Ultrasonic
    from sim_robot_hat import reset_mcu, run_command
import logging
import atexit
import concurrent.futures
import sys
import rossros

print(sys.executable)
from readerwriterlock import rwlock

logging_format = "%(asctime)s: %(message)s"
logging.basicConfig(format=logging_format, level=logging.INFO,datefmt="%H:%M:%S")
logging.getLogger().setLevel(logging.DEBUG)

# reset robot_hat
reset_mcu()
time.sleep(0.2)

def constrain(x, min_val, max_val):
    '''
    Constrains value to be within a range.
    '''
    return max(min_val, min(max_val, x))

class Picarx(object):
    CONFIG = '/opt/picar-x/picar-x.conf'

    DEFAULT_LINE_REF = [1000, 1000, 1000]
    DEFAULT_CLIFF_REF = [500, 500, 500]

    DIR_MIN = -30
    DIR_MAX = 30
    CAM_PAN_MIN = -90
    CAM_PAN_MAX = 90
    CAM_TILT_MIN = -35
    CAM_TILT_MAX = 65

    PERIOD = 4095
    PRESCALER = 10
    TIMEOUT = 0.02

    # servo_pins: camera_pan_servo, camera_tilt_servo, direction_servo
    # motor_pins: left_swicth, right_swicth, left_pwm, right_pwm
    # grayscale_pins: 3 adc channels
    # ultrasonic_pins: tring, echo2
    # config: path of config file
    def __init__(self, 
                servo_pins:list=['P0', 'P1', 'P2'], 
                motor_pins:list=['D4', 'D5', 'P12', 'P13'],
                grayscale_pins:list=['A0', 'A1', 'A2'],
                ultrasonic_pins:list=['D2','D3'],
                config:str=CONFIG,
                ):

        # --------- config_flie ---------
        self.config_flie = fileDB(config, 774, os.getlogin())

        # --------- servos init ---------
        self.cam_pan = Servo(servo_pins[0])
        self.cam_tilt = Servo(servo_pins[1])   
        self.dir_servo_pin = Servo(servo_pins[2])
        # get calibration values
        self.dir_cali_val = float(self.config_flie.get("picarx_dir_servo", default_value=0))
        self.cam_pan_cali_val = float(self.config_flie.get("picarx_cam_pan_servo", default_value=0))
        self.cam_tilt_cali_val = float(self.config_flie.get("picarx_cam_tilt_servo", default_value=0))
        # set servos to init angle
        self.dir_servo_pin.angle(self.dir_cali_val)
        self.cam_pan.angle(self.cam_pan_cali_val)
        self.cam_tilt.angle(self.cam_tilt_cali_val)

        # --------- motors init ---------
        self.left_rear_dir_pin = Pin(motor_pins[0])
        self.right_rear_dir_pin = Pin(motor_pins[1])
        self.left_rear_pwm_pin = PWM(motor_pins[2])
        self.right_rear_pwm_pin = PWM(motor_pins[3])
        self.motor_direction_pins = [self.left_rear_dir_pin, self.right_rear_dir_pin]
        self.motor_speed_pins = [self.left_rear_pwm_pin, self.right_rear_pwm_pin]
        # get calibration values
        self.cali_dir_value = self.config_flie.get("picarx_dir_motor", default_value="[1, 1]")
        self.cali_dir_value = [int(i.strip()) for i in self.cali_dir_value.strip().strip("[]").split(",")]
        self.cali_speed_value = [0, 0]
        self.dir_current_angle = 0
        # init pwm
        for pin in self.motor_speed_pins:
            pin.period(self.PERIOD)
            pin.prescaler(self.PRESCALER)

        # --------- grayscale module init ---------
        adc0, adc1, adc2 = [ADC(pin) for pin in grayscale_pins]
        self.grayscale = Grayscale_Module(adc0, adc1, adc2, reference=None)
        # get reference
        self.line_reference = self.config_flie.get("line_reference", default_value=str(self.DEFAULT_LINE_REF))
        self.line_reference = [float(i) for i in self.line_reference.strip().strip('[]').split(',')]
        self.cliff_reference = self.config_flie.get("cliff_reference", default_value=str(self.DEFAULT_CLIFF_REF))
        self.cliff_reference = [float(i) for i in self.cliff_reference.strip().strip('[]').split(',')]
        # transfer reference
        self.grayscale.reference(self.line_reference)

        # --------- ultrasonic init ---------
        tring, echo= ultrasonic_pins
        self.ultrasonic = Ultrasonic(Pin(tring), Pin(echo))

        # Make sure the motors stop
        atexit.register(self.stop)
        
    def set_motor_speed(self, motor, speed):
        ''' set motor speed
        
        param motor: motor index, 1 means left motor, 2 means right motor
        type motor: int
        param speed: speed
        type speed: int      
        '''
        speed = constrain(speed, -100, 100)
        motor -= 1
        if speed >= 0:
            direction = 1 * self.cali_dir_value[motor]
        elif speed < 0:
            direction = -1 * self.cali_dir_value[motor]
        speed = abs(speed)
        # Commented this out
        # if speed != 0:
        #     speed = int(speed /2 ) + 50
        speed = speed - self.cali_speed_value[motor]
        if direction < 0:
            self.motor_direction_pins[motor].high()
            self.motor_speed_pins[motor].pulse_width_percent(speed)
        else:
            self.motor_direction_pins[motor].low()
            self.motor_speed_pins[motor].pulse_width_percent(speed)

    def motor_speed_calibration(self, value):
        self.cali_speed_value = value
        if value < 0:
            self.cali_speed_value[0] = 0
            self.cali_speed_value[1] = abs(self.cali_speed_value)
        else:
            self.cali_speed_value[0] = abs(self.cali_speed_value)
            self.cali_speed_value[1] = 0

    def motor_direction_calibrate(self, motor, value):
        ''' set motor direction calibration value
        
        param motor: motor index, 1 means left motor, 2 means right motor
        type motor: int
        param value: speed
        type value: int
        '''      
        motor -= 1
        if value == 1:
            self.cali_dir_value[motor] = 1
        elif value == -1:
            self.cali_dir_value[motor] = -1
        self.config_flie.set("picarx_dir_motor", self.cali_dir_value)

    def dir_servo_calibrate(self, value):
        self.dir_cali_val = value
        self.config_flie.set("picarx_dir_servo", "%s"%value)
        self.dir_servo_pin.angle(value)

    def set_dir_servo_angle(self, value):
        self.dir_current_angle = constrain(value, self.DIR_MIN, self.DIR_MAX)
        angle_value  = self.dir_current_angle + self.dir_cali_val
        self.dir_servo_pin.angle(angle_value)

    def cam_pan_servo_calibrate(self, value):
        self.cam_pan_cali_val = value
        self.config_flie.set("picarx_cam_pan_servo", "%s"%value)
        self.cam_pan.angle(value)

    def cam_tilt_servo_calibrate(self, value):
        self.cam_tilt_cali_val = value
        self.config_flie.set("picarx_cam_tilt_servo", "%s"%value)
        self.cam_tilt.angle(value)

    def set_cam_pan_angle(self, value):
        value = constrain(value, self.CAM_PAN_MIN, self.CAM_PAN_MAX)
        self.cam_pan.angle(-1*(value + -1*self.cam_pan_cali_val))

    def set_cam_tilt_angle(self,value):
        value = constrain(value, self.CAM_TILT_MIN, self.CAM_TILT_MAX)
        self.cam_tilt.angle(-1*(value + -1*self.cam_tilt_cali_val))

    def set_power(self, speed):
        self.set_motor_speed(1, speed)
        self.set_motor_speed(2, speed)
    
    def ackermann_approx(self, angle, ell=1, w=1):
        inner = math.atan2(2 * ell * math.sin(angle), 2 * ell * math.cos(angle) - w * math.sin(angle))
        outer = math.atan2(2 * ell * math.sin(angle), 2 * ell * math.cos(angle) + w * math.sin(angle))
        return inner, outer
        

    def backward(self, speed):
        current_angle = self.dir_current_angle
        if current_angle != 0:
            abs_current_angle = abs(current_angle)
            if abs_current_angle > self.DIR_MAX:
                abs_current_angle = self.DIR_MAX
            power_scale = (100 - abs_current_angle) / 100.0 
            inner, outer = self.ackermann_approx(abs_current_angle)
            if (current_angle / abs_current_angle) > 0:
                logging.debug(f"Backward: Angle > 0, Speed: {speed}, power_scale: {power_scale}")
                # OG
                self.set_motor_speed(1, -1*speed)
                self.set_motor_speed(2, speed * power_scale)
                # Updated
                # self.set_motor_speed(1, -1*speed * inner)
                # self.set_motor_speed(2, speed * outer)
            else:
                logging.debug(f"Backward: Angle < 0, Speed: {speed}, power_scale: {power_scale}")
                # OG
                self.set_motor_speed(1, -1*speed * power_scale)
                self.set_motor_speed(2, speed )
                # Updated
                # self.set_motor_speed(1, -1*speed * outer)
                # self.set_motor_speed(2, speed * inner)
        else:
            logging.debug(f"Backward: Angle == 0, Speed: {speed}")
            self.set_motor_speed(1, -1*speed)
            self.set_motor_speed(2, speed)  

    def forward(self, speed):
        current_angle = self.dir_current_angle
        if current_angle != 0:
            abs_current_angle = abs(current_angle)
            if abs_current_angle > self.DIR_MAX:
                abs_current_angle = self.DIR_MAX
            power_scale = (100 - abs_current_angle) / 100.0
            if (current_angle / abs_current_angle) > 0:
                logging.debug(f"Forward: Angle > 0, Speed: {speed}, power_scale: {power_scale}")
                self.set_motor_speed(1, 1*speed * power_scale)
                self.set_motor_speed(2, -speed) 
            else:
                logging.debug(f"Forward: Angle < 0, Speed: {speed}, power_scale: {power_scale}")
                self.set_motor_speed(1, speed)
                self.set_motor_speed(2, -1*speed * power_scale)
        else:
            logging.debug(f"Backward: Angle == 0, Speed: {speed}")
            self.set_motor_speed(1, speed)
            self.set_motor_speed(2, -1*speed)                  

    def stop(self):
        '''
        Execute twice to make sure it stops
        '''
        for _ in range(2):
            self.motor_speed_pins[0].pulse_width_percent(0)
            self.motor_speed_pins[1].pulse_width_percent(0)
            time.sleep(0.002)

    def get_distance(self):
        return self.ultrasonic.read()

    def set_grayscale_reference(self, value):
        if isinstance(value, list) and len(value) == 3:
            self.line_reference = value
            self.grayscale.reference(self.line_reference)
            self.config_flie.set("line_reference", self.line_reference)
        else:
            raise ValueError("grayscale reference must be a 1*3 list")

    def get_grayscale_data(self):
        return list.copy(self.grayscale.read())

    def get_line_status(self,gm_val_list):
        return self.grayscale.read_status(gm_val_list)

    def set_line_reference(self, value):
        self.set_grayscale_reference(value)

    def get_cliff_status(self,gm_val_list):
        for i in range(0,3):
            if gm_val_list[i]<=self.cliff_reference[i]:
                return True
        return False

    def set_cliff_reference(self, value):
        if isinstance(value, list) and len(value) == 3:
            self.cliff_reference = value
            self.config_flie.set("cliff_reference", self.cliff_reference)
        else:
            raise ValueError("grayscale reference must be a 1*3 list")
    

def forward_backward_test(px):
    px.forward(50)
    time.sleep(1)
    px.stop()
    px.backward(50)
    time.sleep(1)
    px.stop()

def parallel_park(px, is_right, angle=25, speed=60, rest_time=1):
    turn_angle = angle
    if not is_right:
        turn_angle *= -1
    # Initial Rotate
    px.set_dir_servo_angle(turn_angle)
    # Start backup
    px.backward(speed)
    time.sleep(rest_time)
    px.stop()
    # Rotate wheel the other way
    px.set_dir_servo_angle(-turn_angle)
    # Start the final backup
    px.backward(speed)
    time.sleep(rest_time)
    px.stop()
    # Straighten wheels
    px.set_dir_servo_angle(0)
    

def three_point_turn(px, is_right, angle=25, speed=50, rest_time=1.5):
    turn_angle = angle
    if not is_right:
        turn_angle *= -1
    px.set_dir_servo_angle(turn_angle)
    px.backward(speed)
    time.sleep(rest_time)
    px.stop()

    px.set_dir_servo_angle(0)
    px.forward(speed)
    time.sleep(rest_time)
    px.stop()

    px.set_dir_servo_angle(turn_angle)
    px.backward(speed)
    time.sleep(rest_time)
    px.stop()

    px.set_dir_servo_angle(0)
    px.forward(speed)
    time.sleep(rest_time)
    px.stop()

def run():
    px = Picarx()
    while(1):
        print("Enter 1, 2, 3, 4, 5, or 6. ")
        usr_in = input()
        print()
        if usr_in == '1':
            print("Forward backward test")
            forward_backward_test(px)
        elif usr_in == '2':
            print("Parallel park going left")
            parallel_park(px, False)
        elif usr_in == '3':
            print("Parallel park going right")
            parallel_park(px, True)
        elif usr_in == '4':
            print("Three point turn going right")
            three_point_turn(px, False)
        elif usr_in == '5':
            print("Three point turn going left")
            three_point_turn(px, True)
        elif usr_in == '6':
            print("\n", "Breaking out of the run.")
            break



#####################################################################
class Grey_Sensing():
    def __init__(self):
        self.greyscale = Grayscale_Module(ADC('A0'), ADC('A1'), ADC('A2'), reference=None)
        

    def read(self):
        return self.greyscale.read()


class Ultra_Sensing():
    def __init__(self):
        self.ultra = Ultrasonic(Pin('D2'), Pin('D3'))

    def read(self):
        return self.ultra.read()
    


#####################################################################
class Grey_Interpreter():
    # Sensitivity is how different light and dark values should be.
    # Polarity is asking the line we are following lighter or darker than surrounding floor (True means lighter)
    def __init__(self, sensitivity=.30, polarity=True):
        self.sensitivity = sensitivity
        self.polar = polarity
        
    
    def find_edge(self, grey_vals):
        edge_detected = False
        max_val = max(grey_vals)
        if sum(grey_vals) == 0:
            norm = grey_vals
        else:
            norm = [v / max_val for v in grey_vals]
            logging.debug(f"Normalized vals {norm}")
            norm_l, norm_m, norm_r = norm
            print(abs(norm_l - norm_m) > self.sensitivity)

        to_return = 0
        if abs(norm_l - norm_m) > self.sensitivity:
            edge_detected = True
            logging.debug("Difference in left")
            # Looking for a light line
            if self.polar:
                logging.debug("Looking for light line")
                # If the left sensor is on the light line
                if norm_l - norm_m > 0:
                    to_return = -(norm_l - norm_m) # This should be negative because the car needs to turn left, which is a - angle
                # If the middle sensor is on the light line, don't change
                else:
                    to_return = 0
            # Looking for a dark line
            else:
                logging.debug("Looking for dark line")
                # If the middle sensor is on the dark line, don't change
                if norm_l - norm_m > 0:
                    to_return = 0
                # If the left sensor is less than the middle sensor (darker), move left
                else:
                    to_return = norm_l - norm_m # This will be negative because we already established above it is not positive
        # If we haven't detected an edge yet or if we have detected an edge and the right edge is further away from the middle
        if not edge_detected or (edge_detected and (norm_r - norm_m) > (norm_l - norm_m)):
            if abs(norm_r - norm_m) > self.sensitivity:
                logging.debug("Difference in right")
                # Looking for a light line
                if self.polar:
                    logging.debug("Looking for light line")
                    # If the right is on the light line
                    if norm_r - norm_m > 0:
                        to_return = norm_r - norm_m
                    # If the center is on the light line, do nothing
                    else:
                        to_return = 0
                # Looking for a dark line
                else:
                    logging.debug("Looking for dark line")
                    # If the right is lighter than the middle, i.e. middle is on the line, do nothing
                    if norm_r - norm_m > 0:
                        to_return = 0
                    # If the right side is darker than the left
                    else:
                        to_return = abs(norm_r - norm_m)
        if abs(norm_l - norm_r) < self.sensitivity:
            logging.debug("left and right grey values are too close")
            to_return = 0
        return to_return


class Ultra_Interpreter():
    def __init__(self, thresh=10):
        self.thresh = thresh

    def find_obstacle(self, dist):
        logging.debug(f"In ultra interpreter: Got {dist}")
        print(type(dist))
        if dist < self.thresh:
            logging.debug("Close object detected")
            # stop the car
            return 0
        else:
            logging.debug("No object detected")
            # don't stop the car
            return 1



####################################################################
class Controller():

    def __init__(self, px, steady_engine, scaling_factor=40, start_engine=50):
        self.scale = scaling_factor
        self.steady_engine = steady_engine
        self.angle = 0
        self.px = px
        # Just get the car going for a bit
        self.px.forward(start_engine)
        time.sleep(0.01)

    def control_loop(self, angle, motor_scale=1):
        logging.debug(f"In control loop: Got angle {angle} and ultra {motor_scale}")
        self.px.set_dir_servo_angle(self.scale * angle)
        self.px.forward(self.steady_engine * motor_scale)


#####################################################################################


if __name__ == "__main__":
    # Week 5
    # Get inputs (sensitivity, polarity, threshold, scale)
    sensitivity = float(input('Enter the sensitivity (default is 0.3) '))
    polarity = input('Enter 1 for a white line and 2 for a dark line ')
    if polarity == '1':
        polarity = True
    else:
        polarity = False
    threshold = input('Enter the threshold value for the ultrasound sensor ')
    scale = input('Enter the scaling factor for the turn angle (default is 40) ')

    grey_sensor_delay = float(input('Enter the delay for the greyscale sensor '))
    ultra_sensor_delay = float(input('Enter the delay for the ultrasonic sensor '))
    grey_inter_delay = float(input('Enter the delay for the greyscale interpreter '))
    ultra_inter_delay = float(input('Enter the delay for the ultrasonic interpreter '))
    cont_delay = float(input('Enter the delay for the controller '))

    # Setup busses and such
    grey_sensor_bus = rossros.Bus()
    grey_sensor = Grey_Sensing()
    ultra_sensor_bus = rossros.Bus()
    ultra_sensor = Ultra_Sensing()

    grey_inter_bus = rossros.Bus()
    grey_interpreter = Grey_Interpreter(sensitivity=sensitivity, polarity=polarity)
    ultra_inter_bus = rossros.Bus()
    ultra_interpreter = Ultra_Interpreter(thresh=threshold)

    cont = Controller(Picarx(), steady_engine=25, scaling_factor=scale, start_engine=50)
    

    grey_sense_prod = rossros.Producer(grey_sensor.read, grey_sensor_bus, delay=grey_sensor_delay, name="Grey Sensing")

    grey_cons_prod = rossros.ConsumerProducer(grey_interpreter.find_edge, grey_sensor_bus, grey_inter_bus, delay=grey_inter_delay, name="Grey Interpreter")



    ultra_sense_prod = rossros.Producer(ultra_sensor.read, ultra_sensor_bus, delay=ultra_sensor_delay, name="Ultra Sensing")

    ultra_cons_prod = rossros.ConsumerProducer(ultra_interpreter.find_obstacle, ultra_sensor_bus, ultra_inter_bus, delay=ultra_inter_delay, name="Ultra Interpreter")


    cont_cons = rossros.Consumer(cont.control_loop, (grey_inter_bus, ultra_inter_bus), delay=cont_delay, name="Controller")

    # Run
    cons_prod_list = [ultra_sense_prod, ultra_cons_prod]
    rossros.runConcurrently(cons_prod_list)
            