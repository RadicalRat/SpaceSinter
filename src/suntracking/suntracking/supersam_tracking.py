import device_interface.arduino_stepper_interface 
import device_interface.power_max_interface 

import matplotlib.pyplot as plt
import cv2
from pypylon import pylon
import time
import numpy as np
import threading

PAUSE_OFFSET = 20 # deg
FOV_V = 60
N_PIXLES_V = 480 
SMALL_GEAR_TEETH = 12
LARGE_GEAR_TEETH = 77
STEPPER_DEG_PER_STEP = 0.018
DRIVER_PULSE_PER_REV = 400
DRIVE_FACTOR = 200/DRIVER_PULSE_PER_REV
DEG_PER_PIXLE = FOV_V/N_PIXLES_V
DEG_PER_STEP = STEPPER_DEG_PER_STEP*DRIVE_FACTOR*(SMALL_GEAR_TEETH/LARGE_GEAR_TEETH)
print('Deg per Step', DEG_PER_STEP)

V_LIMIT_1_REBOUND = 0.1 #deg
DEFAULT_SPEED = 500
DEFAULT_ACCELERATION = 2000

CAMERA_ID = 'usb-DSJ-211216-ZW_UC40M_Audio_01.00.00-video-index0'

class Tracking:
    def __init__(self, horizontal_arduino_serial, vertical_arduino_serial, exposure=-12, enable_camera=False):
        print('Connecting to Tracking Horizontal Arduino')
        self.h_arduino = device_interface.arduino_stepper_interface.Arduino(horizontal_arduino_serial)
        print('Connecting to Tracking Vertical Arduino')
        self.v_arduino = device_interface.arduino_stepper_interface.Arduino(vertical_arduino_serial)
        # print('Connecting to Shade Arduino')
        # self.shade_arduino = device_interface.arduino_stepper_interface.Arduino(shade_arduino_serial)
        
        # if enable_camera:
        #     try:
        #         self.cap.release()
        #         cv2.destroyAllWindows()
        #     except:
        #         pass

        #     width = 1920
        #     height = 1080
        #     self.cap = cv2.VideoCapture('/dev/v4l/by-id/'+ CAMERA_ID)

        #     self.cap.set(cv2.CAP_PROP_EXPOSURE, exposure) 
        #     self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        #     if not (self.cap.isOpened()):
        #         print("Could not open video device")

        #     ret, frame = self.cap.read() 
        #     # self.cap.release()
        #     plt.figure(figsize = (10,10))
        #     plt.imshow(frame)

        #     self.set_up_detector()
        #     self.tracking_on = False
        #     self.current_PAUSE_OFFSET = 0
        # Connect to camera
        if enable_camera:
            try:
                self.cap.release()
                cv2.destroyAllWindows()
            except:
                pass

            width = 1920
            height = 1080
            # self.cap = cv2.VideoCapture('/dev/v4l/by-id/'+ CAMERA_ID)
            # self.cap = cv2.VideoCapture(1)
            self.cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)

            self.cap.set(cv2.CAP_PROP_EXPOSURE, exposure) 
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

            if not (self.cap.isOpened()):
                print("Could not open video device")

            ret, frame = self.cap.read() 
            # self.cap.release()
            plt.figure(figsize = (10,10))
            plt.imshow(frame)

            self.set_up_detector()
            self.tracking_on = False
            self.current_PAUSE_OFFSET = 0


        print('Connected to Tracking')
        
    def move_h_cw(self, degs, speed=DEFAULT_SPEED, accel=DEFAULT_ACCELERATION):
        self.move_h(abs(degs), speed, accel)
        
    def move_h_ccw(self, degs, speed=DEFAULT_SPEED, accel=DEFAULT_ACCELERATION):
        self.move_h(-abs(degs), speed, accel)
    
    def move_h(self, degs, speed=DEFAULT_SPEED, accel=DEFAULT_ACCELERATION):
        self.h_arduino.set_acceleration(accel)
        self.h_arduino.set_speed(speed)
        self.h_arduino.move(degs/DEG_PER_STEP)
        
    def move_v_up(self, degs, speed=DEFAULT_SPEED, accel=DEFAULT_ACCELERATION):
        self.move_v(abs(degs), speed=speed, accel=accel)
        
    def move_v_down(self, degs, speed=DEFAULT_SPEED, accel=DEFAULT_ACCELERATION):
        self.move_v(-abs(degs), speed=speed, accel=accel)
    
    def move_v(self, degs, speed=DEFAULT_SPEED, accel=DEFAULT_ACCELERATION):
        self.v_arduino.set_acceleration(accel)
        self.v_arduino.set_speed(speed)
        self.v_arduino.move(degs/DEG_PER_STEP)   
        
    def set_up_detector(self):
        # SimpleBlobDetector parameters.
        params = cv2.SimpleBlobDetector_Params()

        params.filterByColor = False

        # Change thresholds
        params.minThreshold = 200
        params.maxThreshold = 255


        # Filter by Area.
        params.filterByArea = True
        params.minArea = 50

        # Filter by Circularity
        params.filterByCircularity = True
#         params.minCircularity = 0.75
        params.minCircularity = 0.5

        # Filter by Convexity
        params.filterByConvexity = True
#         params.minConvexity = 0.87
        params.minConvexity = 0.5

        # Filter by Inertia
        params.filterByInertia = False
        params.minInertiaRatio = 0.01

        # Create a detector with the parameters
        self.detector = cv2.SimpleBlobDetector_create(params)
        
    def get_blob(self, output=False):
        # self.cap = cv2.VideoCapture(0)
        ret, frame = self.cap.read()
        ret, frame = self.cap.read()
        # self.cap.release()

        # Read image
        im = frame

        # Detect blobs.
        keypoints = self.detector.detect(im)

        # Draw detected blobs as red circles.
        # cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS ensures
        # the size of the circle corresponds to the size of blob

        im_with_keypoints = cv2.drawKeypoints(im, keypoints, np.array([]), (255,0,0), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

        # Show blobs
        if output:
            plt.figure(figsize = (10,10))
            plt.imshow(im_with_keypoints)
            if len(keypoints) >0:
                print(keypoints[0].pt)

        if len(keypoints) != 1: 
            if output:
                print("Error Keypoints list not equal to 1")
            return None

        return keypoints[0].pt

    def track_sun(self, stop, set_point = [320, 250], output=False):
        if set_point == 'sim':
            print('Tracking Sim')
            # Run sim sun
            while(True):
                time.sleep(0.1)
                if stop():
                    return
        self.v_arduino.set_speed(400)
        self.v_arduino.set_acceleration(2000)
        self.h_arduino.set_speed(400)
        self.h_arduino.set_acceleration(2000)

        damped_steps_per_pixle = 5
        if self.get_blob() is not None:
            print('Sun Found Starting Tracking')
        else:
            print('Sun Not Found')
            return None
            
        while(True):
            if stop():
                break
            time.sleep(0.05) # 0.02
            sun_loc = self.get_blob()
            if sun_loc is None: continue
            h_error = sun_loc[0] - set_point[0]
            v_error = sun_loc[1] - set_point[1]
            if output:
                print("Error H: {:.2f} Error V: {:.2f}".format(h_error, v_error), end='\r')
            if abs(h_error) > 2.5: #0.3
                self.h_arduino.move(-h_error*damped_steps_per_pixle)
        #         continue
            if abs(v_error) > 2.5: #0.3
                self.v_arduino.move(v_error*damped_steps_per_pixle)
            
    def start_tracking_thread(self, set_point = [360,170]):
        self.current_PAUSE_OFFSET = 0
        if not self.tracking_on:
            self.stop_tracking = False
            self.t1 = threading.Thread(target = self.track_sun, args =(lambda : self.stop_tracking, set_point))
            self.t1.start()
            self.tracking_on = True
            
    def stop_tracking_thread(self):
        self.stop_tracking = True
        self.t1.join()
        self.tracking_on = False
        
#     def sun_on(self, set_point= [360,170]):
#         self.move_h_cw(self.current_PAUSE_OFFSET, speed=1500, accel=1000)
#         self.start_tracking_thread(set_point)
#         self.current_PAUSE_OFFSET = 0
#         time.sleep(3)
        
#     def sun_off(self):
#         if self.tracking_on:
#             self.stop_tracking_thread()
#             self.move_h_ccw(PAUSE_OFFSET, speed=1500, accel=1000)
#             self.current_PAUSE_OFFSET = PAUSE_OFFSET
        






