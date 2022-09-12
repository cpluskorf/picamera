#!/usr/bin/env python3
import subprocess
import shlex
import math
import time
import RPi.GPIO as GPIO
from picamera import PiCamera
import sys
import numpy as np
import cv2
import os
import json
from stereovision.calibration import StereoCalibrator
from stereovision.calibration import StereoCalibration
from stereovision.exceptions import ChessboardNotFoundError

# Global variables preset
total_photos = 0
#photo_width = 640
#photo_height = 240
#img_width = 320
#img_height = 240
image_format = ".png"

# Chessboard parameters
rows = 6
columns = 9
square_size = 2.3

# Camera settimgs
cam_width = 1920
cam_height = 1080

# Final image capture settings
scale_ratio = 1

# Camera resolution height must be dividable by 16, and width by 32
cam_width = int((cam_width+31)/32)*32
cam_height = int((cam_height+15)/16)*16

# Buffer for captured image settings
img_width = int (cam_width * scale_ratio)
img_height = int (cam_height * scale_ratio)
capture = np.zeros((img_height, img_width, 4), dtype=np.uint8)
image_size = (img_width, img_height)


class Calibration:
    """
    # mvvelo = 80.0
    # mvacc = 100.0
    # mvtime = 0.0
    # base = 0
    # tool = 1
    # absolute = 0
    # relative = 1
    """

    set_wait = "rosparam set /xarm/wait_for_finish true"
    enable_motors = "rosservice call /xarm/motion_ctrl 8 1"
    set_ready_mode = "rosservice call /xarm/set_mode 0"
    set_ready_state = "rosservice call /xarm/set_state 0"
    go_home_cmd = "rosservice call /xarm/move_joint [-0.6655980944633484,-0.014080026187002659,-1.0995478630065918,1.111729621887207,-0.11332858353853226] 0.1 1 0 0"
    zero_position = "rosservice call /xarm/go_home [] 0.10 2.5 0 0"
    move_line = "rosservice call /xarm/move_line_aa"
    pos_1 = "rosservice call /xarm/move_joint [-0.6655980944633484,-0.036307018250226974,-1.1792691946029663,1.215576171875,-0.11332876235246658] 0.1 1 0 0"


    def __init__(self) -> None:
        self.positions = []

    def go_home(self):
        subprocess.Popen(shlex.split(self.go_home_cmd), stdout=subprocess.DEVNULL).wait()

    def go_to_zero_postion(self):
        subprocess.Popen(shlex.split(self.zero_position), stdout=subprocess.DEVNULL).wait()

    def move_relative(
        self,
        axis: str = "x",
        amount: float = 0,
        velocity: float = 80.0,
        acceleration: float = 100.0,
        coord_sys: int = 0,
        movement: int = 1) -> None:

        axis_idx = None

        if axis == "x":
            axis_idx = 0
        elif axis == "y":
            axis_idx = 1
        elif axis == "z":
            axis_idx = 2
        elif axis == "rx":
            amount = math.radians(amount)
            axis_idx = 3
        elif axis == "ry":
            amount = math.radians(amount)
            axis_idx = 4
        elif axis == "rz":
            amount = math.radians(amount)
            axis_idx = 5
        else:
            raise ValueError
        

        new_pose = [0,0,0,0,0,0]
        new_pose[axis_idx] = amount

        new_position = {
            "pose": "[" + "".join(str(i) + "," for i in new_pose) + "]",
            "mvvelo": str(velocity),
            "mvacc": str(acceleration),
            "mvtime": str(0.0),
            "coord": str(coord_sys),
            "relative": str(movement)
        }

        print(f"moving to new position {new_position}")
        subprocess.Popen(
                shlex.split(" ".join([
                    self.move_line,
                    new_position.get("pose"),
                    new_position.get("mvvelo"),
                    new_position.get("mvacc"),
                    new_position.get("mvtime"),
                    new_position.get("coord"),
                    new_position.get("relative")])), stdout=subprocess.DEVNULL).wait()
        print("new position reached")

    def get_ready(self):
        subprocess.Popen(shlex.split(self.set_wait), stdout=subprocess.DEVNULL).wait()
        subprocess.Popen(shlex.split(self.enable_motors), stdout=subprocess.DEVNULL).wait()
        subprocess.Popen(shlex.split(self.set_ready_mode), stdout=subprocess.DEVNULL).wait()
        subprocess.Popen(shlex.split(self.set_ready_state), stdout=subprocess.DEVNULL).wait()


def main():
    camera = PiCamera()
    camera.resolution = (cam_width, cam_height)
    camera.framerate = 20
    camera.hflip = True

    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    GPIO.setup(4, GPIO.OUT, initial=GPIO.LOW)

    # initializing calibration
    calibration = Calibration()
    calibration.get_ready()
    calibration.go_home()
    print("moving to home position")
    
    # all positions to move to in order
    positions = [
        ("z", -20), ("x", 20), ("y", 20), ("rz", -10), ("y", -20), ("x", -20),
        ("z", 5), ("x", 15), ("y", 15), ("rz", 10), ("y", -15), ("x", -15),
        ("z", 5), ("x", 5), ("y", 5), ("rz", -10), ("y", -5), ("x", -5),
        ("z", 5), ("x", 25), ("y", 25), ("rz", 10), ("y", -25), ("x", -25),
        ("z", 5), ("x", 10), ("y", 10), ("rz", -0), ("y", -10), ("x", -10),
        ]
    

    img_cnt = 0
    calibrating = True
    start = time.time()
    while calibrating:
        
        # move to new position
        try:
            calibration.move_relative(positions[img_cnt][0], positions[img_cnt][1])
        except IndexError:
            calibrating = False
            global total_photos
            total_photos = img_cnt
            continue
        except subprocess.CalledProcessError as e:
            print(e)
            sys.exit(0)
        
        print(f"taking picture number {img_cnt + 1}")

        GPIO.output(4, GPIO.HIGH)
        file_name_l = "/home/pi/calibration/pairs/left_" + str(img_cnt + 1).zfill(2) + image_format

        camera.capture(file_name_l)
        GPIO.output(4, GPIO.LOW)
        file_name_r = "/home/pi/calibration/pairs/right_" + str(img_cnt + 1).zfill(2) + image_format
        camera.capture(file_name_r)
        
        img_cnt += 1

    print('elapsed time:', round((time.time()-start), 2), " seconds")
    print("taking pictures done.")
    print("moving to home position...")
    #calibration.go_to_zero_postion()
    
    main_calib()


def main_calib():
    calibrator = StereoCalibrator(rows, columns, square_size, image_size)
    photo_counter = 0
    print ('Start main calibration')

    while photo_counter != total_photos:
      photo_counter = photo_counter + 1
      print ('Import pair No ' + str(photo_counter))
      leftName = './pairs/left_'+str(photo_counter).zfill(2)+ image_format
      rightName = './pairs/right_'+str(photo_counter).zfill(2)+image_format
      if os.path.isfile(leftName) and os.path.isfile(rightName):
          imgLeft = cv2.imread(leftName,1)
          imgRight = cv2.imread(rightName,1)
          try:
            calibrator._get_corners(imgLeft)
            calibrator._get_corners(imgRight)
          except ChessboardNotFoundError as error:
            print (error)
            print ("Pair No "+ str(photo_counter) + " ignored")
          else:
            calibrator.add_corners((imgLeft, imgRight), False)
            
    print ('End cycle')


    print ('Starting calibration... It can take several minutes!')
    calibration = calibrator.calibrate_cameras()
    calibration.export('calib_result')
    print ('Calibration complete!')


    # Lets rectify and show last pair after  calibration
    calibration = StereoCalibration(input_folder='calib_result')
    rectified_pair = calibration.rectify((imgLeft, imgRight))

    cv2.imshow('Left CALIBRATED', rectified_pair[0])
    cv2.imshow('Right CALIBRATED', rectified_pair[1])
    cv2.imwrite("rectifyed_left.jpg",rectified_pair[0])
    cv2.imwrite("rectifyed_right.jpg",rectified_pair[1])
    cv2.waitKey(0)

if __name__ == '__main__':
    main()
