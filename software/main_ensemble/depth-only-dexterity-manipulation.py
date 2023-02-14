#!/usr/bin/env python

import device_patches       # Device specific patches for Jetson Nano (needs to be before importing cv2)

import cv2
import os
import time
import sys, getopt
import numpy as np
from edge_impulse_linux.image import ImageImpulseRunner

import depthai as dai
from calc import HostSpatialsCalc
from utility import *
import numpy as np
import math

import time
from signal import pause
from buildhat import Motor

motor_gripper = Motor('A')
motor_gripper_control = Motor('B')
motor_body_control = Motor('C')
motor_body_movement = Motor('D')

print("Position Motor Gripper Port A", motor_gripper.get_aposition())
print("Position Motor Gripper Control Port B", motor_gripper.get_aposition())
print("Position Motor Body Control Port C", motor_gripper.get_aposition())
print("Position Motor Body Movement Port D", motor_gripper.get_aposition())

# Start defining a pipeline

pipeline = dai.Pipeline()

# Define sources and outputs
monoLeft = pipeline.create(dai.node.MonoCamera)
monoRight = pipeline.create(dai.node.MonoCamera)
stereo = pipeline.create(dai.node.StereoDepth)

# Properties
monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
monoLeft.setBoardSocket(dai.CameraBoardSocket.LEFT)
monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
monoRight.setBoardSocket(dai.CameraBoardSocket.RIGHT)

stereo.initialConfig.setConfidenceThreshold(255)
stereo.setLeftRightCheck(True)
stereo.setSubpixel(False)

# Linking
monoLeft.out.link(stereo.left)
monoRight.out.link(stereo.right)

xoutDepth = pipeline.create(dai.node.XLinkOut)
xoutDepth.setStreamName("depth")
stereo.depth.link(xoutDepth.input)
i = 0
xoutDepth = pipeline.create(dai.node.XLinkOut)
xoutDepth.setStreamName("disp")
stereo.disparity.link(xoutDepth.input)

    
def turn():
    TurnDegrees1 = 72
    x = (TurnDegrees1/22.5)
    motor_body_movement.run_for_rotations(x, speed=-60)
    motor_body_control.run_to_position(-30, speed=20, direction='clockwise')
    motor_gripper.run_to_position(150, speed=30, direction='anticlockwise')

def moveToLocation2D(x, y):
    #ensure that the oak-d is pointing in the same direction as the front arm of the robotic arm.
    height = 300 - y
    distanceFromCentre = x - 150 #adjusted coordinates according to opencv coordinate system which starts at 0,0 from top left
    tan_inv = math.atan(height/distanceFromCentre)
    tan_inv_deg = 90 - math.degrees(tan_inv)
    convert_inv_deg = (tan_inv_deg/45)
    #this command aligns the robotic arm in the vectorial direction of the fruit
    if x-150>150:
        motor_body_movement.run_for_rotations(convert_inv_deg, speed=60)
    else:
        motor_body_movement.run_for_rotations(convert_inv_deg, speed=-60)

def moveToLocation3D(x, y, z):
    #used to move to exact position in the z plane. 
    distanceToCentroid = math.sqrt((x - 0)**2 + (y - 150)**2)
    tan_inv = math.atan(z/distanceToCentroid)
    tan_inv_deg = 90 - math.degrees(tan_inv)
    motor_body_control.run_to_position((20 - tan_inv_deg), speed=50, direction='anticlockwise')
    
def otherMovements():
    motor_body_movement.run_for_rotations(x, speed=-60)
    motor_gripper_control.run_to_position(-4, speed=20, direction='anticlockwise')
    motor_body_control.run_to_position(-30, speed=20, direction='clockwise')   

    
def stopMovement():
    motor_body_movement.stop()
    motor_body_control.stop()
    motor_gripper_control.stop()
    motor_gripper.stop()

runner = None
# if you don't want to see a video preview, set this to False
show_camera = True



def help():
    print('python classify-video.py <path_to_model.eim> <path_to_video.mp4>')

def main(argv):
    incCounter = 0
    run = False
    try:
        opts, args = getopt.getopt(argv, "h", ["--help"])
    except getopt.GetoptError:
        help()
        sys.exit(2)

    for opt, arg in opts:
        if opt in ('-h', '--help'):
            help()
            sys.exit()


    model = args[0]

    dir_path = os.path.dirname(os.path.realpath(__file__))
    modelfile = os.path.join(dir_path, model)

    print('MODEL: ' + modelfile)

    with ImageImpulseRunner(modelfile) as runner:
        try:
            model_info = runner.init()
            print('Loaded runner for "' + model_info['project']['owner'] + ' / ' + model_info['project']['name'] + '"')
            labels = model_info['model_parameters']['labels']

            sec = 0
            start_time = time.time()

            with dai.Device(pipeline) as device:
                # Output queue will be used to get the depth frames from the outputs defined above
                depthQueue = device.getOutputQueue(name="depth")
                dispQ = device.getOutputQueue(name="disp")

                hostSpatials = HostSpatialsCalc(device)
                y = 200
                x = 300
                step = 3
                delta = 5
                hostSpatials.setDeltaRoi(delta)

                while True:
                    depthFrame = depthQueue.get().getFrame()
                    # Calculate spatial coordiantes from depth frame
                    spatials, centroid = hostSpatials.calc_spatials(depthFrame, (x,y)) # centroid == x/y in our case

                    # Get disparity frame for nicer depth visualization
                    disp = dispQ.get().getFrame()
                    disp = (disp * (255 / stereo.initialConfig.getMaxDisparity())).astype(np.uint8)
                    disp = cv2.applyColorMap(disp, cv2.COLORMAP_JET)                    

                    img = disp

                    if img is not None:
                        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

                        # imread returns images in BGR format, so we need to convert to RGB

                        # get_features_from_image also takes a crop direction arguments in case you don't have square images
                        features, cropped = runner.get_features_from_image(img)

                        # the image will be resized and cropped, save a copy of the picture here
                        # so you can see what's being passed into the classifier

                        res = runner.classify(features)
                        

                        if "classification" in res["result"].keys():
                            print('Result (%d ms.) ' % (res['timing']['dsp'] + res['timing']['classification']), end='')
                            for label in labels:
                                score = res['result']['classification'][label]
                                print('%s: %.2f\t' % (label, score), end='')
                            print('', flush=True)

                        elif "bounding_boxes" in res["result"].keys():
                            print('Found %d bounding boxes (%d ms.)' % (len(res["result"]["bounding_boxes"]), res['timing']['dsp'] + res['timing']['classification']))
                            for bb in res["result"]["bounding_boxes"]:
                                xmid = int(bb['x'] + (bb['width']/2))
                                ymid = int(bb['y'] + (bb['height']/2))
                                #img = cv2.rectangle(cropped, (bb['x'], bb['y']), (bb['x'] + bb['width'], bb['y'] + bb['height']), (255, 0, 0), 1)
                                if ymid>45:
                                    img = cv2.circle(cropped, (xmid, ymid), 4, (255,255,255), 2)
                                    print('\t%s (%.2f): x=%d y=%d w=%d h=%d' % (bb['label'], bb['value'], bb['x'], bb['y'], bb['width'], bb['height']))

                            if len(res["result"]["bounding_boxes"]) == 0:
                                print("none")
                                stopMovement()
                            else:
                                for bb in res["result"]["bounding_boxes"]:
                                    if bb['value'] > 0.75:
                                        incCounter += 1
                                        x = int(bb['x'] + (bb['width']/2))
                                        y = int(bb['y'] + (bb['height']/2))
                                        z = spatials['z']/1000                                        
                                        print(incCounter)

                                if incCounter == 70: #only when the object is continuously detected for 3 seconds straight, the arm moves.
                                    run = True
                                    print(run)    
                                                
                                if run == True:
                                    incCounter = 0                                
                                    print("working")
                                    pickup() 
                                    turn()
                                    moveToLocation2D(x,y)
                                    moveToLocation3D(x,y,z)
                                    otherMovements()
                                    returnback()
                                                               

                        if (show_camera):
                            cropped = cv2.resize(cv2.cvtColor(cropped, cv2.COLOR_BGR2RGB), (640, 480)) 
                            cv2.imshow('edgeimpulse', cropped)
                    if cv2.waitKey(1) == ord('q'):
                        break

                    sec = time.time() - start_time
                    sec = round(sec, 2)
                    print("Getting frame at: %.2f sec" % sec)
                        
        finally:
            if (runner):
                runner.stop()

if __name__ == "__main__":
   main(sys.argv[1:])
