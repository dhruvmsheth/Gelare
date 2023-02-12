#!/usr/bin/env python

import device_patches       # Device specific patches for Jetson Nano (needs to be before importing cv2)

import cv2
import os
import time
import sys, getopt
import numpy as np
from edge_impulse_linux.image import ImageImpulseRunner

import cv2
import depthai as dai
import numpy as np


import cv2
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

def indexMovement():
    TurnDegrees1 = 90
    x = (TurnDegrees1/22.5)
    TurnDegrees2 = 70
    x2 = (TurnDegrees2/22.5)    
    motor_body_control.run_to_position(70, speed=50, direction='anticlockwise')
    motor_gripper_control.run_to_position(75, speed=30, direction='clockwise')
    motor_body_movement.run_for_rotations(x, speed=60)
    motor_gripper_control.run_to_position(-4, speed=20, direction='anticlockwise')
    motor_body_control.run_to_position(-30, speed=20, direction='clockwise')
    time.sleep(0.25)
    motor_gripper.run_to_position(110, speed=30, direction='anticlockwise')
    motor_gripper_control.run_to_position(75, speed=30, direction='clockwise')
    motor_body_control.run_to_position(70, speed=50, direction='anticlockwise')
    motor_gripper_control.run_to_position(75, speed=30, direction='clockwise')
    time.sleep(0.25)
    #motor_gripper_control.run_to_position(-4, speed=20, direction='anticlockwise')
    motor_body_movement.run_for_rotations(x2, speed=-40)
    motor_body_control.run_to_position(-14, speed=20, direction='clockwise')
    motor_gripper_control.run_to_position(-4, speed=20, direction='anticlockwise')
    motor_gripper.run_to_position(-30, speed=30, direction='clockwise')

def thumbMovement():
    TurnDegrees1 = 125
    x = (TurnDegrees1/22.5)
    TurnDegrees2 = 105
    x2 = (TurnDegrees2/22.5)   
    motor_body_control.run_to_position(70, speed=50, direction='anticlockwise')
    motor_gripper_control.run_to_position(75, speed=30, direction='clockwise')
    motor_body_movement.run_for_rotations(x, speed=60)
    motor_gripper_control.run_to_position(-4, speed=20, direction='anticlockwise')
    motor_body_control.run_to_position(-30, speed=20, direction='clockwise')
    time.sleep(0.25)
    motor_gripper.run_to_position(110, speed=30, direction='anticlockwise')
    motor_gripper_control.run_to_position(75, speed=30, direction='clockwise')
    motor_body_control.run_to_position(70, speed=50, direction='anticlockwise')
    motor_gripper_control.run_to_position(75, speed=30, direction='clockwise')
    time.sleep(0.25)
    #motor_gripper_control.run_to_position(-4, speed=20, direction='anticlockwise')
    motor_body_movement.run_for_rotations(x2, speed=-40)
    motor_body_control.run_to_position(-14, speed=20, direction='clockwise')
    motor_gripper_control.run_to_position(-4, speed=20, direction='anticlockwise')
    motor_gripper.run_to_position(-30, speed=30, direction='clockwise')

def palmMovement():
    TurnDegrees1 = 195
    x = (TurnDegrees1/22.5)
    TurnDegrees2 = 120
    x2 = (TurnDegrees2/22.5)     
    motor_body_control.run_to_position(70, speed=50, direction='anticlockwise')
    motor_gripper_control.run_to_position(75, speed=30, direction='clockwise')
    motor_body_movement.run_for_rotations(x, speed=70)
    motor_gripper_control.run_to_position(-4, speed=20, direction='anticlockwise')
    motor_body_control.run_to_position(-30, speed=20, direction='clockwise')
    time.sleep(0.25)
    motor_gripper.run_to_position(110, speed=30, direction='anticlockwise')
    motor_gripper_control.run_to_position(75, speed=30, direction='clockwise')
    motor_body_control.run_to_position(70, speed=50, direction='anticlockwise')
    motor_gripper_control.run_to_position(75, speed=30, direction='clockwise')
    time.sleep(0.25)
    #motor_gripper_control.run_to_position(-4, speed=20, direction='anticlockwise')
    motor_body_movement.run_for_rotations(x2, speed=-40)
    motor_body_control.run_to_position(-14, speed=20, direction='clockwise')
    motor_gripper_control.run_to_position(-4, speed=20, direction='anticlockwise')
    motor_gripper.run_to_position(-30, speed=30, direction='clockwise')


    
def stopMovement():
    motor_body_movement.stop()
    motor_body_control.stop()
    motor_gripper_control.stop()
    motor_gripper.stop()

    
runner = None
# if you don't want to see a camera preview, set this to False
show_camera = True
if (sys.platform == 'linux' and not os.environ.get('DISPLAY')):
    show_camera = False
    

def help():
    print('python classify.py <path_to_model.eim> <Camera port ID, only required when more than 1 camera is present>')

def main(argv):    
    incCounteri = 0
    incCounterp = 0
    incCountert = 0
    runi = False
    runp = False
    runt = False
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
                # Start pipeline
                device.startPipeline()
                # Output queue will be used to get the depth frames from the outputs defined above
                depthQueue = device.getOutputQueue(name="depth")
                dispQ = device.getOutputQueue(name="disp")

                text = TextHelper()
                hostSpatials = HostSpatialsCalc(device)
                y = 200
                x = 300
                step = 3
                delta = 5
                hostSpatials.setDeltaRoi(delta)
                

                # Output queue will be used to get the rgb frames from the output defined above
                q_rgb = device.getOutputQueue(name="rgb", maxSize=4, blocking=False)

                while True:
                    in_rgb = q_rgb.get()  # blocking call, will wait until a new data has arrived
                    src = in_rgb.getCvFrame()
                    depthFrame = depthQueue.get().getFrame()
                    # Calculate spatial coordiantes from depth frame
                    spatials, centroid = hostSpatials.calc_spatials(depthFrame, (x,y)) # centroid == x/y in our case

                    # Get disparity frame for nicer depth visualization
                    disp = dispQ.get().getFrame()
                    disp = (disp * (255 / stereo.initialConfig.getMaxDisparity())).astype(np.uint8)
                    disp = cv2.applyColorMap(disp, cv2.COLORMAP_JET)                    

                    img = src

                    if img is not None:
                        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

                        # imread returns images in BGR format, so we need to convert to RGB

                        # get_features_from_image also takes a crop direction arguments in case you don't have square images
                        features, cropped = runner.get_features_from_image(img)

                        # the image will be resized and cropped, save a copy of the picture here
                        # so you can see what's being passed into the classifier

                        res = runner.classify(features)
                # print('classification runner response', res)

                # print('classification runner response', res)

                        if "classification" in res["result"].keys():
                            #print('Result (%d ms.) ' % (res['timing']['dsp'] + res['timing']['classification']), end='')
                            for label in labels:
                                score = res['result']['classification'][label]
                                print('%s: %.2f\t' % (label, score), end='')
                            print('', flush=True)

                        elif "bounding_boxes" in res["result"].keys():
                            #print('Found %d bounding boxes (%d ms.)' % (len(res["result"]["bounding_boxes"]), res['timing']['dsp'] + res['timing']['classification']))
                            for bb in res["result"]["bounding_boxes"]:
                                #print('\t%s (%.2f): x=%d y=%d w=%d h=%d' % (bb['label'], bb['value'], bb['x'], bb['y'], bb['width'], bb['height']))
                                #print(bb['label'])  
                                img = cv2.rectangle(img, (bb['x'], bb['y']), (bb['x'] + bb['width'], bb['y'] + bb['height']), (255, 0, 0), 1)

                            if len(res["result"]["bounding_boxes"]) == 0:
                                print("none")
                                stopMovement()
                            else:
                                for bb in res["result"]["bounding_boxes"]:
                                    if bb['value'] > 0.6:
                                        if bb['label'] == "thumb":
                                            print("thumb", bb['value'])
                                            incCountert += 1
                                        elif bb['label'] == "index":
                                            print("index", bb['value'])
                                            incCounteri += 1
                                        elif bb['label'] == "palm":
                                            print("palm", bb['value'])
                                            incCounterp += 1
                            print(incCounteri, incCountert, incCounterp)                
                            if incCounteri == 10:
                                runi = True
                            elif incCountert == 10:
                                runt = True
                            elif incCounterp == 10:
                                runp = True
                            print(runi, runt, runp)    
                                            
                            if runi == True:
                                print("openi")
                                incCounteri = 0
                                incCountert = 0
                                incCounterp = 0
                                indexMovement()
                                runi = False
                            elif runt == True:
                                print("opent")
                                thumbMovement()
                                incCountert = 0
                                incCounteri = 0
                                incCounterp = 0                        
                                runt = False
                            elif runp == True:
                                print("openp")
                                incCounterp = 0
                                incCounteri = 0
                                incCountert = 0                        
                                palmMovement()
                                runp = False
                        
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
