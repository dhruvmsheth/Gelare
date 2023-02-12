#!/usr/bin/env python

import device_patches       
import cv2
import os
import sys, getopt
import signal
import time
from edge_impulse_linux.image import ImageImpulseRunner

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
    

def now():
    return round(time.time() * 1000)

def get_webcams():
    port_ids = []
    for port in range(5):
        print("Looking for a camera in port %s:" %port)
        camera = cv2.VideoCapture(port)
        if camera.isOpened():
            ret = camera.read()[0]
            if ret:
                backendName =camera.getBackendName()
                w = camera.get(3)
                h = camera.get(4)
                print("Camera %s (%s x %s) found in port %s " %(backendName,h,w, port))
                port_ids.append(port)
            camera.release()
    return port_ids

def sigint_handler(sig, frame):
    print('Interrupted')
    if (runner):
        runner.stop()
    sys.exit(0)

signal.signal(signal.SIGINT, sigint_handler)

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

    if len(args) == 0:
        help()
        sys.exit(2)

    model = args[0]

    dir_path = os.path.dirname(os.path.realpath(__file__))
    modelfile = os.path.join(dir_path, model)

    print('MODEL: ' + modelfile)

    with ImageImpulseRunner(modelfile) as runner:
        try:
            model_info = runner.init()
            print('Loaded runner for "' + model_info['project']['owner'] + ' / ' + model_info['project']['name'] + '"')
            labels = model_info['model_parameters']['labels']
            if len(args)>= 2:
                videoCaptureDeviceId = int(args[1])
            else:
                port_ids = get_webcams()
                if len(port_ids) == 0:
                    raise Exception('Cannot find any webcams')
                if len(args)<= 1 and len(port_ids)> 1:
                    raise Exception("Multiple cameras found. Add the camera port ID as a second argument to use to this script")
                videoCaptureDeviceId = int(port_ids[0])

            camera = cv2.VideoCapture(videoCaptureDeviceId)
            ret = camera.read()[0]
            if ret:
                backendName = camera.getBackendName()
                w = camera.get(3)
                h = camera.get(4)
                print("Camera %s (%s x %s) in port %s selected." %(backendName,h,w, videoCaptureDeviceId))
                camera.release()
            else:
                raise Exception("Couldn't initialize selected camera.")

            next_frame = 0 # limit to ~10 fps here

            for res, img in runner.classifier(videoCaptureDeviceId):
                if (next_frame > now()):
                    time.sleep((next_frame - now()) / 1000)

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
                    img = cv2.resize(img, (300, 300))
                    cv2.imshow('edgeimpulse', cv2.cvtColor(img, cv2.COLOR_RGB2BGR))
                    if cv2.waitKey(1) == ord('q'):
                        break

                next_frame = now() + 100
        finally:
            if (runner):
                runner.stop()

if __name__ == "__main__":
   main(sys.argv[1:])
