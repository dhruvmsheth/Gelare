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


TurnDegrees1 = 85
x = (TurnDegrees1/22.5)
TurnDegrees1 = 75
x2 = (TurnDegrees1/22.5)    
motor_body_control.run_to_position(70, speed=50, direction='anticlockwise')
motor_gripper_control.run_to_position(75, speed=30, direction='clockwise')
motor_body_movement.run_for_rotations(x, speed=60)
motor_gripper_control.run_to_position(-80, speed=20, direction='anticlockwise')
motor_body_control.run_to_position(-30, speed=20, direction='clockwise')
time.sleep(0.25)
motor_gripper.run_to_position(140, speed=30, direction='clockwise')
motor_gripper_control.run_to_position(70, speed=30, direction='clockwise')
motor_body_control.run_to_position(70, speed=50, direction='anticlockwise')
motor_gripper_control.run_to_position(75, speed=30, direction='clockwise')
time.sleep(0.25)
motor_body_movement.run_for_rotations(x2, speed=-60)
motor_body_control.run_to_position(-14, speed=20, direction='clockwise')
motor_gripper_control.run_to_position(-4, speed=20, direction='anticlockwise')
motor_gripper.run_to_position(13, speed=60, direction='anticlockwise')
motor_gripper_control.run_to_position(70, speed=30, direction='clockwise')
