This is an error documentation README and it might not be very organized for a while. We hope to organize it with instructions once we have completed debugging it.

- 1. Bluetooth Error:
- https://www.reddit.com/r/raspberry_pi/comments/m9hk4s/fresh_rpi_4b_2go_cant_connect_to_bluetooth_devices/
- https://www.reddit.com/r/FLL/comments/dm8bxr/our_ev3_wont_connect/

- 2. Errors in Robot Arm Movement

Sometimes the motors can give errors in movements when setting the angle for the motor position. You can read more about the issue [here](https://github.com/RaspberryPiFoundation/python-build-hat/issues/179). To reduce this error, you will have to run [this](https://github.com/dhruvsheth-ai/Gelare/blob/main/software/Mindstorms_discrete/motor-diff.py) code to find the average error. Adjust the movement of the base motor to incorporate angular movement using the average error reading to increase the accuracy of the spatial movement.

```
#!/usr/bin/python3.9

import time
from buildhat import Motor

m = Motor('D')
m.bias(0.4)

sum_of_angle_differences = 0
angs = [-180, 180, 90, -90, 45, -45, 0] * 2
for i in angs:
    m.run_to_position(i)
    time.sleep(1)
    pos1 = m.get_aposition()
    print("Expected", i)
    print("Actual", m.get_aposition())
    diff = abs((i - pos1 + 180) % 360 - 180)
    sum_of_angle_differences += diff
    print(diff)

print("avg ", sum_of_angle_differences / len(angs))
```

This is how the code output looks:
<br>
![Code output](https://user-images.githubusercontent.com/45729391/215796317-4d2e4562-81d3-41d9-9690-a899bf3aecbd.jpeg)

Running the Robot Arm Error Calculation code:

https://user-images.githubusercontent.com/45729391/215797554-ade70045-533f-4334-a380-e1bb1855d9e2.mp4

