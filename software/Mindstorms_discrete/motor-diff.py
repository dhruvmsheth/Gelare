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
