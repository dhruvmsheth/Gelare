# Gelāre: Depth-sensing and EdgeImpulse powered Assistive Robotics on LEGO Inventor Kit
  
<p align="center">
    <img width="650" src="https://user-images.githubusercontent.com/67831664/213986629-835ee71b-f6b1-49f6-a993-d8fe26681e22.png" alt="Gelare">
</p>  
  
|Towards Spatial-guided Vision and Acoustic based Assistive Robotics using EdgeImpulse and LEGO Mindstorms. Gelare is a customizable tool to control LEGO Inventor kit robotic arms with Build Hat and OAK-D with an application in Assistive Robotics.|![Mindstorms Robot Inventor Logo](https://raw.githubusercontent.com/gpdaniels/spike-prime/master/simulator/images/icon-mindstorms.png)|![EI Logo](https://user-images.githubusercontent.com/67831664/214021405-40ce9f8a-e185-49e3-9605-864f46f0029c.png)|
|--|--|--|

![roboss](https://user-images.githubusercontent.com/67831664/214067911-7d130763-ddb9-40e6-91ac-c16eaa73013f.png)

**Models deployed using Gelāre are trained using [EdgeImpulse](https://edgeimpulse.com) which allows developers to train ML models in minutes and deploy these models on dozens of MCUs.**

The guide to train models within minutes and deploy it on Raspberry Pi and OAK-D can be found on the hackster project page. The specific instructions for converting a Tensorflow Lite Model (.tflite) available on EdgeImpulse dashboard to a (.blob) model compatible with OAK-D can be found [here](https://github.com/dhruvsheth-ai/Gelare/tree/main/models)

Gelāre supports two method to deploy object detection methods for real-time object detection off-the shelf. Each method uses LEGO Mindstorms Inventor kit and Raspberry Pi Build Hat.

- Using the [OAK-D (Opencv AI Kit with Depth)](https://store.opencv.ai/products/oak-d). This is the first repository to extend support for deploying EdgeImpulse trained models to OAK-D.


- (under development)
