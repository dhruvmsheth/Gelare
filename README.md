# Gelāre: Depth-sensing and EdgeImpulse powered Assistive Robotics on LEGO Inventor Kit
  
<p align="center">
    <img width="650" src="https://user-images.githubusercontent.com/67831664/213986629-835ee71b-f6b1-49f6-a993-d8fe26681e22.png" alt="Gelare">
</p>  
  
|Towards Spatial-guided Vision and Acoustic based Assistive Robotics using EdgeImpulse and LEGO Mindstorms. Gelare is a customizable tool to control LEGO Inventor kit robotic arms with Build Hat and OAK-D with an application in Assistive Robotics.|![Mindstorms Robot Inventor Logo](https://raw.githubusercontent.com/gpdaniels/spike-prime/master/simulator/images/icon-mindstorms.png)|![EI Logo](https://user-images.githubusercontent.com/67831664/214021405-40ce9f8a-e185-49e3-9605-864f46f0029c.png)|
|--|--|--|

**Models deployed using Gelāre are trained using [EdgeImpulse](https://edgeimpulse.com) which allows developers to train ML models in minutes and deploy these models on dozens of MCUs.**

![roboss](https://user-images.githubusercontent.com/67831664/214067911-7d130763-ddb9-40e6-91ac-c16eaa73013f.png)

<h2> Guides </h2>

The guide to train models within minutes and deploy it on Raspberry Pi and OAK-D can be found on the hackster project page. The specific instructions for converting a Tensorflow Lite Model (.tflite) available on EdgeImpulse dashboard to a (.blob) model compatible with OAK-D can be found [here](https://github.com/dhruvsheth-ai/Gelare/tree/main/models)

- **[Hardware Building Instructions](https://github.com/dhruvsheth-ai/Gelare/tree/main/building-instructions)**: Contains Instructions to Build the LEGO Mindstorms Inventor Kit Robotic Arm and integrating it with Raspberry Pi and Raspberry Pi Build Hat.
- **[Installation](https://github.com/dhruvsheth-ai/Gelare/blob/main/README.md#-installation-)**: If you have the setup ready, this guide provides you on running the model off-the-shelf using Gelāre's pretrained models
- **[Custom Model Training and Deployment](https://github.com/dhruvsheth-ai/Gelare/tree/main/models)**: Our hackster as well as github guide provides instructions to convert EdgeImpulse format models to run on OAK-D
- **[Running on Raspberry Pi](https://github.com/dhruvsheth-ai/Gelare/blob/main/README.md#-raspberry-pi-deployment-)**: Instructions to run Object Detection and Speech Recognition Models on Raspberry Pi without the need of OAK-D.
- **[Algorithms](https://github.com/dhruvsheth-ai/Gelare/tree/main/Algorithms)**: Algorithms which go behind running Gelāre.
- **[Recording Videos](https://github.com/dhruvsheth-ai/Gelare/tree/main/Demo%20Videos)**: Demo of the LEGO Mindstorms Robot running Gelāre.  
- **[Error Documentation](https://github.com/dhruvsheth-ai/Gelare/tree/main/error-docs)**: Solutions to some obvious errors during the build process.

<h2> Deployment </h2>

Gelāre supports two method to deploy object detection methods for real-time object detection off-the shelf. Each method uses LEGO Mindstorms Inventor kit and Raspberry Pi Build Hat.

**Using the [OAK-D (Opencv AI Kit with Depth)](https://store.opencv.ai/products/oak-d):** 

This is the first repository to extend support for deploying EdgeImpulse trained models to OAK-D. Gelāre integrates Depth estimation with object detection through EdgeImpulse's FOMO. To know more about OAK-D specifications, direct [here](https://github.com/dhruvsheth-ai/Gelare/blob/main/building-instructions/README.md#-oak-d-deployment-). 

The examples <insert_example1>, <insert_example2> and <insert_example3> are examples to run object detection on OAK-D controlling the LEGO Mindstorms Robot. The face-detection model, human-assisted feeding and gesture recognition model are trained on EdgeImpulse and are converted to `(.blob)` format required for OAK-D through the conversion instructions. `(.tflite)` -> `(.onnx)` -> `(.blob)`. You can train your own models on EdgeImpulse required for the specific use-case and follow the documentation to integrate it in the pipeline. We do have plans on integrating [Pose Estimation](https://github.com/edgeimpulse/pose-estimation-processing-block) feature released recently by EdgeImpulse, however this is a future prospect.

**Using Raspberry Pi and RPi Webcam:**

This is the simplest approach to deploy EdgeImpulse models on Raspberry Pi and get started with LEGO Robot Inventor Kit. This doesn't require Model Conversion, nor does it require an OAK-D. Since the processing power on a Raspberry Pi is limited and since it doesn't offer depth estimation capabilites, we have limited our deployment in the Computer Vision end only to Gesture recognition as seen in <insert_example 4>. However, Gelāre offers an extremely customizable pipeline, allowing users to modify the example and integrate their custom trained model using EdgeImpulse. Additionally, we extend this to Acoustic Modality, integrating speech-controlled Assistive Robotics in <insert_example 5>

<h2> Installation </h2>
<h3> Installation for OAK-D </h3>

```shell
$ sudo curl -fL https://docs.luxonis.com/install_dependencies.sh | bash
```

```shell
$ cd Gelare
$ pip3 install -r requirements.txt
```

<h3> Installation for running EdgeImpulse on Raspberry Pi: </h3>

```shell
$ sudo apt update
$ sudo apt upgrade
$ curl -sL https://deb.nodesource.com/setup_12.x | sudo bash -
$ sudo apt install -y gcc g++ make build-essential nodejs sox gstreamer1.0-tools gstreamer1.0-plugins-good gstreamer1.0-plugins-base gstreamer1.0-plugins-base-apps
$ npm config set user root && sudo npm install edge-impulse-linux -g --unsafe-perm
```  
Setup the camera module through `sudo raspi-config`. Not required for a webcam. This documentation uses a Microsoft Lifecam NX-6000

<h3> Installation for Build Hat </h3>

<h2> Quick Start </h2>

```shell
$ git clone https://github.com/dhruvsheth-ai/Gelare.git
$ cd Gelare/software/
```

**Using OAK-D**

- Example 1: Controlling Robotic Arm using Spatial coordinates of detected face:
Credit: EdgeImpulse Team. Model adopted from [source](https://studio.edgeimpulse.com/public/87291/latest/deployment)

To run:
```shell
python3 spatial_movement.py
```

- Example 2: Robot-assisted Feeding for patients with Hand Tremor. To investigate the algorithm behind the decision making process, check here <linkhere>. 


<h2> Raspberry Pi deployment </h2>

<h2> About the Developers </h2>

- **Dhruv Sheth**: Dhruv possibly knows more types of development boards than the letters of the alphabet and is skilled with TinyML and Robotic Arms. He recently represented India and won 3rd Grand Award at ISEF. More about him: https://dhruvsheth-ai.github.io/
- **Atharva Wasekar**: Atharva has a lot of experience in robotics competitions (2nd place IRC League Russia and Most Innovative Solution in WRS Japan) and can code in several different languages. More about him: https://in.linkedin.com/in/atharvawasekar

