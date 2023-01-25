**This page contains the trained Mobilenet SSD and [FOMO (Faster Objects More Objects)](https://www.edgeimpulse.com/blog/announcing-fomo-faster-objects-more-objects) models as well as instructions for converting your custom trained models on EdgeImpulse available in a TensorflowLite format (.tflite) to an OAK-D compatible (.blob) format.**

Credit to [@PINTO309](https://github.com/PINTO0309) for creating https://github.com/PINTO0309/tflite2tensorflow repository to enable efficient conversion.
  
<h2> Conversion </h2>

- On EdgeImpulse Dashboard, enable the `Custom deploys` and `Show Linux deploy options` under the **Admininstrative Zone**.
- Head over to deployment section and deploy the `custom block`. Once installed, a `(trained.tflite)` file should appear within the zip file.

Once done, follow the next set of instructions: 

Run the following commands on Docker. You do not need to install any packages other than Docker. It consumes about 26.7GB of host storage.

```
$ docker pull ghcr.io/pinto0309/tflite2tensorflow:latest

$ docker run -it --rm \
  -v `pwd`:/home/user/workdir \
  ghcr.io/pinto0309/tflite2tensorflow:latest
```

Next:

```
tflite2tensorflow \
--model_path /path/to/your/trained.tflite \
--flatc_path ../flatc \
--schema_path ../schema.fbs \
--output_no_quant_float32_tflite \
--output_onnx \
--onnx_opset 11 \
--output_openvino_and_myriad
```

That's it! It generates a `(.blob)` file in one of the zip files in your folder. Replace that `(.blob)` file with the default `(.blob)` model in the python script to use a custom EdgeImpulse FOMO trained model for object detection.

