## Check Raspberry Pi 4 model b camera
``` shell
vcgencmd get_camera
```
### might be see the 

  supported=1 detected=1, libcamera interfaces=0

## tflite pre-train model and coco dataset
```shell
wget https://storage.googleapis.com/download.tensorflow.org/models/tflite/coco_ssd_mobilenet_v1_1.0_quant_2018_06_29.zip
unzip coco_ssd_mobilenet_v1_1.0_quant_2018_06_29.zip -d Sample_TFLite_model
```
## Raspberry Pi 4 Model B connect Arduino check
```shell
ls /dev/tty*
```
might see he 
  /dev/ttyACM0 different
