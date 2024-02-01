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
might see the /dev/ttyACM0 different

### Set the connection and Serial baud
In object_detecter.py :

  - rospy.Publisher("picmd"), String, callback)
  - rospy.Subscriber("feedback"), String, callback)

will Set the connection and Serial baud
![image](https://github.com/CTHMIT/RaspberryPi4B-ROS-Noetic-Arduino/assets/107465888/3485f5e7-4dfe-45a9-8fbb-c304cdac3a23)

## When the Car tracing object can send the command to the car
![image](https://github.com/CTHMIT/RaspberryPi4B-ROS-Noetic-Arduino/assets/107465888/92870fbc-d7ec-4d48-a3ed-e2ee5c74bbb8)

