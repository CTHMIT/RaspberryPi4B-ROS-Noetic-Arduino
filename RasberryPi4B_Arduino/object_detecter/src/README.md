## Test the arduino and pi connection
```shell
python3 listener.py
```
![image](https://github.com/CTHMIT/RaspberryPi4B-ROS-Noetic-Arduino/assets/107465888/0cabb768-d70f-4ab6-befe-2bacbda792c3)

### Set the connection and Serial baud
In object_detecter.py :

  - rospy.Publisher("picmd"), String, callback)
  - rospy.Subscriber("feedback"), String, callback)

will Set the connection and Serial baud
![image](https://github.com/CTHMIT/RaspberryPi4B-ROS-Noetic-Arduino/assets/107465888/3485f5e7-4dfe-45a9-8fbb-c304cdac3a23)

## When the Car tracing object can send the command to the car
![image](https://github.com/CTHMIT/RaspberryPi4B-ROS-Noetic-Arduino/assets/107465888/92870fbc-d7ec-4d48-a3ed-e2ee5c74bbb8)

