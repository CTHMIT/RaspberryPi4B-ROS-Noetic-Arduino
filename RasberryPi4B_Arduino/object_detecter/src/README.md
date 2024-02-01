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
![image](https://github.com/CTHMIT/RaspberryPi4B-ROS-Noetic-Arduino/assets/107465888/e8e13d2a-3f7b-4127-9b07-c33ab22048e5)
## When the Car tracing object can send the command to the car
![image](https://github.com/CTHMIT/RaspberryPi4B-ROS-Noetic-Arduino/assets/107465888/0bf89b15-0075-4e8a-850a-436e56a55f37)
