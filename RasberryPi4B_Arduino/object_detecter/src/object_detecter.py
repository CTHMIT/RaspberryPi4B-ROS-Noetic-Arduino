import os
import argparse
import cv2
import numpy as np
import sys
import serial
import rospy
import time
from std_msgs.msg import String
from threading import Thread
import importlib.util



class VideoStream:
    def __init__(self, resolution=(640, 480), framerate=30):
        # Initialize the PiCamera and the camera image stream
        self.stream = cv2.VideoCapture(0)
        ret = self.stream.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        ret = self.stream.set(3, resolution[0])
        ret = self.stream.set(4, resolution[1])

        # Read first frame from the stream
        (self.grabbed, self.frame) = self.stream.read()

        # Variable to control when the camera is stopped
        self.stopped = False

    def start(self):
        # Start the thread that reads frames from the video stream
        Thread(target=self.update, args=()).start()
        return self

    def update(self):
        # Keep looping indefinitely until the thread is stopped
        while True:
            # If the camera is stopped, stop the thread
            if self.stopped:
                # Close camera resources
                self.stream.release()
                return

            # Otherwise, grab the next frame from the stream
            (self.grabbed, self.frame) = self.stream.read()

    def read(self):
        # Return the most recent frame
        return self.frame

    def stop(self):
        # Indicate that the camera and thread should be stopped
        self.stopped = True

# Define and parse input arguments
parser = argparse.ArgumentParser()
parser.add_argument('--modeldir', help='Folder the .tflite file is located in', default='/home/pi/ros_catkin_ws/src/object_detecter/src/object_detecter_model/')
parser.add_argument('--graph', help='Name of the .tflite file, if different than detect.tflite', default='detect.tflite')
parser.add_argument('--labels', help='Name of the labelmap file, if different than labelmap.txt', default='labelmap.txt')
parser.add_argument('--threshold', help='Minimum confidence threshold for displaying detected objects', default=0.6)
parser.add_argument('--resolution', help='Desired webcam resolution in WxH. If the webcam does not support the resolution entered, errors may occur.', default='640x480')
parser.add_argument('--edgetpu', help='Use Coral Edge TPU Accelerator to speed up detection', action='store_true')

args = parser.parse_args()

MODEL_NAME = args.modeldir
GRAPH_NAME = args.graph
LABELMAP_NAME = args.labels
min_conf_threshold = float(args.threshold)
resW, resH = args.resolution.split('x')
imW, imH = int(resW), int(resH)
use_TPU = args.edgetpu

# Import TensorFlow libraries
# If tflite_runtime is installed, import interpreter from tflite_runtime, else import from regular tensorflow
# If using Coral Edge TPU, import the load_delegate library
pkg = importlib.util.find_spec('tflite_runtime')
if pkg:
    from tflite_runtime.interpreter import Interpreter
    if use_TPU:
        from tflite_runtime.interpreter import load_delegate
else:
    from tensorflow.lite.python.interpreter import Interpreter
    if use_TPU:
        from tensorflow.lite.python.interpreter import load_delegate

# If using Edge TPU, assign filename for Edge TPU model
if use_TPU:
    # If user has specified the name of the .tflite file, use that name, otherwise use default 'edgetpu.tflite'
    if GRAPH_NAME == 'detect.tflite':
        GRAPH_NAME = 'edgetpu.tflite'

# Get path to current working directory
CWD_PATH = os.getcwd()

# Path to .tflite file, which contains the model that is used for object detection
PATH_TO_CKPT = os.path.join(CWD_PATH, MODEL_NAME, GRAPH_NAME)

# Path to label map file
PATH_TO_LABELS = os.path.join(CWD_PATH, MODEL_NAME, LABELMAP_NAME)

# Load the label map
with open(PATH_TO_LABELS, 'r') as f:
    labels = [line.strip() for line in f.readlines()]

# Have to do a weird fix for label map if using the COCO "starter model" from
# https://www.tensorflow.org/lite/models/object_detection/overview
# First label is '???', which has to be removed.
if labels[0] == '???':
    del labels[0]

# Load the Tensorflow Lite model.
# If using Edge TPU, use special load_delegate argument
if use_TPU:
    interpreter = Interpreter(model_path=PATH_TO_CKPT, experimental_delegates=[load_delegate('libedgetpu.so.1.0')])
else:
    interpreter = Interpreter(model_path=PATH_TO_CKPT)

interpreter.allocate_tensors()

# Get model details
input_details = interpreter.get_input_details()
output_details = interpreter.get_output_details()
height = input_details[0]['shape'][1]
width = input_details[0]['shape'][2]

floating_model = input_details[0]['dtype'] == np.float32

input_mean = 127.5
input_std = 127.5

# Check output layer name to determine if this model was created with TF2 or TF1,
# because outputs are ordered differently for TF2 and TF1 models
outname = output_details[0]['name']

if 'StatefulPartitionedCall' in outname:  # This is a TF2 model
    boxes_idx, classes_idx, scores_idx = 1, 3, 0
else:  # This is a TF1 model
    boxes_idx, classes_idx, scores_idx = 0, 1, 2

# Initialize frame rate calculation
frame_rate_calc = 1
freq = cv2.getTickFrequency()

information = None
def callback(data):
    global information
    information=data.data

# Initialize video stream
videostream = VideoStream(resolution=(imW, imH), framerate=30).start()
rospy.init_node('object_detection_node')
rospy.loginfo('tflite_opencv_object_detection')
pub=rospy.Publisher("picmd", String, queue_size=10)
rospy.Subscriber("picmd", String, callback)
rate = rospy.Rate(10)
max_score = 0
target_list = ['person']
target_detect = False
while not rospy.is_shutdown():
    try:   
        
        # Grab frame from video stream
        frame = videostream.read()
        rospy.Subscriber("feedback", String, callback)
        # Acquire frame and resize to expected shape [1xHxWx3]
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        frame_resized = cv2.resize(frame_rgb, (width, height))
        input_data = np.expand_dims(frame_resized, axis=0)

        # Normalize pixel values if using a floating model (i.e., if model is non-quantized)
        if floating_model:
            input_data = (np.float32(input_data) - input_mean) / input_std

        # Perform the actual detection by running the model with the image as input
        interpreter.set_tensor(input_details[0]['index'], input_data)
        interpreter.invoke()

        # Retrieve detection results
        boxes = interpreter.get_tensor(output_details[boxes_idx]['index'])[0]  # Bounding box coordinates of detected objects
        classes = interpreter.get_tensor(output_details[classes_idx]['index'])[0]  # Class index of detected objects
        scores = interpreter.get_tensor(output_details[scores_idx]['index'])[0]  # Confidence of detected objects
        center_x = int(imW / 2)
        center_y = int(imH / 2)
        cross_size = 15
        cv2.line(frame, (center_x - cross_size, center_y), (center_x + cross_size, center_y), (0, 0, 255), 2)
        cv2.line(frame, (center_x, center_y - cross_size), (center_x, center_y + cross_size), (0, 0, 255), 2)

        # Loop over all detections and draw detection box if confidence is above the minimum threshold
        
    
        for i in range(len(scores)):
            if min_conf_threshold <= scores[i] <= 1.0:
                if labels[int(classes[i])] in target_list:
                    if target_detect:
                        continue
                    else:                                           
                        max_score = scores[i]
                        max_score_index = i
                        object_name = labels[int(classes[max_score_index])]  
                        label = f'{object_name}: {int(scores[max_score_index]*100)}%' 
                        target_detect = True
                    
                
        if target_detect:
            ymin = int(max(1, (boxes[max_score_index][0] * imH)))
            xmin = int(max(1, (boxes[max_score_index][1] * imW)))
            ymax = int(min(imH, (boxes[max_score_index][2] * imH)))
            xmax = int(min(imW, (boxes[max_score_index][3] * imW)))

            cv2.rectangle(frame, (xmin, ymin), (xmax, ymax), (10, 255, 0), 2)

            target_x = int((xmin + xmax) / 2)
            target_y = int((ymin + ymax) / 2)

            cv2.line(frame, (target_x-10, target_y), (target_x+10, target_y), (255, 0, 0), 2)  # X-axis
            cv2.line(frame, (target_x, target_y-10), (target_x, target_y+10), (255, 0, 0), 2)  # Y-axis

            tolerance = center_x - target_x # Only need the X-axis for correction

            if tolerance > 15:
                pi_sent = "L"
            elif tolerance < -15:
                pi_sent = "R"
            else:
                pi_sent = "F"
            pub.publish(pi_sent)
            
            rospy.loginfo(f'Detect {label} Command Pi sent {pi_sent} -> Arduino, Arduino got {information} from Pi')

        else :
            pub.publish("B")
            time.sleep(1)      
            pub.publish("S")
            rospy.loginfo(f'No Target Object dectection')
            target_detect = False
                        
                                        
        # Display the frame
        cv2.imshow('Object detector', frame)
        
        # Check if 'q' key is pressed to quit
        if cv2.waitKey(1) == ord('q'):
            pub.publish("S")
            break
    except:
        break

    rate.sleep()
rospy.spin()    
# Clean up
cv2.destroyAllWindows()
videostream.stop()
pub.publish("S")