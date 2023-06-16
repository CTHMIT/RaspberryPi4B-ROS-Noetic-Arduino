import rospy
from std_msgs.msg import String
import adafruit_dht
import time
import board
from RPLCD import i2c

dhtDevice = adafruit_dht.DHT11(board.D4, use_pulseio=False)
lcdmode = 'i2c'
cols = 16
rows = 2
charmap = 'A00'
i2c_expander = 'PCF8574'
address = 0x27
port = 1
lcd = i2c.CharLCD(i2c_expander, address, port=port, charmap=charmap, cols=cols, rows=rows)

lcd.write_string("  Hello world! ")
lcd.crlf()
lcd.write_string("  Initialize.. ")
time.sleep(2.0)

rospy.init_node('dh11_monitor')

temperature_humidity_pub = rospy.Publisher('DH11TH', String, queue_size=10)

def dh11_callback(data):
    print("Received data: ",data.data)

rospy.Subscriber('DH11TH', String, dh11_callback)

while not rospy.is_shutdown():
    try:
        temperature_c = dhtDevice.temperature
        temperature_f = temperature_c * (9 / 5) + 32
        humidity = dhtDevice.humidity

        temperature_humidity_pub.publish("Temp: {:.2f} / Humi: {:.2f}".format(temperature_c, humidity))

        print("Temp: {:.2f} F / {:.2f} C    Humidity: {}% ".format(temperature_f, temperature_c, humidity))
        lcd.write_string("  Temp :{:.2f}C  ".format(temperature_c))
        
        lcd.write_string("  Humi :{:.2f}%  ".format(humidity))
        lcd.crlf()
    except RuntimeError as error:
        # print(error.args[0])
        # lcd.write_string("RuntimeError as error")
        # time.sleep(2.0)
        continue

    except Exception as error:
        dhtDevice.exit()
        raise error
        # lcd.write_string("Exception as error")
        # time.sleep(2.0)
        lcd.close(clear=True)

    time.sleep(2.0)

lcd.close(clear=True)
rospy.spin()
