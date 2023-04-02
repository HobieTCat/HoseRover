from flask import Flask, render_template, request, Response, jsonify
import RPi.GPIO as GPIO
import cv2
import smbus
# from gy271 import read_gy_271
import serial
import time
from gpsParser import getPositionData

app = Flask(__name__)

# Setup the GPIO pin
GPIO.setmode(GPIO.BOARD)
GPIO.setup(11, GPIO.OUT)

# Create a PWM instance
pwm = GPIO.PWM(11, 50)
pwm.start(0)

# Create an instance of the I2C bus
bus = smbus.SMBus(1)

# GY-271 compass address
GY_271_ADDR = 0x0D

# GY-271 compass register addresses
GY_271_X_MSB = 0x01
GY_271_X_LSB = 0x02
GY_271_Y_MSB = 0x03
GY_271_Y_LSB = 0x04
GY_271_Z_MSB = 0x05
GY_271_Z_LSB = 0x06

ser = serial.Serial('/dev/ttyACM0', 9600)
# ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)

camera = cv2.VideoCapture(0)
fourcc = cv2.VideoWriter_fourcc(*'mp4v')
out = cv2.VideoWriter('output.mp4', fourcc, 20.0, (640, 480))

def read_gy_271():
    # Read the X, Y, and Z values from the GY-271 compass
    x_msb = bus.read_byte_data(GY_271_ADDR, GY_271_X_MSB)
    x_lsb = bus.read_byte_data(GY_271_ADDR, GY_271_X_LSB)
    y_msb = bus.read_byte_data(GY_271_ADDR, GY_271_Y_MSB)
    y_lsb = bus.read_byte_data(GY_271_ADDR, GY_271_Y_LSB)
    z_msb = bus.read_byte_data(GY_271_ADDR, GY_271_Z_MSB)
    z_lsb = bus.read_byte_data(GY_271_ADDR, GY_271_Z_LSB)

    # Convert the raw data to 16-bit signed integers
    x = (x_msb << 8) | x_lsb
    y = (y_msb << 8) | y_lsb
    z = (z_msb << 8) | z_lsb
    if x > 32767:
        x -= 65536
    if y > 32767:
        y -= 65536
    if z > 32767:
        z -= 65536

    # print('z=' , "12.2")    

    return x, y, z

def gen_frames():
    while True:
        success, frame = camera.read()
        if not success:
            break
        else:
            ret, buffer = cv2.imencode('.jpg', frame)
            frame = buffer.tobytes()
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
    # camera.release()
    # cv2.destroyAllWindows()        

@app.route('/')
def home():  
    return render_template('index.html')


@app.route('/data')
def data():
    x, y, z = read_gy_271()
    # print('y=' , y) 
    data = ''
    while '$GPGLL' not in data:
        data += ser.read_until(b'\r\n').decode('utf-8').strip()   
    latitudePosition, longitudePosition = getPositionData(data)  

    # latitudePosition = 1223.22312
    # longitudePosition = 312.22323

    #convert output to formay google maps can read 
    lat_deg = int(latitudePosition[:2])
    lat_min = latitudePosition[2:]
    lat_str = f"{lat_deg}°{lat_min}"
    latitudePosition = lat_str

    lon_deg = -int(longitudePosition[:3])
    lon_min = longitudePosition[3:]
    lon_str = f"{lon_deg}°{lon_min}"
    longitudePosition = lon_str

    # print(latitudePosition)    
    timestamp = int(time.time())
    return jsonify(x=x, y=y, z=z, gpsLat = latitudePosition, gpsLong = longitudePosition, theTime =timestamp)


@app.route('/video_feed')
def video_feed():
    return Response(gen_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/update', methods=['POST'])
def update():
    duty_cycle = int(request.form['slider'])
    print('Received duty cycle:', duty_cycle)
    pwm.ChangeDutyCycle(100-duty_cycle)
    return ''

if __name__ == '__main__':
    app.run(debug=False, host="192.168.0.232")

# Cleanup GPIO resources
GPIO.cleanup()


