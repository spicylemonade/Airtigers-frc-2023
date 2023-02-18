import time
import board
import busio
import math
import digitalios
import adafruit_lis3dh


## Accelerometer is hooked up to robot (IMU)

ic2 = busio.I2C(board.Accelerometer_SCL, board.Accelerometer_SDA)
_intl = digitalio.DigitalInOut(board.Accelerometer_INTERRUPT)
lis3dh = adafruit_lis3dh.LIS3DH_I2C(i2c, address = 0x19, intl=_intl)
lis3dh.range = adafruit_lis3dh.RANGE_8_G

while True:
    
    ax, ay, az = lis3dh.acceleration
    anorm = math.sqrt(ax*ax + ay*ay + az*az)
    axbar = ax/anorm
    aybar = ay/anorm
    azbar = az/anorm
    theta = math.asin(axbar)*180.0/math.pi
    roll = math.atan2(aybar, azbar)*180.0/math.pi
    print((theta, roll))
    #print((axbar,aybar,azbar))
    time.sleep(0.1)
