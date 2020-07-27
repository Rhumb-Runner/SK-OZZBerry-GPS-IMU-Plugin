#!/usr/bin/python
#
#    This program  reads the angles from the acceleromteer, gyroscope
#    and mangnetometer on a BerryIMU connected to a Raspberry Pi.
#
#    Both the BerryIMUv1 and BerryIMUv2 are supported
#
#    BerryIMUv1 uses LSM9DS0 IMU
#    BerryIMUv2 uses LSM9DS1 IMU
#
#    This program includes a number of calculations to improve the
#    values returned from BerryIMU. If this is new to you, it
#    may be worthwhile first to look at berryIMU-simple.py, which
#    has a much more simplified version of code which would be easier
#    to read.
#
#    This script is python 2.7 and 3 compatible
#
#    Feel free to do whatever you like with this code.
#    Distributed as-is; no warranty is given.
#
#    http://ozzmaker.com/

#    MODIFIED FOR SIGNAL K by Chuck Bowers (Rhumb Runner J/29)

import time
import math
import IMU
import datetime
import json
import sys
import bmp388

# Create a bmp388 object to communicate with I2C.
bmp388 = bmp388.SK_BMP388()

skData0 = ''
skData1 = ''
skData2 = ''
skData3 = ''
skData4 = ''
skData5 = ''
skDataTx = ''
currentHeading = 0
previousHeading = 0
rateOfTurn = 0
# If the IMU is upside down (Skull logo facing up), change this value to 1
IMU_UPSIDE_DOWN = 0

RAD_TO_DEG = 57.29578
M_PI = 3.14159265358979323846
G_GAIN = 0.070          # [deg/s/LSB]  If you change the dps for gyro, you need to update this value accordingly
AA = 0.40              # Complementary filter constant
MAG_LPF_FACTOR = .3    # Low pass filter constant magnetometer
ACC_LPF_FACTOR = .3    # Low pass filter constant for accelerometer
ACC_MEDIANTABLESIZE = 12         # Median filter table size for accelerometer. Higher = smoother but a longer delay
MAG_MEDIANTABLESIZE = 12         # Median filter table size for magnetometer. Higher = smoother but a longer delay

################# Compass Calibration values ############
# Use calibrateBerryIMU.py to get calibration values
# Calibrating the compass isnt mandatory, however a calibrated
# compass will result in a more accurate heading value.

magXmin = -1169
magYmin = -1617
magZmin = -1278
magXmax = 1530
magYmax = 1522
magZmax = 1055

# Kalman filter variables
Q_angle = 0.02
Q_gyro = 0.0015
R_angle = 0.005
y_bias = 0.0
x_bias = 0.0
XP_00 = 0.0
XP_01 = 0.0
XP_10 = 0.0
XP_11 = 0.0
YP_00 = 0.0
YP_01 = 0.0
YP_10 = 0.0
YP_11 = 0.0
KFangleX = 0.0
KFangleY = 0.0

def kalmanFilterY(accAngle, gyroRate, DT):
    y = 0.0
    S = 0.0

    global KFangleY
    global Q_angle
    global Q_gyro
    global y_bias
    global YP_00
    global YP_01
    global YP_10
    global YP_11

    KFangleY = KFangleY + DT * (gyroRate - y_bias)

    YP_00 = YP_00 + (- DT * (YP_10 + YP_01) + Q_angle * DT)
    YP_01 = YP_01 + (- DT * YP_11)
    YP_10 = YP_10 + (- DT * YP_11)
    YP_11 = YP_11 + (+ Q_gyro * DT)

    y = accAngle - KFangleY
    S = YP_00 + R_angle
    K_0 = YP_00 / S
    K_1 = YP_10 / S

    KFangleY = KFangleY + (K_0 * y)
    y_bias = y_bias + (K_1 * y)

    YP_00 = YP_00 - (K_0 * YP_00)
    YP_01 = YP_01 - (K_0 * YP_01)
    YP_10 = YP_10 - (K_1 * YP_00)
    YP_11 = YP_11 - (K_1 * YP_01)

    return KFangleY

def kalmanFilterX(accAngle, gyroRate, DT):
    x = 0.0
    S = 0.0

    global KFangleX
    global Q_angle
    global Q_gyro
    global x_bias
    global XP_00
    global XP_01
    global XP_10
    global XP_11

    KFangleX = KFangleX + DT * (gyroRate - x_bias)

    XP_00 = XP_00 + (- DT * (XP_10 + XP_01) + Q_angle * DT)
    XP_01 = XP_01 + (- DT * XP_11)
    XP_10 = XP_10 + (- DT * XP_11)
    XP_11 = XP_11 + (+ Q_gyro * DT)

    x = accAngle - KFangleX
    S = XP_00 + R_angle
    K_0 = XP_00 / S
    K_1 = XP_10 / S

    KFangleX = KFangleX + (K_0 * x)
    x_bias = x_bias + (K_1 * x)

    XP_00 = XP_00 - (K_0 * XP_00)
    XP_01 = XP_01 - (K_0 * XP_01)
    XP_10 = XP_10 - (K_1 * XP_00)
    XP_11 = XP_11 - (K_1 * XP_01)

    return KFangleX


gyroXangle = 0.0
gyroYangle = 0.0
gyroZangle = 0.0
CFangleX = 0.0
CFangleY = 0.0
CFangleXFiltered = 0.0
CFangleYFiltered = 0.0
kalmanX = 0.0
kalmanY = 0.0
oldXMagRawValue = 0
oldYMagRawValue = 0
oldZMagRawValue = 0
oldXAccRawValue = 0
oldYAccRawValue = 0
oldZAccRawValue = 0

a = datetime.datetime.now()

# Setup the tables for the mdeian filter. Fill them all with '1' so we dont get devide by zero error
acc_medianTable1X = [1] * ACC_MEDIANTABLESIZE
acc_medianTable1Y = [1] * ACC_MEDIANTABLESIZE
acc_medianTable1Z = [1] * ACC_MEDIANTABLESIZE
acc_medianTable2X = [1] * ACC_MEDIANTABLESIZE
acc_medianTable2Y = [1] * ACC_MEDIANTABLESIZE
acc_medianTable2Z = [1] * ACC_MEDIANTABLESIZE
mag_medianTable1X = [1] * MAG_MEDIANTABLESIZE
mag_medianTable1Y = [1] * MAG_MEDIANTABLESIZE
mag_medianTable1Z = [1] * MAG_MEDIANTABLESIZE
mag_medianTable2X = [1] * MAG_MEDIANTABLESIZE
mag_medianTable2Y = [1] * MAG_MEDIANTABLESIZE
mag_medianTable2Z = [1] * MAG_MEDIANTABLESIZE

IMU.detectIMU()     # Detect if BerryIMUv1 or BerryIMUv2 is connected.
IMU.initIMU()       # Initialise the accelerometer, gyroscope and compass


while True:

    # Read the accelerometer,gyroscope and magnetometer values
    ACCx = IMU.readACCx()
    ACCy = IMU.readACCy()
    ACCz = IMU.readACCz()
    GYRx = IMU.readGYRx()
    GYRy = IMU.readGYRy()
    GYRz = IMU.readGYRz()
    MAGx = IMU.readMAGx()
    MAGy = IMU.readMAGy()
    MAGz = IMU.readMAGz()

    # Apply compass calibration
    MAGx -= (magXmin + magXmax) / 2
    MAGy -= (magYmin + magYmax) / 2
    MAGz -= (magZmin + magZmax) / 2

    # Calculate loop Period(LP). How long between Gyro Reads
    b = datetime.datetime.now() - a
    a = datetime.datetime.now()
    LP = b.microseconds/(1000000*1.0)
    outputString = ""  # "Loop Time %5.2f " % ( LP )

    ###############################################
    #### Apply low pass filter ####
    ###############################################
    MAGx = MAGx * MAG_LPF_FACTOR + oldXMagRawValue*(1 - MAG_LPF_FACTOR)
    MAGy = MAGy * MAG_LPF_FACTOR + oldYMagRawValue*(1 - MAG_LPF_FACTOR)
    MAGz = MAGz * MAG_LPF_FACTOR + oldZMagRawValue*(1 - MAG_LPF_FACTOR)
    ACCx = ACCx * ACC_LPF_FACTOR + oldXAccRawValue*(1 - ACC_LPF_FACTOR)
    ACCy = ACCy * ACC_LPF_FACTOR + oldYAccRawValue*(1 - ACC_LPF_FACTOR)
    ACCz = ACCz * ACC_LPF_FACTOR + oldZAccRawValue*(1 - ACC_LPF_FACTOR)

    oldXMagRawValue = MAGx
    oldYMagRawValue = MAGy
    oldZMagRawValue = MAGz
    oldXAccRawValue = ACCx
    oldYAccRawValue = ACCy
    oldZAccRawValue = ACCz

    #########################################
    #### Median filter for accelerometer ####
    #########################################
    # cycle the table
    for x in range(ACC_MEDIANTABLESIZE-1, 0, -1):
        acc_medianTable1X[x] = acc_medianTable1X[x-1]
        acc_medianTable1Y[x] = acc_medianTable1Y[x-1]
        acc_medianTable1Z[x] = acc_medianTable1Z[x-1]

    # Insert the lates values
    acc_medianTable1X[0] = ACCx
    acc_medianTable1Y[0] = ACCy
    acc_medianTable1Z[0] = ACCz

    # Copy the tables
    acc_medianTable2X = acc_medianTable1X[:]
    acc_medianTable2Y = acc_medianTable1Y[:]
    acc_medianTable2Z = acc_medianTable1Z[:]

    # Sort table 2
    acc_medianTable2X.sort()
    acc_medianTable2Y.sort()
    acc_medianTable2Z.sort()

    # The middle value is the value we are interested in
    ACCx = acc_medianTable2X[int(ACC_MEDIANTABLESIZE/2)]
    ACCy = acc_medianTable2Y[int(ACC_MEDIANTABLESIZE/2)]
    ACCz = acc_medianTable2Z[int(ACC_MEDIANTABLESIZE/2)]

    #########################################
    #### Median filter for magnetometer ####
    #########################################
    # cycle the table
    for x in range(MAG_MEDIANTABLESIZE-1, 0, -1):
        mag_medianTable1X[x] = mag_medianTable1X[x-1]
        mag_medianTable1Y[x] = mag_medianTable1Y[x-1]
        mag_medianTable1Z[x] = mag_medianTable1Z[x-1]

    # Insert the latest values
    mag_medianTable1X[0] = MAGx
    mag_medianTable1Y[0] = MAGy
    mag_medianTable1Z[0] = MAGz

    # Copy the tables
    mag_medianTable2X = mag_medianTable1X[:]
    mag_medianTable2Y = mag_medianTable1Y[:]
    mag_medianTable2Z = mag_medianTable1Z[:]

    # Sort table 2
    mag_medianTable2X.sort()
    mag_medianTable2Y.sort()
    mag_medianTable2Z.sort()

    # The middle value is the value we are interested in
    MAGx = mag_medianTable2X[int(MAG_MEDIANTABLESIZE/2)]
    MAGy = mag_medianTable2Y[int(MAG_MEDIANTABLESIZE/2)]
    MAGz = mag_medianTable2Z[int(MAG_MEDIANTABLESIZE/2)]

    # Convert Gyro raw to degrees per second
    rate_gyr_x = GYRx * G_GAIN
    rate_gyr_y = GYRy * G_GAIN
    rate_gyr_z = GYRz * G_GAIN

    # Calculate the angles from the gyro.
    gyroXangle += rate_gyr_x*LP
    gyroYangle += rate_gyr_y*LP
    gyroZangle += rate_gyr_z*LP

    # Convert Accelerometer values to degrees

    if not IMU_UPSIDE_DOWN:
        # If the IMU is up the correct way (Skull logo facing down), use these calculations
        AccXangle = (math.atan2(ACCy, ACCz)*RAD_TO_DEG)
        AccYangle = (math.atan2(ACCz, ACCx)+M_PI)*RAD_TO_DEG
    else:
        # Use these four lines when the IMU is upside down. Skull logo is facing up
        AccXangle = (math.atan2(-ACCy, -ACCz)*RAD_TO_DEG)
        AccYangle = (math.atan2(-ACCz, -ACCx)+M_PI)*RAD_TO_DEG

    # Change the rotation value of the accelerometer to -/+ 180 and
    # Move the Y axis '0' point to up.  This makes it easier to read.
    if AccYangle > 90:
        AccYangle -= 270.0
    else:
        AccYangle += 90.0

    # Complementary filter used to combine the accelerometer and gyro values.
    CFangleX = AA*(CFangleX+rate_gyr_x*LP) + (1 - AA) * AccXangle
    CFangleY = AA*(CFangleY+rate_gyr_y*LP) + (1 - AA) * AccYangle

    # Kalman filter used to combine the accelerometer and gyro values.
    kalmanY = kalmanFilterY(AccYangle, rate_gyr_y, LP)
    kalmanX = kalmanFilterX(AccXangle, rate_gyr_x, LP)

    if IMU_UPSIDE_DOWN:
        MAGy = -MAGy      # If IMU is upside down, this is needed to get correct heading.
    # Calculate heading
    heading = 180 * math.atan2(MAGy, MAGx)/M_PI

    # Only have our heading between 0 and 360
    if heading < 0:
        heading += 360

    ####################################################################
    ###################Tilt compensated heading#########################
    ####################################################################
    # Normalize accelerometer raw values.
    if not IMU_UPSIDE_DOWN:
        # Use these two lines when the IMU is up the right way. Skull logo is facing down
        accXnorm = ACCx/math.sqrt(ACCx * ACCx + ACCy * ACCy + ACCz * ACCz)
        accYnorm = ACCy/math.sqrt(ACCx * ACCx + ACCy * ACCy + ACCz * ACCz)
    else:
        # Use these four lines when the IMU is upside down. Skull logo is facing up
        accXnorm = -ACCx/math.sqrt(ACCx * ACCx + ACCy * ACCy + ACCz * ACCz)
        accYnorm = ACCy/math.sqrt(ACCx * ACCx + ACCy * ACCy + ACCz * ACCz)

    # Calculate pitch and roll

    pitch = math.asin(accXnorm)
    roll = -math.asin(accYnorm/math.cos(pitch))

    # Calculate the new tilt compensated values
    magXcomp = MAGx*math.cos(pitch)+MAGz*math.sin(pitch)

    # The compass and accelerometer are orientated differently on the LSM9DS0 and LSM9DS1 and the Z axis on the compass
    # is also reversed. This needs to be taken into consideration when performing the calculations
    if(IMU.LSM9DS0):
        magYcomp = MAGx*math.sin(roll)*math.sin(pitch)+MAGy*math.cos(roll)-MAGz*math.sin(roll)*math.cos(pitch)   # LSM9DS0
    else:
        magYcomp = MAGx*math.sin(roll)*math.sin(pitch)+MAGy*math.cos(roll)+MAGz*math.sin(roll)*math.cos(pitch)   # LSM9DS1

        # Calculate tilt compensated heading
    tiltCompensatedHeading = 180 * math.atan2(magYcomp, magXcomp)/M_PI

    if tiltCompensatedHeading < 0:
        tiltCompensatedHeading += 360

    previousHeading = currentHeading
    currentHeading = heading
    rateOfTurn = (currentHeading - previousHeading) / LP
    # print ('Raw Rateof Turn: %4.0f' % rateOfTurn)

##################### BMP388 Code ######################################

    # You can use an accurate altitude to calibrate sea level air pressure.
    # And then use this calibrated sea level pressure as a reference to obtain the calibrated altitude.
    # In this case,525.0m is chendu accurate altitude.
    seaLevel = bmp388.readSeaLevel(218.8)
    # print("seaLevel : %s Pa" %seaLevel)

    # If there is no need to calibrate altitude, calibrated_altitude = False
    calibrated_altitude = True

    # Read temperature and print it
    temp = (bmp388.readTemperature() + 273.15)  # Deg C Converted to Kelvin (K)
    # print("Temperature : %s C" %temp)
    pres = bmp388.readPressure()
    # print("Pressure : %s Pa" %pres)
    if(calibrated_altitude):
        # Read the calibrated altitude
        altitude = bmp388.readCalibratedAltitude(seaLevel)
        # print("calibrate Altitude : %s m" %altitude)
    else:
        # Read the altitude
        altitude = bmp388.readAltitude()
        # print("Altitude : %s m" %altitude)

    ############################ END ##################################

    if 1:                       # Change to '0' to stop sending the heading
        skData0 = '{"updates": [{ "values": [{ "path": "navigation.headingCompass", "value" :%01.3f' % (math.radians(tiltCompensatedHeading)) + ' }] }] }'
        serial_dict = json.loads(skData0.encode('ascii', 'strict'))
        sys.stdout.write(json.dumps(serial_dict))
        sys.stdout.write('\n')
        sys.stdout.flush()
        time.sleep(0.1)

    if 1:                       # Change to '0' to stop sending the rate of turn
        skData1 = '{"updates": [{ "values": [{ "path": "navigation.rateOfTurn", "value": %1.0f' % (math.radians(rateOfTurn)) + ' }] }] }'
        serial_dict = json.loads(skData1.encode('ascii', 'strict'))
        sys.stdout.write(json.dumps(serial_dict))
        sys.stdout.write('\n')
        sys.stdout.flush()
        time.sleep(0.1)

    if 1:                       # Change to '0' to stop sending the roll/pith angles from the Kalman filters
        skData2 = '{"updates": [{ "values": [{ "path": "navigation.attitude", "value": { "roll": %01.3f' % (math.radians(kalmanX)) + ', "pitch": %4.3f' % (math.radians(kalmanY)) + ' } }] }] }'
        serial_dict = json.loads(skData2.encode('ascii', 'strict'))
        sys.stdout.write(json.dumps(serial_dict))
        sys.stdout.write('\n')
        sys.stdout.flush()
        time.sleep(0.1)

    if 1:                       # Change to '0' to stop sending the temperature inside the RPi enclosure
        skData3 = '{"updates": [{ "values": [{ "path": "environment.rpi.temperature", "value": %01.1f' % (temp) + ' }] }] }'
        serial_dict = json.loads(skData3.encode('ascii', 'strict'))
        sys.stdout.write(json.dumps(serial_dict))
        sys.stdout.write('\n')
        sys.stdout.flush()
        time.sleep(0.1)

    if 1:                       # Change to '0' to stop sending the Barometric pressure from inside the RPi enclosure
        skData4 = '{"updates": [{ "values": [{ "path": "environment.rpi.pressure", "value": %01.2f' % (pres) + ' }] }] }'
        serial_dict = json.loads(skData4.encode('ascii', 'strict'))
        sys.stdout.write(json.dumps(serial_dict))
        sys.stdout.write('\n')
        sys.stdout.flush()
        time.sleep(0.1)

    if 0:                       # Change to '0' to stop sending the Altitude of the RPi enclosure
        skData5 = '{"updates": [{ "values": [{ "path": "environment.rpi.altitude", "value": %01.1f' % (altitude) + ' }] }] }'
        serial_dict = json.loads(skData5.encode('ascii', 'strict'))
        sys.stdout.write(json.dumps(serial_dict))
        sys.stdout.write('\n')
        sys.stdout.flush()
        time.sleep(0.1)

    # sys.stdout.flush()
    # slow program down a bit, makes the output more readable
    # time.sleep(0.5)