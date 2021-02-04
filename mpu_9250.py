'''
Name:           MPU_9250.py

Author:         Kenny Bunnell

Date:           2 February 2021

Installation:   Illinois Institute of Technology

Description:    This program was developed to calibrate and read from an MPU 9250 sensor.
                Developed as a homework assignment for Prof. Williams Aerospace Laboratory II

                Wiring:     (MPU)       (RasPi)
                            VCC     ->  3V3
                            GND     ->  GND
                            SCL     ->  SCL1
                            SDA     ->  SDA1
                            EOA     ->  NC
                            ECL     ->  NC
                            ADO     ->  NC
                            INT     ->  NC
                            NCS     ->  NC
                            FSYNC   ->  NC
'''

import math
import time
import smbus
import numpy as np
import RPi.GPIO as GPIO
from scipy.stats import t
import matplotlib.pyplot as plt
from sklearn.linear_model import LinearRegression
import csv
import os.path

# Register
power_mgmt_1 = 0x6b
power_mgmt_2 = 0x6c

# Calibration Data
G_XcountsPerG = 16348.0
G_XcountOffset = 0
G_YcountsPerG = 16348.0
G_YcountOffset = 0
G_ZcountsPerG = 16348.0
G_ZcountOffset = 0
G_keepHistory = False
G_calibrationFile = None
G_csvWriter = None
G_calibrations = []

# Create the Calibration CSV if it does not exist
def getCSV():
    global G_calibrationFile
    global G_csvWriter
    global G_calibrations
    newFileFlag = False
    if os.path.isfile('calibrations.csv'):
        readCalibrationFile = open('calibrations.csv', 'r')
        csvReader = csv.DictReader(readCalibrationFile)
        for line in csvReader:
            G_calibrations.append(line)
        print(G_calibrations)
    else:
        newFileFlag = True
    G_calibrationFile = open('calibrations.csv', 'w', newline='')
    fields = ['name','coefficient','intercept']
    G_csvWriter = csv.DictWriter(G_calibrationFile, fields)
    G_csvWriter.writeheader()

getCSV()

# Write the calibrations to the CSV file
def writeCSV():
    global G_csvWriter
    global G_calibrations
    G_csvWriter.writerows(G_calibrations)

# Read a byte from the MPU sensor
def read_byte(reg):
    return bus.read_byte_data(address, reg)

# Read a word from the MPU sensor
def read_word(reg):
    h = bus.read_byte_data(address, reg)
    l = bus.read_byte_data(address, reg+1)
    value = (h << 8) + l
    return value
 
# Read a word from the MPU and return the equivalent signed short
def read_word_2c(reg):
    val = read_word(reg)
    if (val >= 0x8000):
        return -((65535 - val) + 1)
    else:
        return val

# Write a byte to the MPU sensor
def write_byte(reg,offset,val):
    bus.write_byte_data(reg, offset, val)

# Find the vector sum of two values
def dist(a,b):
    return math.sqrt((a*a)+(b*b))

# Get the angle that y has been rotated by
def get_y_rotation(x,y,z):
    radians = math.atan2(x, dist(y,z))
    if(z < 0):
        if(x < 0):
            radians = -math.pi - radians
        else:
            radians = math.pi - radians
    return math.degrees(radians)

# Get the angle that x has been rotated by
def get_x_rotation(x,y,z):
    radians = math.atan2(y, dist(x,z))
    if(z < 0):
        if(y < 0):
            radians = -math.pi - radians
        else:
            radians = math.pi - radians
    return math.degrees(radians)

# Find the expected acceleration/g of X using the Y rotation
def accelX(yRot):
    return math.sin(math.radians(yRot))

# Find the expected acceleration/g of Y using the X rotation
def accelY(xRot):
    return math.sin(math.radians(xRot))

# Find the expected acceleration/g of Z using the X and Y rotation
def accelZ(xRot,yRot):
    if((abs(xRot) < 90) and (abs(yRot) < 90)):
        return math.cos(math.radians(xRot))*math.cos(math.radians(yRot))
    else:
        return -math.cos(math.radians(xRot))*math.cos(math.radians(yRot))

# Take Accelerometer measurements and find the uncertainties
def takeDataMPU(sensor, factoryCal = False):
    counts, idealAccels, accels = [], [], []
    df = -1
    while(input('Set the MPU orientation, then press \'Enter\' when you are ready\nto record the data point or type \'q\' to quit: ') != 'q'):
        # Read the acceleration values from the sensor
        accelerometerCountsXout = read_word_2c(0x3b)
        accelerometerCountsYout = read_word_2c(0x3d)
        accelerometerCountsZout = read_word_2c(0x3f)
        
        # Use the acceleration values to approximate the x and y rotation of the sensor
        x_rot = get_x_rotation(accelerometerCountsXout,accelerometerCountsYout,accelerometerCountsZout)
        y_rot = get_y_rotation(accelerometerCountsXout,accelerometerCountsYout,accelerometerCountsZout)
        
        if(factoryCal):
            # Use the ideal factory calibration to find the ideal acceleration
            accelerationXout = accelerometerCountsXout / 16384.0
            accelerationYout = accelerometerCountsYout / 16384.0
            accelerationZout = accelerometerCountsZout / 16384.0
        else:
            accelerationXout = (accelerometerCountsXout - G_XcountOffset) / G_XcountsPerG
            accelerationYout = (accelerometerCountsYout - G_YcountOffset) / G_YcountsPerG
            accelerationZout = (accelerometerCountsZout - G_ZcountOffset) / G_ZcountsPerG
        
        # Increase the degrees of freedom after each new data point
        df += 1
        
        # Append the counts, actual acceleration values, and the ideal acceleration values to a list
        if(sensor == 'x'):
            counts.append(accelerometerCountsXout)
            accels.append(accelX(y_rot))
            idealAccels.append(accelerationXout)
            print("\033[F\033[F {:3d} Counts X: {:7d}    Counts Y: {:7d}    Accel X: {:10.4f}    Accel Ideal X: {:10.4f}".format(df,accelerometerCountsXout,accelerometerCountsYout,accelX(y_rot),accelerationXout))
        elif(sensor == 'y'):
            counts.append(accelerometerCountsYout)
            accels.append(accelY(x_rot))
            idealAccels.append(accelerationYout)
            print("\033[F\033[F {:3d} Counts X: {:7d}    Counts Y: {:7d}    Accel Y: {:10.4f}    Accel Ideal Y: {:10.4f}".format(df,accelerometerCountsXout,accelerometerCountsYout,accelY(x_rot),accelerationYout))
        elif(sensor == 'z'):
            counts.append(accelerometerCountsZout)
            accels.append(accelZ(x_rot,y_rot))
            idealAccels.append(accelerationZout)
            print("\033[F\033[F {:3d} Counts X: {:7d}    Counts Y: {:7d}    Accel Z: {:10.4f}    Accel Ideal Z: {:10.4f}".format(df,accelerometerCountsXout,accelerometerCountsYout,accelZ(x_rot,y_rot),accelerationZout))
    return counts, idealAccels, accels

# Measure and find the uncertainty for 1 orientation
def measureOrientation():
    try:
        sensor = input("Select a sensor to focus on (X,Y or Z): ").lower()
        if(len(sensor) > 0):
            if(sensor in 'xyz'):
                counts, idealAccels, accels = takeDataMPU(sensor)
                measurement_mean = np.mean(counts)
                measurement_stdev = np.std(counts)
                df = len(counts) - 1
                uncertainty = t.ppf(0.025,df) * measurement_stdev / math.sqrt(len(counts))
                print("\nAverage counts: {:.0f} \u00B1 {:.0f} counts".format(measurement_mean,abs(uncertainty)))
    except KeyboardInterrupt:
        pass
    return


# 1st Degree Calibratation of X,Y, or Z sensor
def calibrateMPU():
    global G_XcountOffset
    global G_YcountOffset
    global G_ZcountOffset
    global G_XcountsPerG
    global G_YcountsPerG
    global G_ZcountsPerG
    try:
        options = input("List any sensors you would like to calibrate (X,Y,Z): ").lower()
        for opt in options:
            if opt in 'xyz':
                print("\tCalibrating {} Sensor".format(opt))
                counts, idealAccels, accels = takeDataMPU(sensor = opt, factoryCal = True)
                df = len(counts)

                if(df > 1):
                    # Sort the data by the acceleration values
                    # This makes the graphs look better if the user chose a random order of data points
                    accelsSorted, countsSorted1 = zip(*sorted(zip(accels, counts)))
                    idealAccelsSorted, countsSorted2 = zip(*sorted(zip(idealAccels, counts)))

                    # Fit a linear line to the actual acceleration data
                    fit = LinearRegression().fit(np.asarray(accelsSorted)[:,np.newaxis],np.asarray(countsSorted1))
                    corrected = fit.predict(np.asarray(accelsSorted)[:,np.newaxis])

                    error = []
                    for i in range(len(corrected)):
                        error.append(countsSorted1[i] - corrected[i])

                    # Sort the error data for the same reason as mentioned in the above sorts
                    accelsSortedErr, errorSorted = zip(*sorted(zip(accels, error)))

                    # Find the average and standard dev of the errors
                    mean_err = np.mean(errorSorted)
                    std_error = np.std(errorSorted)

                    # Calculate the student's uncertainty of the data
                    uncertainty = t.ppf(0.025,df) * std_error / math.sqrt(df+1)

                    print("\nLinear Regression: y = mx + b, where m = {:.4f}, b = {:.4f}".format(fit.coef_[0],fit.intercept_))
                    print("R2 =",fit.score(np.asarray(accelsSorted)[:,np.newaxis],np.asarray(countsSorted1)))
                    print("Uncertainty of sensor: {:.0f} counts ({:.3f}%)".format(abs(uncertainty),abs(uncertainty)/163.84))

                    # Create a plot to help visualize the calibration that was just performed
                    fig, axs = plt.subplots(2)
                    if(opt == 'x'):
                        G_XcountsPerG = fit.coef_[0]
                        G_XcountOffset = fit.intercept_
                        fig.suptitle('MPU 9250 X Calibration')
                    elif(opt == 'y'):
                        G_YcountsPerG = fit.coef_[0]
                        G_YcountOffset = fit.intercept_
                        fig.suptitle('MPU 9250 Y Calibration')
                    elif(opt == 'z'):
                        G_ZcountsPerG = fit.coef_[0]
                        G_ZcountOffset = fit.intercept_
                        fig.suptitle('MPU 9250 Z Calibration')

                    axs[0].errorbar(accelsSorted,countsSorted1,yerr=uncertainty,fmt='b--x',label="Factory Calibration")
                    axs[0].plot(idealAccelsSorted,countsSorted2,'r:.',label="Ideal Calibration")
                    axs[0].legend()

                    axs[1].plot(accelsSortedErr,errorSorted,'k--o')
                    
                    axs[0].set(xlabel='Acceleration [/g]', ylabel='Counts')
                    axs[1].set(xlabel='Acceleration [/g]', ylabel='Count Errors')

                    for ax in axs.flat:
                        ax.label_outer()
                    plt.show()

                    saveFile = input("\nWould you like to save this calibration? (y/n): ").lower()
                    if(saveFile == 'y'):
                        global G_calibrationFile
                        global G_csvWriter
                        global G_calibrations
                        if(G_calibrationFile == None):
                            getCSV()
                        calibrationName = input("Choose a name for your calibration: ")
                        G_calibrations.append({'name':calibrationName,'coefficient':fit.coef_[0],'intercept':fit.intercept_})
                        print("Saved!")
                else:
                    print("Not enough points taken, exiting this calibration...")
    except KeyboardInterrupt:
        pass
    return

# Load a previous calibration
def loadCalibration():
    global G_XcountOffset
    global G_YcountOffset
    global G_ZcountOffset
    global G_XcountsPerG
    global G_YcountsPerG
    global G_ZcountsPerG
    global G_calibrations
    print("The loaded calibrations are:")
    index = 0
    calNames = []
    for cal in G_calibrations:
        print("\t{}:\t{}".format(index,cal['name']))
        index += 1
    for sensor in 'XYZ':
        calIndex = input('\nSelect a calibration index to load for the {} sensor (or press \'Enter\' to skip): '.format(sensor))
        if(len(calIndex) > 0):
            if((calIndex in '0123456789') and (int(calIndex) < index)):
                calIndex = int(calIndex)
                if(sensor == 'X'):
                    G_XcountOffset = float(G_calibrations[calIndex]['intercept'])
                    G_XcountsPerG = float(G_calibrations[calIndex]['coefficient'])
                    print("\tSet Offset to {} and slope to {} for {}".format(G_XcountOffset,G_XcountsPerG,sensor))
                elif(sensor == 'Y'):
                    G_YcountOffset = float(G_calibrations[calIndex]['intercept'])
                    G_YcountsPerG = float(G_calibrations[calIndex]['coefficient'])
                    print("\tSet Offset to {} and slope to {} for {}".format(G_YcountOffset,G_YcountsPerG,sensor))
                elif(sensor == 'Z'):
                    G_ZcountOffset = float(G_calibrations[calIndex]['intercept'])
                    G_ZcountsPerG = float(G_calibrations[calIndex]['coefficient'])
                    print("\tSet Offset to {} and slope to {} for {}".format(G_ZcountOffset,G_ZcountsPerG,sensor))
            else:
                print("\tInvalid calibration index")
        else:
            print("\tSkipping...")
    return

# Toggle the G_keepHistory bool
def toggleHistory():
    global G_keepHistory
    G_keepHistory = not G_keepHistory
    print("Keep History flag is now set to",G_keepHistory)
    return

# Display the gyroscope data every half-second
def continuousGyro():
    global G_keepHistory
    try:
        print("\n -Gyroscope- Press CTRL+C to exit.")
        while(True):
            # Read the gyroscope counts from the MPU sensor
            gyroskop_xout = read_word_2c(0x43)
            gyroskop_yout = read_word_2c(0x45)
            gyroskop_zout = read_word_2c(0x47)

            print("Gyroscope X: {:7d}    Normalized: {:8.4f}    ".format(gyroskop_xout,gyroskop_xout / 131))
            print("Gyroscope Y: {:7d}    Normalized: {:8.4f}    ".format(gyroskop_yout,gyroskop_yout / 131))
            print("Gyroscope Z: {:7d}    Normalized: {:8.4f}    ".format(gyroskop_zout,gyroskop_zout / 131))
            time.sleep(0.5)
            if(G_keepHistory):
                print()
            else:
                print("\033[F "*4)
    except KeyboardInterrupt:
        pass
    return

# Display the acceleration data every half-second
def continuousAccel():
    global G_keepHistory
    try:
        print("\n -Accelerometer- Press CTRL+C to exit.")
        while(True):
            # Read the acceleration counts from the MPU sensor
            accelerometerCountsXout = read_word_2c(0x3b)
            accelerometerCountsYout = read_word_2c(0x3d)
            accelerometerCountsZout = read_word_2c(0x3f)
            
            # Use the calibration to find the acceleration from the sensor counts
            # If no calibration is performed prior, it uses the factory calibration
            accelerationXout = (accelerometerCountsXout - G_XcountOffset) / G_XcountsPerG
            accelerationYout = (accelerometerCountsYout - G_YcountOffset) / G_YcountsPerG
            accelerationZout = (accelerometerCountsZout - G_ZcountOffset) / G_ZcountsPerG
            
            print("X Counts: {:7d}    Normalized: {:8.4f}    ".format(accelerometerCountsXout,accelerationXout))
            print("Y Counts: {:7d}    Normalized: {:8.4f}    ".format(accelerometerCountsYout,accelerationYout))
            print("Z Counts: {:7d}    Normalized: {:8.4f}    ".format(accelerometerCountsZout,accelerationZout))
            time.sleep(0.5)
            if(G_keepHistory):
                print()
            else:
                print("\033[F "*4)
    except KeyboardInterrupt:
        pass
    return

# Display the rotation angles every half-second
def continuousRot():
    global G_keepHistory
    try:
        print("\n -MPU Rotation- Press CTRL+C to exit.")
        while(True):
            accelerometerCountsXout = read_word_2c(0x3b)
            accelerometerCountsYout = read_word_2c(0x3d)
            accelerometerCountsZout = read_word_2c(0x3f)
            
            x_rot = get_x_rotation(accelerometerCountsXout,accelerometerCountsYout,accelerometerCountsZout)
            y_rot = get_y_rotation(accelerometerCountsXout,accelerometerCountsYout,accelerometerCountsZout)
            
            accelerationXout = (accelerometerCountsXout - G_XcountOffset) / G_XcountsPerG
            accelerationYout = (accelerometerCountsYout - G_YcountOffset) / G_YcountsPerG
            accelerationZout = (accelerometerCountsZout - G_ZcountOffset) / G_ZcountsPerG
            
            theta_x = math.asin(max(-1,min(1,accelerationXout)))
            theta_z = math.acos(max(-1,min(1,accelerationZout)))
            
            wx = math.cos(theta_x)**2
            wz = math.sin(theta_x)**2
            
            theta_avg = math.degrees(wx*theta_x + wz*theta_z)
            
            print("X Rot: {:8.4f}    ".format(x_rot))
            print("Y Rot: {:8.4f}    ".format(y_rot))
            print("Avg Rot: {:8.4f}    ".format(theta_avg))
            time.sleep(0.5)
            if(G_keepHistory):
                print()
            else:
                print("\033[F "*4)
    except KeyboardInterrupt:
            pass
    return

# Write to a register
def setAccelScale():
    accelScales = {'0':{'description':'2g','val':0},'1':{'description':'4g','val':8},'2':{'description':'8g','val':16},'3':{'description':'16g','val':24}}
    print("Acceleration Scales:")
    for opt in sorted(list(accelScales.keys())):
        print('\t{}:\t\u00B1{}'.format(opt,accelScales[opt]['description']))
    scaleOpt = input('Select an option:')
    if len(scaleOpt) > 0:
        if scaleOpt in accelScales.keys():
            write_byte(0x68, 28, accelScales[scaleOpt]['val'])
        else:
            print("Please enter a valid option!")

# List all the valid user commands
def listCommands():
    print("Commands:")
    for index in sorted(list(options.keys())):
        print("\t{:6}-{}".format(index,options[index]['description']))
    return

# Exit the program
def quit():
    raise KeyboardInterrupt
    return

bus = smbus.SMBus(1) # bus = smbus.SMBus(0) fuer Revision 1
address = 0x68       # via i2cdetect

# Activate the MPU so the module can talk to it
bus.write_byte_data(address, power_mgmt_1, 0)

# Create a list of commands and assign each a function pointer to execute when called
options = {'h': {'exe':listCommands,'description':'Prints this menu'},
                   '1': {'exe':measureOrientation,'description':'Measure a single sensor in one orientation'},
		   '2': {'exe':continuousGyro,'description':'Read Gyro Continuously'},
		   '3': {'exe':continuousAccel,'description':'Read Acceleration Continuously'},
		   '4': {'exe':continuousRot,'description':'Read Rotation Continuously'},
		   '5': {'exe':calibrateMPU,'description':'Calibarate MPU'},
                   '6': {'exe':loadCalibration,'description':'Load a previous calibration'},
                   '7': {'exe':setAccelScale,'description':'Sets the scale of the accelerometer'},
                   '99':{'exe':toggleHistory,'description':'Sets or unsets continuous read data to show the history of data'},
                   'q': {'exe':quit,'description':'Exit Program'}}
listCommands()
try:
    while(True):
        command = input("Please choose an option: ").lower()
        # If user enters text
        if len(command) > 0:
            # If the command is valid
            if command in options.keys():
                # Run the function that is associated with the entered command
                options[command]['exe']()
            else:
                print("Please select a valid command!")
        print('\n')

except KeyboardInterrupt:
    print()
finally:
    if(len(G_calibrations) > 0):
        writeCSV()
