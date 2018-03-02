#!/usr/bin/env python3
import wpilib
import math
from wpilib.buttons.joystickbutton import JoystickButton
import drive
#import time
#import numpy as np
#import cv2

class BeaverTronicsRobot(wpilib.IterativeRobot):
    def robotInit(self):
     
        #**************Camera Initialization********************
        #Networktables.initialize('server=roborio.5970-frc.local')
        #wpilib.CameraServer.launch()
        #wpilib.CameraServer.launch('vision.py:main')
        """
        This function is called upon program startup and
        should be used for any initialization code.
        """

        #**************Robot-Side Initialization***************
        
        # Inititalize the Drive motors
        self.left_motors = []
        self.left_motors.append(wpilib.VictorSP(0))
        self.left_motors.append(wpilib.VictorSP(1))
        self.left_motors.append(wpilib.VictorSP(2))#2

        self.right_motors = []
        self.right_motors.append(wpilib.VictorSP(4))#4
        self.right_motors.append(wpilib.VictorSP(3))
        self.right_motors.append(wpilib.VictorSP(5))

        # Initialize the InCube Motor
        self.RnCube_motor = []
        self.RnCube_motor.append(wpilib.VictorSP(7))
        self.LnCube_motor = []
        self.LnCube_motor.append(wpilib.VictorSP(8))
        
        #initialize the updaisy motor
        self.Updaisy_motor = []
        self.Updaisy_motor.append(wpilib.VictorSP(9))
        
        # Initialize the Gteen Motor
        self.Gteen_motor = []
        self.Gteen_motor.append(wpilib.VictorSP(6))
        
        #initialize the ultrasonic sensor
        #self.Ultra = wpilib.AnalogInput(3)
        self.Gyroo = wpilib.ADXRS450_Gyro()
        
        
        # Initialize the pop Motor
        #self.pop_motor = []
        #self.pop_motor.append(wpilib.VictorSP(7))

        #Initialize the left and right encoders
        self.Rcoder = wpilib.Encoder(3,2)
        self.Lcoder = wpilib.Encoder(1,0)


        #***************Driverstation Initialization******************
        
        
        #self.xbox = wpilib.XboxController(1)
        
        
        #Initialize the Joysticks that will be used
        self.throttle = wpilib.Joystick(0)
        self.steering = wpilib.Joystick(1)
        self.Gteencont = wpilib.Joystick(2)

        #got the pinouts off of google need to test
        self.InCubeUp = JoystickButton(self.Gteencont,1)
        self.InCubeDown = JoystickButton(self.Gteencont, 2)
        self.Updaisy = JoystickButton(self.Gteencont, 3)
        self.Downdaisy = JoystickButton(self.Gteencont, 4)

        #self.pop = JoystickButton(self.xbox, 3)#Y
        self.pop = JoystickButton(self.steering, 1)#Y
        self.Lshift = wpilib.Solenoid(0)
        self.Rshift = wpilib.Solenoid(1)
        

    def setSetpoint(self):
        self.setpoint = 0
            # PID Values
        self.P = 1
        self.I = 0
        self.D = 0
        self.integral = 0
        self.previous_error = 0.0

    def autonomousInit(self):
        """This function is run once each time the robot enters autonomous mode."""
        self.auto_loop_counter = 0
        #print("here")
        self.Lcoder.reset()
        self.Rcoder.reset()
        self.stage = -5
        #self.cap = cv2.VideoCapture(0)
        #self.new_con = self.cap.set(11, 0.1)
        #print("new contrast " + (str(self.new_con))) #-trast
        #self.cap = cv2.VideoCapture(0)
        data = wpilib.DriverStation.getInstance().getGameSpecificMessage()
        #self.setSetpoint()
        #setSetpoint

      
    def autonomousPeriodic(self):
        #old auto code could be used to cross baseline
        #print(self.auto_loop_counter)
        #x = self.Ultra.getVoltage()
        #self.Ultra.getVoltage() == x:
        '''if self.auto_loop_counter < 10:
            self.setDriveMotors(.5, -.5)  # forward auto
            print("left encoder value "+str(self.Lcoder.get()))
            print("Right encoder value "+str(self.Rcoder.get()))
        elif self.Rcoder.get() >= 20000:
            self.setDriveMotors(0,0)
            print("got past 2000")
        if self.Rcoder.get() <= 2000:
            self.setDriveMotors(.5,-.5)
        elif self.Ultra.getVoltage() <= 1:
            if self.Rcoder.get() >= 6000:
                self.setDriveMotors(.3,-.3)'''
        #print(self.Ultra.getVoltage()
        data = wpilib.DriverStation.getInstance().getGameSpecificMessage()
        #print(data)
        
        if self.stage == -5:
            if data.find("R") == 0:
                self.stage=50
            elif data.find("L") == 0:
                self.stage=0           
        
        
        if self.stage ==0:
            self.setDriveMotors(0,0)
            self.Gyroo.calibrate()
            self.Rcoder.reset()
            self.stage =1
        
        
        #n = self.PID()
        elif self.stage ==1:
            if self.Rcoder.get() >= -7500:
                x=self.Gyroo.getAngle()
                #print("the gyro is: "+str(x))
                y=self.Rcoder.get()
                #print(y)
                print("the Left value is: "+str(y))
                #print("the Right value is: .25")
                self.setDriveMotors(-.27,.25)
            else:
                self.Rcoder.reset()
                self.setDriveMotors(0,0)
                self.stage =2 
        
        elif self.stage ==2:
            self.setDriveMotors(.45,.45)
            x=self.Gyroo.getAngle()
            print(x)
            if int(x) >= 85:
                self.Rcoder.reset()
                self.setDriveMotors(0,0)
                self.stage = 3
                
        elif self.stage ==3:
            print(self.Rcoder.get())
            if self.Rcoder.get() >= -1000:
                self.setDriveMotors(-.26,.25)
            else:
                self.Rcoder.reset()
                self.stage = 4
        
        elif self.stage ==4:
            if self.Rcoder.get() <= 1000:
                self.setDriveMotors(.26,-.25)
            else:
                self.Rcoder.reset()
                self.stage = 5
                
        elif self.stage ==5:
            self.setDriveMotors(-.25,-.25)
            x=self.Gyroo.getAngle()
            print(x)
            if int(x) <= 0:
                self.Rcoder.reset()
                self.setDriveMotors(0,0)
                self.stage = 6
        
        
        elif self.stage ==6:
            print(self.Rcoder.get())
            if self.Rcoder.get() >= -1000:
                self.setDriveMotors(-.25,.25)
            else:
                self.setDriveMotors(0,0)
                self.Rcoder.reset()
                self.stage = -1
                
                
#*************************************************************************************************
#*************************************************************************************************
#*************************************************************************************************
#*************************************************************************************************
        elif self.stage ==50:
            self.setDriveMotors(0,0)
            self.Gyroo.calibrate()
            self.Rcoder.reset()
            self.stage =51
        
        
        #n = self.PID()
        elif self.stage ==51:
            if self.Rcoder.get() >= -9000:
                x=self.Gyroo.getAngle()
                #print("the gyro is: "+str(x))
                y=self.Rcoder.get()
                #print(y)
                print("the Left value is: "+str(y))
                #print("the Right value is: .25")
                self.setDriveMotors(-.27,.25)
            else:
                self.Rcoder.reset()
                self.setDriveMotors(0,0)
                self.stage = 52
        
        elif self.stage ==52:
            self.setDriveMotors(.45,.45)
            x=self.Gyroo.getAngle()
            print(x)
            if int(x) >= 85:
                self.Rcoder.reset()
                self.setDriveMotors(0,0)
                self.stage = 53
                
        elif self.stage ==53:
            print(self.Rcoder.get())
            if self.Rcoder.get() >= -1000:
                self.setDriveMotors(-.26,.25)
            else:
                self.Rcoder.reset()
                self.stage = 54
        
        elif self.stage ==54:
            if self.Rcoder.get() <= 1000:
                self.setDriveMotors(.27,-.25)
            else:
                self.Rcoder.reset()
                self.stage = 55
                
        elif self.stage ==55:
            self.setDriveMotors(-.25,-.25)
            x=self.Gyroo.getAngle()
            print(x)
            if int(x) <= 0:
                self.Rcoder.reset()
                self.setDriveMotors(0,0)
                self.stage = 56
        
        
        elif self.stage ==56:
            print(self.Rcoder.get())
            if self.Rcoder.get() >= -5000:
                self.setDriveMotors(-.25,.25)
            else:
                self.setDriveMotors(0,0)
                self.Rcoder.reset()
                self.stage = -1








                
#*************************************************************************************************
#*************************************************************************************************
#*************************************************************************************************
#*************************************************************************************************
              
        
        
        '''elif self.stage ==4:
            self.setDriveMotors(.35,-.35)
            if self.Ultra.getVoltage() <= .7:
                self.stage =1
                self.Rcoder.reset()
                '''
        
        #print(self.cap)
            #capture an image
        #_, self.frame = self.cap.read() 
        #convert to HSV
        #cv2.imshow("frame",self.frame)
            #self.hsv = cv2.cvtColor(self.frame,cv::COLOR_BGR2HSV);
        #self.hsv = cv2.cvtColor(self.frame,cv2.COLOR_BGR2HSV)
       
        #find green pixels image using limits in in numpy arrays
        #lower_green = np.array([75,100,150]) 
        #upper_green = np.array([95,255,255])
        #self.lower_green = np.array([75,100,150]) 
        #self.upper_green = np.array([95,255,255])


        #mask filters colors out of the green range from
        # the frame being read
        #self.mask = cv2.inRange(self.hsv, self.lower_green, self.upper_green)
        #cv2.imshow('mask', mask)

        #pixelates image, does not show small detections
        #self.kernel = np.ones((5, 5), np.uint8)
        #self.erosion = cv2.erode(self.mask, self.kernel, iterations=1)
        #cv2.imshow('erosion', self.erosion)
        #cv2.waitKey(0)

        #contours the mask
        #self.image,self.contours,self.hierarchy = cv2.findContours(self.erosion,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)

        #finds largest object and contours it, saves in recordIndex
        """self.recordSize = 0
        self.recordIndex = -1
        for i in range(len(self.contours)):
            if (cv2.contourArea(self.contours[i]) > self.recordSize):
                self.recordSize = cv2.contourArea(self.contours[i])
                self.recordIndex = i
        if self.recordIndex >= 0:
            #print "hello"
            #drawContours is destructive in OpenCV <3.x.x
            cv2.drawContours(self.hsv,self.contours,self.recordIndex,(0,255,0),3)
            #boundingRect output when printed is the (x,y and w,h)...maybe...pretty sure...
            self.bound = cv2.boundingRect(self.contours[self.recordIndex])
            #print(self.bound)
                   """
        #cv2.imshow('hsv',self.hsv)
        """else:
                if self.auto_loop_counter < 20/2:
                    self.DoubleSolenoid.set(1)
                    

                else:
                    if self.auto_loop_counter < 30/2:
                        self.DoubleSolenoid.set(2)
                        self.auto_loop_counter += 1
                    else:
                            if self.auto_loop_counter < 50/2:
                                self.DoubleSolenoid.set(1)
                                self.auto_loop_counter += 1
                            else:
                                    if self.auto_loop_counter < 70/2:
                                        self.DoubleSolenoid.set(2)
                                        self.auto_loop_counter += 1
                                        self.auto_loop_counter = 0"""
                                        
    def teleopPeriodic(self):
        """This function is called periodically during operator control."""
        self.drivetrainMotorControl()#driving motors
        self.InCube()#intake/outake
        self.Pop() #shifters
        self.Gteen()#raise and lower Gteen
        self.Upsydaisy()#raise and lower intake
    
    def testPeriodic(self):
        """This function is called periodically during test mode."""

    

    def PID(self):
        """PID for angle control"""
        error = math.tan(self.setpoint - self.Gyroo.getAngle())/4 # Error = Target - Actual
        #print("error is "+str(error))
        self.integral = self.integral + (error*.02)
        #print("Int is "+str(self.integral))
        derivative = (error - self.previous_error) / .02
        print("the gyro is "+str(self.Gyroo.getAngle()))
        #print("Der is "+str(derivative))
        self.rcw = self.P*error + self.I*self.integral + self.D*derivative
        #self.rcw = .25 + self.I*self.integral + self.D*derivative
        print(str(self.rcw)+" = "+str(self.P*error)+" + "+str(self.I*self.integral)+" + "+str(self.D*derivative))
        #print(str(self.rcw)+" = "+str(.25)+" + "+str(self.I*self.integral)+" + "+str(self.D*derivative))
        #print("Motor of the right is "+str(self.rcw))
        n = self.rcw
        return n

    def Gteen(self):
        value = self.Gteencont.getY()
        for motor in self.Gteen_motor:
            motor.set(value)
            #print(value)
    
    def GteenAuto(self,needed):
        for motor in self.Gteen_motor:
         motor.set(needed)
    
    def drivetrainMotorControl(self):
            right = self.steering.getY()
            left = self.throttle.getY()
            drive_powers = drive.tankdrive(right*-1, left*-1)
            self.leftspeeds = drive_powers[0]
            self.rightspeeds = drive_powers[1]
            # set the motors to powers
            self.setDriveMotors(self.leftspeeds, self.rightspeeds)
        
    def setDriveMotors(self, leftspeed, rightspeed):
            for motor in self.right_motors:
                motor.set(leftspeed*-1)
            for motor in self.left_motors:
                motor.set(rightspeed*-1)

    def InCube(self):#intake function
        if self.InCubeUp.get():
            for motor in self.LnCube_motor:
                motor.set(1)
            for motor in self.RnCube_motor:
                motor.set(-1)
        elif self.InCubeDown.get():
            for motor in self.LnCube_motor:
                motor.set(-1)
            for motor in self.RnCube_motor:
                motor.set(1)
        else:
            for motor in self.LnCube_motor:
                motor.set(0)
            for motor in self.RnCube_motor:
                motor.set(0)
                
    def Upsydaisy(self):#intake function
        if self.Updaisy.get():
            for motor in self.Updaisy_motor:
                motor.set(1)
        elif self.Downdaisy.get():
            for motor in self.Updaisy_motor:
                motor.set(-1)
        else:
            for motor in self.Updaisy_motor:
                motor.set(0)

    def Pop(self):
        if self.pop.get():
            self.Lshift.set(True)
            self.Rshift.set(True)
            #for motor in self.pop_motor:
                #motor.set(1)
        else:
            self.Lshift.set(False)
            self.Rshift.set(False)
            #for motor in self.pop_motor:
                #motor.set(0)

if __name__ == "__main__":
    wpilib.run(BeaverTronicsRobot)
