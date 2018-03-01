#!/usr/bin/env python3
import wpilib
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
        self.InCube_motor = []
        self.InCube_motor.append(wpilib.VictorSP(6))
        
        # Initialize the Gteen Motor
        self.Gteen_motor = []
        self.Gteen_motor.append(wpilib.VictorSP(7))
        
        #initialize the ultrasonic sensor
        self.Ultra = wpilib.AnalogInput(3)
        self.Gyroo = wpilib.ADXRS450_Gyro()
        
        
        # Initialize the pop Motor
        #self.pop_motor = []
        #self.pop_motor.append(wpilib.VictorSP(7))

        #Initialize the left and right encoders
        self.Rcoder = wpilib.Encoder(3,2)
        self.Lcoder = wpilib.Encoder(1,0)


        #***************Driverstation Initialization******************
        
        #TANK DRIVE Xbox
        # Initialize the Joysticks that will be used
        self.xbox = wpilib.XboxController(1)


        #got the pinouts off of google need to test
        self.InCubeUp = JoystickButton(self.xbox, 1)
        self.InCubeDown = JoystickButton(self.xbox, 2)

        #self.pop = JoystickButton(self.xbox, 3)#Y
        self.pop = JoystickButton(self.xbox, 3)#Y
		
        #Initialize the Joysticks that will be used
        self.throttle = wpilib.Joystick(0)
        self.steering = wpilib.Joystick(1)
        
    def autonomousInit(self):
        """This function is run once each time the robot enters autonomous mode."""
        self.auto_loop_counter = 0
        #print("here")
        self.Lcoder.reset()
        self.Rcoder.reset()
        self.stage = 0
        #self.cap = cv2.VideoCapture(0)
        #self.new_con = self.cap.set(11, 0.1)
        #print("new contrast " + (str(self.new_con))) #-trast
        #self.cap = cv2.VideoCapture(0)
        data = wpilib.DriverStation.getInstance().getGameSpecificMessage()        
      
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
        #data = wpilib.DriverStation.getInstance().getGameSpecificMessage()
        #print(data)
        
        '''if len(data) > 0:
            if data.find("R") == 0:
                self.setDriveMotors(1,1)
            elif data.find("L") == 0:
                self.setDriveMotors(-1,-1)'''            
        
        
        if self.stage ==0:
            self.setDriveMotors(0,0)
            self.Gyroo.calibrate()
            self.stage =1
        
        
        
        if self.stage ==1:
            if self.Rcoder.get() >= -1000:
                x=self.Gyroo.getAngle()
                print("the gyro is: "+str(x))
                print("the Left value is: "+str(-.25-int(x)*.5))
                print("the Right value is: .25")
                self.setDriveMotors(-.25-int(x)*.5,.25)
            else:
                self.Rcoder.reset()
                self.stage = 2
        
        elif self.stage ==3:
            self.setDriveMotors(.45,.45)
            x=self.Gyroo.getAngle()
            print(x)
            if int(x) >= 90:
                self.Rcoder.reset()
                self.stage = 3
                
        elif self.stage ==3:
            print(self.Rcoder.get())
            if self.Rcoder.get() >= -3000:
                self.setDriveMotors(-.65,.65)
            else:
                self.Rcoder.reset()
                self.stage = 4
        
        elif self.stage ==4:
            if self.Rcoder.get() <= 2000:
                self.setDriveMotors(.75,-.75)
            else:
                self.Rcoder.reset()
                self.stage = 5
                
        
        
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
        self.drivetrainMotorControl()
        self.InCube()
        #self.Pop()
    
    def testPeriodic(self):
        """This function is called periodically during test mode."""
   

    
	# PID Values
        self.P = 1
        self.I = 1
        self.D = 1

        self.integral = 0
        self.previous_error = 0


    def setSetpoint(self, setpoint):
        self.setpoint = setpoint

    def PID(self):
        """PID for angle control"""
        error = self.setpoint - self.gyro.getAngle() # Error = Target - Actual
        self.integral = integral + (error*.02)
        derivative = (error - self.previous_error) / .02
        self.rcw = self.P*error + self.I*self.integral + self.D*derivative

	
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


    
    def InCube(self):
        if self.InCubeUp.get():
            for motor in self.InCube_motor:
                motor.set(1)
        elif self.InCubeDown.get():
            for motor in self.InCube_motor:
                motor.set(-1)
        else:
            for motor in self.InCube_motor:
                motor.set(0)

    #def Pop(self):
        #if self.pop.get():
            #for motor in self.pop_motor:
                #motor.set(1)
        #else:
            #for motor in self.pop_motor:
                #motor.set(0)

if __name__ == "__main__":
    wpilib.run(BeaverTronicsRobot)
