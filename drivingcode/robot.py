#!/usr/bin/env python3
import wpilib
import math
from wpilib.buttons.joystickbutton import JoystickButton
import drive
import os
import sys
here = os.path.dirname(os.path.realpath(__file__))
sys.path.insert(0, here + '/../vision/tcp')
sys.path.insert(0, here + '/../vision')
#Windows path
#sys.path.insert(0,"C:/Users/Beavertronics/Desktop/2018Workstation/2018code5970/vision/tcp")
import socket
import json
from time import sleep
from server import parse, decode_json
import re
import argparse

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
        self.RnCube_motor.append(wpilib.Spark(6))
        self.LnCube_motor = []
        self.LnCube_motor.append(wpilib.Spark(7))
        
        #initialize the updaisy motor
        self.Updaisy_motor = []
        self.Updaisy_motor.append(wpilib.VictorSP(9))
        
        # Initialize the Gteen Motor
        self.Gteen_motor = []
        self.Gteen_motor.append(wpilib.VictorSP(8))
        
        #initialize the ultrasonic sensor
        #self.Ultra = wpilib.AnalogInput(3)
        self.Gyroo = wpilib.ADXRS450_Gyro()
        
        
        # Initialize the pop Motor
        #self.pop_motor = []
        #self.pop_motor.append(wpilib.VictorSP(7))

        #Initialize the left and right encoders
        #self.Rcoder = wpilib.Encoder(2,3)
        self.Lcoder = wpilib.Encoder(0,1)
        self.Gcoder = wpilib.Encoder(4,5)
        self.Wcoder = wpilib.Encoder(2,3)


        #***************Driverstation Initialization******************
        
        
        #self.xbox = wpilib.XboxController(1)
        
        
        #Initialize the Joysticks that will be used
        self.throttle = wpilib.Joystick(0)
        self.steering = wpilib.Joystick(1)
        self.Gteencont = wpilib.Joystick(2)

        self.InCubeUp = JoystickButton(self.Gteencont,3)
        self.SeanInCubeUp =JoystickButton(self.throttle,3)
        #self.SeanInCubeDown =JoystickButton(self.throttle,4)
        #self.InCubeDown = JoystickButton(self.Gteencont, 4)
        self.Updaisy = JoystickButton(self.Gteencont, 5)
        self.Downdaisy = JoystickButton(self.Gteencont, 6)
        self.UpClimber = JoystickButton(self.Gteencont, 1)
        #self.pop = JoystickButton(self.xbox, 3)#Y
        self.pop = JoystickButton(self.steering, 5)#Y
        self.Lshift = wpilib.Solenoid(0)
        self.Rshift = wpilib.Solenoid(1)
        self.Lpiston = wpilib.Solenoid(2)
        self.Rpiston = wpilib.Solenoid(3)
        self.pneumaL=JoystickButton(self.steering, 1)
        self.pneumaR=JoystickButton(self.throttle, 1)
    
    def drive_forward(self,distance,direction):
        self.Lcoder.reset()
        if direction =="Forward":
            while self.Lcoder.get()<distance:
                self.setDriveMotors(.5, -.5)
            self.setDriveMotors(0, 0)
        elif direction =="Backward":
            while self.Lcoder.get()>distance:
                self.setDriveMotors(-.5, .5)
            self.setDriveMotors(0, 0)
    
    def turn(self,degrees):
        self.Gyroo.reset()
        if degrees >=0:#if it's turning left
            while self.Gyroo.getAngle()()<=degrees:
                self.setDriveMotors(-.25, -.25)#assuming it's turning left here
            self.setDriveMotors(0, 0)
            return 0
        if degrees <=0:#if it's turning right
            while self.Gyroo.getAngle()()>=degrees:#expecting a negative value for this side
                self.setDriveMotors(.25,.25)#assuming it's turning right here
            self.setDriveMotors(0, 0)
        
    def find_tape(self):
        while self.Gyroo.getAngle()<=360:#im assuming it's going to count up from 0 to 360
            self.setDriveMotors(-.25, -.25)
            deg,asmith,dist= self.distance_to_tape()
            if dist != -1:
                self.setDriveMotors(0, 0)
                return deg,asmith,dist
        self.setDriveMotors(0, 0)
        return -1
    
    
    
    
    
    def distance_to_tape(self):
        #lolly's code here
        #Jetson path
        here = os.path.dirname(os.path.realpath(__file__))
        sys.path.insert(0, here + '/../../vision/tcp')

        #Windows path
        #sys.path.insert(0,"C:/Users/Beavertronics/Desktop/2018Workstation/2018code5970/vision/tcp")
        #import socket
        #import json
        #from time import sleep
        #from server import parse, decode_json
        #import re
        #import argparse

        self.PY2 = sys.version_info[0] == 2

        self.MSG_DEFAULT = "shutdown:" + json.dumps({}, ensure_ascii=False)
        self.LOC_DEFAULT = "locate_tape:" + json.dumps({}, ensure_ascii=False)
        self.RESET_DEFAULT = "reset_tape:"  + json.dumps({}, ensure_ascii=False)
        self.DEBUG_ON_DEFAULT = "debug_on:"  + json.dumps({}, ensure_ascii=False)
        self.DEBUG_DEFAULT = "debug_on:" + json.dumps({'filename':'/tmp/debugout'}, ensure_ascii=False)

        self.TCP_IP = '10.59.70.12'
        self.TCP_PORT = 5005
        self.BUFFER_SIZE = 1024

        self.parser = argparse.ArgumentParser(description="Beavertronics Jetson TX1 client")

        self.parser.add_argument('-d', '--debug', action='store_true')
        self.parser.add_argument('--sim', action='store_true')
        print(sys.argv)
        self.args = self.parser.parse_args
        if self.args.sim:
            sys.argv=['robot.py', 'sim']

        if self.args.debug:
            print("Connecting to server on localhost...")
            self.TCP_IP = '127.0.0.1'

        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.connect((self.TCP_IP, self.TCP_PORT))

        if self.args.debug:
            print("Connection to local host established")

        try:

            if self.PY2:
                s.send(self.RESET_DEFAULT)
            else:
                s.send(bytes(self.RESET_DEFAULT, 'utf-8'))

            self.cmd, self.json_data = parse(s.recv(self.BUFFER_SIZE))
            if self.args.debug:
                print("Got json_data: <" + str(self.json_data) + ">")

            while 1:
                sleep(0.1)
                if self.PY2:
                    s.send(self.LOC_DEFAULT)
                else:
                    s.send(bytes(self.LOC_DEFAULT, 'utf-8'))

                self.cmd, self.json_data = parse(s.recv(self.BUFFER_SIZE))
                if self.PY2:
                    self.tmp =  self.json_data
                else:
                    self.tmp =  self.bytes_decode(self.json_data)
                    
                self.degrees, self.azim, self.distance = decode_json(self.tmp)
                return self.degrees, self.azim, self.distance
                #print("client received loc data: <"+ str(self.degrees) + " " + str(self.azim) + " " + str(self.distance) + ">")
        #KeyboardInterrupt is Ctrl-C
        except KeyboardInterrupt:
            print('interrupted')

        s.send(MSG_DEFAULT)
        self.json_data = s.recv(BUFFER_SIZE)
        s.close()

        if self.PY2:
            self.tmp =  self.json_data
        else:
            self.tmp =  bytes_decode(json_data)
        print("client received shutdown data:" + self.tmp)

        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
    def pinser(self,state):
        self.auto_loop_counter = 0
        while self.auto_loop_counter <= 50:
            self.Lpiston.set(False)
            self.Rpiston.set(False)
            self.auto_loop_counter= self.auto_loop_counter+1
        self.Lpiston.set(True)
        self.Rpiston.set(True)
        return 0
    def wrist(self,state):#must get maximum and minimum
        max_wrist=90#set low so it will stop before it breaks itself at 100
        min_wrist=10#set high so it will stop before it breaks itself at 0
        if direction =="Forward":
            while self.Wcoder.get()<max_wrist:
                for motor in self.LnCube_motor:
                    motor.set(.25)
            for motor in self.LnCube_motor:
                    motor.set(0)
        elif direction =="Backward":
            while self.Wcoder.get()<max_wrist:
                for motor in self.LnCube_motor:
                    motor.set(-.25)
            for motor in self.LnCube_motor:
                    motor.set(0)


  
 
    def autonomousInit(self):
        """This function is run once each time the robot enters autonomous mode."""
        self.auto_loop_counter = 0
        self.Lcoder.reset()
        #self.Rcoder.reset()
        self.Gcoder.reset()
        self.stage = 0
        data = wpilib.DriverStation.getInstance().getGameSpecificMessage()
        
        
    def autonomousPeriodic(self):
        self.find_tape()
        #print("made it")
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
        
        if self.stage == -1:
            if data.find("R",0,0) == 0:
                    self.stage=0 
            elif data.find("L",0,0) == 0:
                self.stage=1000
        
        
        #**************************************************
        # For left side box run
        #**************************************************
        '''if self.stage ==0:
            self.setDriveMotors(0,0)
            self.Gyroo.calibrate()
            self.Rcoder.reset()
            self.stage =1
        
        
        elif self.stage ==1:
            if self.Rcoder.get() <= 7100:
                x=self.Gyroo.getAngle()
                #print("the gyro is: "+str(x))
                y=self.Rcoder.get()
                #print(y)
                print("the Left value is: "+str(y))
                #print("the Right value is: .25")
                self.setDriveMotors(-.25,.25)
            else:
                self.Rcoder.reset()
                self.setDriveMotors(0,0)
                self.stage =2 
        
        elif self.stage ==2:
            self.setDriveMotors(.45,.45)
            x=self.Gyroo.getAngle()
            print(x)
            if int(x) >= 90:
                self.Rcoder.reset()
                self.setDriveMotors(0,0)
                self.stage = 3
                
        elif self.stage ==3:
            print(self.Rcoder.get())
            if self.Rcoder.get() <= 1000:
                self.setDriveMotors(-.25,.25)
            else:
                self.Rcoder.reset()
                self.auto_loop_counter = 0
                self.stage = 4
        elif self.stage ==4:
            if self.auto_loop_counter < 100:
                self.AutoInCube(1)
            else:
                self.AutoInCube(0)
                self.stage =4.5
                
        elif self.stage ==4.5:
            if self.Rcoder.get() >= -1000:
                self.setDriveMotors(.25,-.25)
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
            if self.Rcoder.get() <= 1000:
                self.setDriveMotors(-.25,.25)
            else:
                self.setDriveMotors(0,0)
                self.Rcoder.reset()
                self.stage = -1
                
                
#*************************************************************************************************
#for right side gear(wont work)
#*************************************************************************************************
        elif self.stage ==50:
            self.setDriveMotors(0,0)
            self.Gyroo.calibrate()
            self.Rcoder.reset()
            self.stage =51
        
        
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
                self.stage = 99
        
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
            if self.Rcoder.get() >= -5000:
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
                self.stage = 99
#*************************************************************************************************
#for LEFT side box ON THE SWITCH(wont work)
#*************************************************************************************************

        if self.stage ==150:
                    self.setDriveMotors(0,0)
                    self.Gyroo.calibrate()
                    self.Rcoder.reset()
                    self.stage =151
        
        
        elif self.stage ==151:
            if self.Rcoder.get() >= -5000:
                y=self.Rcoder.get()
                #print(y)
                #print("the Left value is: "+str(y))
                self.setDriveMotors(-.27,.25)
            else:
                self.Rcoder.reset()
                self.setDriveMotors(0,0)
                self.stage =152 
        
        elif self.stage ==152:#get back to 0 
            x=self.Gyroo.getAngle()
            print(x)
            if int(x) == 0:
                self.Rcoder.reset()
                self.setDriveMotors(0,0)
                self.stage = 153
            elif int(x) > 0:
                self.setDriveMotors(-.25,-.25)
            elif int(x) < 0:
                self.setDriveMotors(.25,.25)

        elif self.stage == 153:        
            if self.Rcoder.get() >= -5000:
                y=self.Rcoder.get()
                #print(y)
                #print("the Left value is: "+str(y))
                self.setDriveMotors(-.27,.25)
            else:
                self.Rcoder.reset()
                self.setDriveMotors(0,0)
                self.stage =154

        elif self.stage ==154:#get back to 0 
            x=self.Gyroo.getAngle()
            print(x)
            if int(x) == 0:
                self.Rcoder.reset()
                self.setDriveMotors(0,0)
                self.stage = 99
            elif int(x) > 0:
                self.setDriveMotors(-.25,-.25)
            elif int(x) < 0:
                self.setDriveMotors(.25,.25)'''

        if self.stage ==0:
            self.setDriveMotors(0,0)
            self.Gyroo.calibrate()
            #self.Rcoder.reset()
            self.stage =1
            self.auto_loop_counter = 0
        

        elif self.stage ==1:
            if self.auto_loop_counter <= 250:
                self.setDriveMotors(-.25,.25)
                self.auto_loop_counter= self.auto_loop_counter+1
            else:
                self.setDriveMotors(0,0)
                self.stage =2           
        #elif self.stage ==2:
         #   x=self.Gyroo.getAngle()
          #  print(x)
           # if int(x) <= 87:
            #    self.setDriveMotors(.35,.35)
            #else:
             #   self.setDriveMotors(0,0)
              #
            #  self.stage = 3
        '''elif self.stage ==3:
            x=self.Gyroo.getAngle()
            print(x)
            if int(x) >= -87:
                self.setDriveMotors(-.35,-.35)
            else:
                self.setDriveMotors(0,0)
                self.stage = 4'''

                
#*************************************************************************************************
#*************************************************************************************************
#*************************************************************************************************
#*************************************************************************************************
        if self.stage ==1000:
            self.setDriveMotors(0,0)
            self.Gyroo.calibrate()
            #self.Rcoder.reset()
            self.stage =1001
            self.auto_loop_counter = 0
        

        elif self.stage ==1001:
            if self.auto_loop_counter <= 250:
                self.setDriveMotors(-.25,.25)
                self.auto_loop_counter= self.auto_loop_counter+1
            else:
                self.setDriveMotors(0,0)
                self.stage =1002           
        elif self.stage ==1002:
            x=self.Gyroo.getAngle()
            print(x)
            if int(x) <= 87:
                self.setDriveMotors(.35,.35)
            else:
                self.setDriveMotors(0,0)
                self.stage = 1003
        elif self.stage ==1003:
            x=self.Gyroo.getAngle()
            print(x)
            if int(x) >= -87:
                self.setDriveMotors(-.35,-.35)
            else:
                self.setDriveMotors(0,0)
                self.stage = 10
        
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
        self.Pneuma()
        print("left encoder value "+str(self.Lcoder.get()))
        #print("right encoder value "+str(self.Rcoder.get()))
    
    def testPeriodic(self):
        """This function is called periodically during test mode."""



    def Gteen(self):
        value = self.Gteencont.getY()#405 is the desired max on practice bot
        turning = self.Gcoder.get()#405 is the desired max
        '''if -1*value <= 0:
            print("falling at "+str(value))
            if turning >= 300:
                for motor in self.Gteen_motor:
                    motor.set(-.05)#fall fast
            elif turning >= 200:
                for motor in self.Gteen_motor:
                    motor.set(-.05)#fall medium
            elif turning >= 10:
                for motor in self.Gteen_motor:
                    motor.set(-.05)#fall 
            elif turning >= 5:
                for motor in self.Gteen_motor:
                    motor.set(0)
                    
                    
                    #fall slow
            #check if its high up or low up
            # if top 2% then fall fast(because it should be resting set output to 0)
            
            
        elif -1*value > .25:
            print("rising at "+str(value))
            if turning <= 100:
                for motor in self.Gteen_motor:
                    motor.set(-.8)#rise fast
            elif turning <= 200:
                for motor in self.Gteen_motor:
                    motor.set(-.6)#rise medium
            elif turning <= 380:
                for motor in self.Gteen_motor:
                    motor.set(-.6)#rise slow 
            elif turning <= 400:
                for motor in self.Gteen_motor:
                    motor.set(-0.3)
        else:
            for motor in self.Gteen_motor:
                motor.set(-0.3)'''
        
        print("The G encoder is at: "+str(turning))
        for motor in self.Gteen_motor:
            motor.set(value)
        print("the value for the motor power "+str(value))
    
    def AutoInCube(self,what):#intake function
        if self.InCubeUp.get():
            if what == 1:
                for motor in self.LnCube_motor:
                    motor.set(1)
                for motor in self.RnCube_motor:
                    motor.set(1)
            else:
                for motor in self.LnCube_motor:
                    motor.set(0)
                for motor in self.RnCube_motor:
                    motor.set(0)
    
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
        if self.InCubeUp.get() or self.SeanInCubeUp.get():
            for motor in self.LnCube_motor:
                motor.set(2.003)
            for motor in self.RnCube_motor:
                motor.set(-2.003)
        #elif self.InCubeDown.get() or self.SeanInCubeDown.get():
            #for motor in self.LnCube_motor:
                #motor.set(-2.003)
            #for motor in self.RnCube_motor:
                #motor.set(2.003)
        else:
            for motor in self.LnCube_motor:
                motor.set(0)
            for motor in self.RnCube_motor:
                motor.set(0)
                
    def Upsydaisy(self):#intake winch function
        if self.Updaisy.get():
            for motor in self.Updaisy_motor:
                motor.set(1.0)
        elif self.Downdaisy.get():
            for motor in self.Updaisy_motor:
                motor.set(-1)
        else:
            for motor in self.Updaisy_motor:
                motor.set(0)
                
    def Climber(self):#Climber winch function
        if self.UpClimber.get():
            for motor in self.UpClimber_motor:
                motor.set(1)
        else:
            for motor in self.UpClimber_motor:
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
                
                
    def Pneuma(self):
        if self.pneumaL.get():
            self.Lpiston.set(False)
        else:
            self.Lpiston.set(True)
        if self.pneumaR.get():
            self.Rpiston.set(False)
        else:
            self.Rpiston.set(True)

if __name__ == "__main__":
    wpilib.run(BeaverTronicsRobot)
