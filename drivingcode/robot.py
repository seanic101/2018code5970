#!/usr/bin/env python3

import wpilib
from wpilib.buttons.joystickbutton import JoystickButton
import drive


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
        self.left_motors.append(wpilib.VictorSP(2))

        self.right_motors = []
        self.right_motors.append(wpilib.VictorSP(4))
        self.right_motors.append(wpilib.VictorSP(3))
        self.right_motors.append(wpilib.VictorSP(5))

        # too lazy to change the formal from a list even though its only 1 thing.deal with it.
        # Initialize the winch Motor
        self.Winch_motor = []
        self.Winch_motor.append(wpilib.VictorSP(8))


        # Initialize the pop Motor
        self.pop_motor = []
        self.pop_motor.append(wpilib.VictorSP(7))

        #Initialize the left and right encoders
        self.Rcoder = wpilib.Encoder(2,3)
        self.Lcoder = wpilib.Encoder(0,1)


        #***************Driverstation Initialization******************
        
        #TANK DRIVE Xbox
        # Initialize the Joysticks that will be used
        self.xbox = wpilib.XboxController(1)


        #got the pinouts off of google need to test
        self.WinchUp = JoystickButton(self.xbox, 1)
        self.WinchDown = JoystickButton(self.xbox, 2)

        #self.pop = JoystickButton(self.xbox, 3)#Y
        self.pop = JoystickButton(self.xbox, 3)#Y
		
        #Initialize the Joysticks that will be used--idk what this does because im doing this @11:45 before comp, so im not deleting it
        self.throttle = wpilib.Joystick(1)
        self.steering = wpilib.Joystick(2)
    def autonomousInit(self):
        """This function is run once each time the robot enters autonomous mode."""
        self.auto_loop_counter = 0
        self.Lcoder.reset()
        self.Rcoder.reset()

    def autonomousPeriodic(self):
        #old auto code could be used to cross baseline
        print(self.auto_loop_counter)
        if self.auto_loop_counter < 10:
            self.setDriveMotors(.5, -.5)  # forward auto
            print("left encoder value "+str(self.Lcoder.get()))
            print("Right encoder value "+str(self.Rcoder.get()))
        elif self.Rcoder.get() >= 2000:
            self.setDriveMotors(0,0)
            print("got past 2000")
        self.auto_loop_counter += 1
    




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
        self.Winch()
        self.Pop()
        
    
    def testPeriodic(self):
        """This function is called periodically during test mode."""
   

    def drivetrainMotorControl(self):
            left = self.steering.getY()
            right = self.throttle.getY()
            drive_powers = drive.tankdrive(right, left)
            self.leftspeeds = drive_powers[0]
            self.rightspeeds = drive_powers[1]
            # set the motors to powers
            self.setDriveMotors(self.leftspeeds, self.rightspeeds)
        
    def setDriveMotors(self, leftspeed, rightspeed):
            for motor in self.right_motors:
                motor.set(leftspeed*-1)
            for motor in self.left_motors:
                motor.set(rightspeed*-1)


    
    def Winch(self):
        if self.WinchUp.get():
            for motor in self.Winch_motor:
                motor.set(1)
        elif self.WinchDown.get():
            for motor in self.Winch_motor:
                motor.set(-1)
        else:
            for motor in self.Winch_motor:
                motor.set(0)

    def Pop(self):
        if self.pop.get():
            for motor in self.pop_motor:
                motor.set(1)
        else:
            for motor in self.pop_motor:
                motor.set(0)

if __name__ == "__main__":
    wpilib.run(BeaverTronicsRobot)
