/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.solarmxli.larrycapucha.v2_0;


import edu.wpi.first.wpilibj.AnalogChannel;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Gyro;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Jaguar;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class LarryCapucha extends IterativeRobot {

    
    /*
     * Public variables declaration.
     */
    Joystick shootingStick, //Joystick used to shoot and aim.
            drivingStick; //Joystick used to drive.
    Victor aimingMotor, //Motor that moves the sooter up and down.
            feederMotor; //Feeder motor.
    Button trigger, //Button to shoot.
            shooterButton,
            feedButton; //Button to turn on the shooter motor.
    Servo triggerMotor; //Servo moved by the trigger to shoot.
    Jaguar shooterMotor; //Shooter motor.
    RobotDrive driver; //To drive the robot.
    DigitalInput backSensor, //Back sensor of the shooter.
            frontSensor, //Front sensor of the shooter.
            freesbieSensor; //Activated when the robot has a freesbie
    boolean hasDisk = false, regreso = true, canContinue = true; //True if has a disk, false if else.
    Gyro gyro; //A gyroscope in the shooter.
    Thread shoot, drive, feed, autoCheck; //Threads for teleop mode.
    
    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    public void robotInit() {
        shootingStick = new Joystick(1);
        drivingStick = new Joystick(2);
        
        aimingMotor = new Victor(4);
        feederMotor = new Victor(3);
        
        trigger = new JoystickButton(shootingStick, 1);
        shooterButton = new JoystickButton(shootingStick, 2);
        feedButton = new JoystickButton(shootingStick, 6);
        
        triggerMotor = new Servo(9);
        shooterMotor = new Jaguar(5);
        
        backSensor = new DigitalInput(3);
        freesbieSensor = new DigitalInput(2);
        frontSensor = new DigitalInput(1);
        
        gyro = new Gyro(new AnalogChannel(1));
        
    }
    
    /**
     * This function is called when autonomous mode start.
     */
    public void autonomousInit() {
        
        /*autoCheck = new Thread(autonomousCheck);
        autoCheck.start();
        
        while(canContinue) {
            aimingMotor.set(-1);
        }
        aimingMotor.set(0);
        
        shooterMotor.set(-4.75);
        Timer.delay(5);
        
        
        
        triggerMotor.set(75);
        Timer.delay(1);
        triggerMotor.set(0);
        triggerMotor.set(75);
        Timer.delay(1);
        triggerMotor.set(0);
        
        autoCheck.interrupt*/
        
    }

    /**
     * This function is called periodically during autonomous
     */
    public void autonomousPeriodic() {
        
    }
    
    /**
     * This function is called when teleoperated mode start.
     */
    public void teleopInit() {
        
        shooterMotor.set(0);
        driver = new RobotDrive(2,1);
        /*while(!frontSensor.get()) {
            aimingMotor.set(1);
        }
        gyro.reset();*/
        
        drive = new Thread(driveJob);
        shoot = new Thread(shootJob);
        //feed = new Thread(feedJob);
        
        drive.start();
        shoot.start();
        //feed.start();
    }

    /**
     * This function is called periodically during operator control
     */
    public void teleopPeriodic() {
        
    }
    
    /**
     * This function is called periodically during test mode
     */
    public void testPeriodic() {
        
    }
    
    /*
     * Thread to drive.
     */
    Runnable driveJob = new Runnable() {
        public void run() {
            while(true) {
                driver.arcadeDrive(drivingStick);
            }
        }
    };
    
    /*
     * Thread to aim and shoot.
     */
    Runnable shootJob = new Runnable() {
        public void run() {
            while(true) {
                if(shootingStick.getY() > .3 && !frontSensor.get()) {
                    aimingMotor.set(1);
                } else if(shootingStick.getY() < -.3 && backSensor.get()) { //gyro.getAngle() < 24
                    aimingMotor.set(-1);
                    System.out.println(gyro.getAngle());
                } else {
                    aimingMotor.set(0);
                }
                
                if(shooterButton.get()) {
                    shooterMotor.set(-.6);
                } else {
                    shooterMotor.set(-.6);
                }
                
                if(trigger.get()&& shooterButton.get() ) {
                    triggerMotor.set(75);
                    regreso = false;
                    hasDisk = false;
                } else {
                    triggerMotor.set(0);
                    Timer.delay(.2);
                    regreso = true;
                }
                
                if(feedButton.get()) {
                    feederMotor.set(.5);
                    Timer.delay(.2);
                    feederMotor.set(0);
                }
                
            }
        }
    };
    
    /*
     * Thread to feed.
     */
    Runnable feedJob = new Runnable() {
        public void run() {
            while(true) {
                if(!hasDisk && !freesbieSensor.get() && regreso) {
                    feederMotor.set(.8);
                    Timer.delay(1);
                    hasDisk = true;
                    feederMotor.set(0);
                } else {
                    feederMotor.set(0);
                }
            }
        }
    };
    
    Runnable autonomousCheck = new Runnable() {

        public void run() {
            while (true) {
                if(!backSensor.get()) {
                    canContinue = false;
                }
            }
        }
    };
    
    
}
