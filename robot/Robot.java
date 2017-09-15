package org.usfirst.frc.team6493.robot;

import com.ctre.CANTalon;
import com.ctre.CANTalon.FeedbackDevice;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.RobotDrive.MotorType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.*;

public class Robot extends IterativeRobot {
	CANTalon frontLeft, rearLeft, frontRight, rearRight;
	Spark rope1 = new Spark(0);
	Spark rope2 = new Spark(1);
	RobotDrive robot;
	ADXRS450_Gyro gyro = new ADXRS450_Gyro();
	Joystick stick = new Joystick(0);
	Joystick gamepad = new Joystick(1);
	Timer timer = new Timer();
	Solenoid ballRelease = new Solenoid(0,0);
	Solenoid armLift = new Solenoid(0,5);
	Solenoid armClamp = new Solenoid(0,6);
	Solenoid mecanumShift = new Solenoid(0,7);
	boolean mecanumDrive = true;
	boolean creepMode = false;
	boolean armDown = false;
	boolean armOpen = false;
	boolean release = false;
	boolean end = false;
	boolean leftPath,centerPath,rightPath,gyroTest,encTest,redBoiler,blueBoiler,baseline,armOverride;
	double xDrive,yDrive,turnDrive, parameters;
	double gs = 1.075;
	double gcs = -0.03;
	double botlength = 36;
	int ticksPerRotation = 256;

	@Override
	public void robotInit() {
		gyro.calibrate();
		CameraServer.getInstance().startAutomaticCapture();
		CameraServer.getInstance().addAxisCamera("axis-camera.local");
		
		
		frontLeft = new CANTalon(50);
		rearLeft = new CANTalon(52);
		frontRight = new CANTalon(53);
		rearRight = new CANTalon(51);
		robot = new RobotDrive(frontLeft,rearLeft,frontRight,rearRight);
		robot.setSafetyEnabled(false);
		
		//The encoders are plugged into the front left and front right talons (the front/rear and left/right is based on what motor they're plugged into)
		frontLeft.setFeedbackDevice(FeedbackDevice.EncRising);
		frontRight.setFeedbackDevice(FeedbackDevice.EncRising);
		
	}
	

	@Override
	public void autonomousInit() {
		robot.setInvertedMotor(MotorType.kFrontLeft,false);
		robot.setInvertedMotor(MotorType.kRearLeft,false);
		mecanumShift.set(false);
		armLift.set(false);
		armClamp.set(false);
		gyro.reset();
		leftPath = SmartDashboard.getBoolean("DB/Button 0",false);
		centerPath = SmartDashboard.getBoolean("DB/Button 1",false);
		rightPath = SmartDashboard.getBoolean("DB/Button 2",false);
		redBoiler = SmartDashboard.getBoolean("DB/Button 3",false);
		blueBoiler = SmartDashboard.getBoolean("DB/Button 4",false);
		gyroTest = SmartDashboard.getBoolean("DB/Button 5",false);
		encTest = SmartDashboard.getBoolean("DB/Button 6",false);
		baseline = SmartDashboard.getBoolean("DB/Button 7",false);
		end = false;
	}
	@Override
	public void autonomousPeriodic() {
		if(!end){
			if(leftPath&&!centerPath&&!rightPath&&!redBoiler&&!blueBoiler&&!gyroTest&&!encTest&&!baseline){
				System.out.println("Left path selected.");
				driveForward(110);
				turnRight(60);
				driveForward(32);
				clampOpen(2);
				driveBackward(24);
				clampClose();
			}else if(!leftPath&&centerPath&&!rightPath&&!redBoiler&&!blueBoiler&&!gyroTest&&!encTest&&!baseline){
				System.out.println("Center path selected.");
				driveForward(114-botlength);
				clampOpen(2);
				driveBackward(24);
				clampClose();
			}else if(!leftPath&&!centerPath&&rightPath&&!redBoiler&&!blueBoiler&&!gyroTest&&!encTest&&!baseline){
				System.out.println("Right path selected.");
				System.out.println("Driving forward");
				driveForward(110);
				System.out.println("Turning");
				turnLeft(60);
				System.out.println("Driving forward again");
				driveForward(32);
				clampOpen(2);
				driveBackward(24);
				clampClose();
			}else if(!leftPath&&!centerPath&&!rightPath&&redBoiler&&!blueBoiler&&!gyroTest&&!encTest&&!baseline){
				System.out.println("Red boiler path selected.");
				driveForward(50);
				turnLeft(135);
				ballRelease(5);
				driveForward(49);
				turnLeft(45);
				driveForward(60);
			}else if(!leftPath&&!centerPath&&!rightPath&&!redBoiler&&blueBoiler&&!gyroTest&&!encTest&&!baseline){
				System.out.println("Blue boiler path selected.");
				driveForward(50);
				turnRight(135);
				driveBackward(49);
				ballRelease(5);
				driveForward(49);
				turnLeft(45);
				driveForward(60);
			}else if(!leftPath&&!centerPath&&!rightPath&&!redBoiler&&!blueBoiler&&gyroTest&&!encTest&&!baseline){
				System.out.println("Gyro test selected. The robot will now rotate 360 degrees clockwise.");
				turnRight(360);
			}else if(!leftPath&&!centerPath&&!rightPath&&!redBoiler&&!blueBoiler&&!gyroTest&&encTest&&!baseline){
				System.out.println("Encoder test selected. The robot will now move 3 feet forward.");
				driveForward(126-botlength);
			}else if(!leftPath&&!centerPath&&!rightPath&&!redBoiler&&!blueBoiler&&!gyroTest&&!encTest&&baseline){
				System.out.println("Baseline path selected.");
				driveForward(148);
			}else{
				System.out.println("Multiple paths (or no paths) selected. The robot will not move.");
				robot.drive(0,0);
			}
			end = true;
		}
	}
	@Override
	public void teleopInit() {
		end = true;
		timer.start();
		mecanumDrive = false;
		frontLeft.setEncPosition(0);
		frontRight.setEncPosition(0);
		armDown = false;
		armOpen = false;
		release = false;
	}
	@Override
	public void teleopPeriodic() {
		//Drive shift toggle
		//Timer is used so that holding the button down for more than a couple of milliseconds doesn't cause rapid toggling
		if(timer.get()>1){
			if(stick.getRawButton(4)){
				timer.reset();
				timer.start();
				if(mecanumDrive){
					mecanumDrive = false;
				}else{
					mecanumDrive = true;
				}
			}
		}
		//Creep mode toggle
		if(timer.get()>1){
			if(stick.getRawButton(3)){
				timer.reset();
				timer.start();
				if(creepMode){
					creepMode = false;
				}else{
					creepMode = true;
				}
			}
		}
		//Arm lift toggle
		if(timer.get()>1){
			if(gamepad.getRawButton(1)){
				timer.reset();
				timer.start();
				if(armDown){
					armDown = false;
				}else{
					armDown = true;
					mecanumDrive = false;
				}
			}
		}
		//Ball release toggle
		if(timer.get()>1){
			if(gamepad.getRawButton(2)){
				timer.reset();
				timer.start();
				if(release){
					release = false;
				}else{
					release = true;
				}
			}
		}
		//Arm clamp
		if(gamepad.getRawAxis(2)>0.5){
			armOpen = true;
		}else{
			armOpen = false;
		}
		if(armOverride){
			armLift.set(false);
		}
		armLift.set(armOverride||armDown);
		armClamp.set(armOpen);
		ballRelease.set(release);
		//Rope motors
		if(stick.getRawButton(5)){
			rope1.set(1);
		}else{
			rope1.set(0);
		}
		if(stick.getRawButton(6)){
			rope2.set(-1);
		}else{
			rope2.set(0);
		}
		if(stick.getRawButton(1)){
			rope1.set(1);
			rope2.set(-1);
			armOverride = true;
		}else{
			rope1.set(0);
			rope2.set(0);
			armOverride = false;
		}
		//Drive shift solenoid
		if(mecanumDrive){
			/** For mecanum drive, it requires that the left side be inverted in order to function properly.
			* However, arcade drive gets switched when the left side is inverted. As a result, every time
			* we have to change between between mecanum and arcade drive, we must invert the left side.
			*/
			robot.setInvertedMotor(MotorType.kFrontLeft,true);
			robot.setInvertedMotor(MotorType.kRearLeft,true);
			mecanumShift.set(true);
		}else{
			robot.setInvertedMotor(MotorType.kFrontLeft,false);
			robot.setInvertedMotor(MotorType.kRearLeft,false);
			mecanumShift.set(false);
		}
		
		//Drive train
		//Acceleration curve to make driving smoother
		//Inputs are joystick axis value, previous drive, acceleration rate
		yDrive = accelDecel(stick.getRawAxis(1),yDrive,0.05);
		xDrive = accelDecel(stick.getRawAxis(0),xDrive,0.05);
		turnDrive = accelDecel(stick.getRawAxis(2),turnDrive,0.05);
		if(mecanumDrive){
			//Order: x,twist,y,gyro angle
			if(stick.getRawAxis(1)>0.1||stick.getRawAxis(1)<-0.1||stick.getRawAxis(2)>0.1||stick.getRawAxis(2)<-0.1||stick.getRawAxis(0)>0.1||stick.getRawAxis(0)<-0.1){
				robot.mecanumDrive_Cartesian(stick.getRawAxis(0),stick.getRawAxis(1),stick.getRawAxis(2),0);
			}else{
				robot.mecanumDrive_Cartesian(0,0,0,0);
			}
		}else{
			if(creepMode){
				//(Move axis, rotate axis)
				robot.arcadeDrive(stick.getRawAxis(1)*0.5,stick.getRawAxis(2)*0.5);
			}else{
				robot.arcadeDrive(stick.getRawAxis(1),stick.getRawAxis(2));
			}
		
		System.out.println("Left enc: "+frontLeft.getEncPosition()+" Right enc: "+frontRight.getEncPosition()+" Gyro: "+gyro.getAngle()+" Creep mode: "+creepMode);
		}
	}

	@Override
	public void testPeriodic() {
		//LiveWindow.run();
		SmartDashboard.putNumber("gyroAngle", gyro.getAngle());
	}
	public static double accelDecel(double joyValue, double previous, double accRate){
		/**
		 * This takes the joystick value and compares it to the previous drive from the last loop,
		 * and then multiplies the previous drive by the acceleration rate. This is done multiple
		 * times a second since it is being called in Periodic Tasks, so it will keep multiplying
		 * until it reaches the joystick value, effectively creating an acceleration curve.
		 * The 6% range is so that you don't have a case where, due to the acceleration rate, the
		 * robot continuously goes above and below the actual drive, causing a small amount of 
		 * stutter that can damage the motors.
		*/
		double drive;
		if((joyValue-0.03)>previous){
			//Acceleration
			drive = previous*(1+accRate);
		}else if((joyValue+0.03)<previous){
			//Deceleration
			drive = previous*(1-accRate);
		}else{
			drive = previous;
		}
		return drive;
	}
	
	public void driveForward(double distance){
		//Reset encoders
		frontLeft.setEncPosition(0);
		frontRight.setEncPosition(0);
		//Wait 100 ms to let the reset fully register
		try{
			Thread.sleep(100);
		}catch(InterruptedException ex){
			Thread.currentThread().interrupt();
		}
		//Reset gyro
		gyro.reset();
		
		//Convert distance to encoder ticks
		double parameters;
		parameters = ((distance/(4*Math.PI))*ticksPerRotation);
		
		//Drive forward until encoder position is equal to the end encoder position
		while(frontLeft.getEncPosition()<parameters&&frontRight.getEncPosition()<parameters&&!end){
			robot.drive(-0.25,gyro.getAngle()*gcs);
			end = !isAutonomous();
		}
		robot.drive(0,0);
	}
	
	public void driveBackward(double distance){
		//Reset encoders
		frontLeft.setEncPosition(0);
		frontRight.setEncPosition(0);
		//Wait 100 ms to let the reset fully register
		try{
			Thread.sleep(100);
		}catch(InterruptedException ex){
			Thread.currentThread().interrupt();
		}
		//Reset gyro
		gyro.reset();
				
		//Convert distance to encoder ticks
		double parameters;
		parameters = (-1*(distance/(4*Math.PI))*ticksPerRotation);
		
		//Drive backward until encoder position is equal to the end encoder position
		while(frontLeft.getEncPosition()>parameters&&frontRight.getEncPosition()>parameters&&!end){
			robot.drive(0.25,gyro.getAngle()*gcs);
			end = !isAutonomous();
		}
		robot.drive(0,0);
	}
	
	public void turnRight(double degrees){
		//Reset gyro
		gyro.reset();
		
		//Turn right until gyro angle is equal to the end gyro angle
		while(gyro.getAngle()*gs<degrees&&!end){
			robot.tankDrive(-0.75,0.75);
			end = !isAutonomous();
		}
		robot.drive(0,0);
	}
	
	public void turnLeft(double degrees){
		//Reset gyro
		gyro.reset();
		
		//Turn left until gyro angle is equal to the end gyro angle
		while(gyro.getAngle()*gs>-degrees&&!end){
			robot.tankDrive(0.75,-0.75);
			end = !isAutonomous();
		}
		robot.drive(0,0);
	}
	
	public void clampOpen(int waitTime){
		end = !isAutonomous();
		if(!end){
			armClamp.set(true);
			try{
				Thread.sleep(1000*waitTime);
			}catch(InterruptedException ex){
				Thread.currentThread().interrupt();
			}
		}
	}
	
	public void clampClose(){
		end = !isAutonomous();
		if(!end){
			armClamp.set(false);
		}
	}
	
	public void ballRelease(int waitTime){
		end = !isAutonomous();
		if(!end){
			ballRelease.set(true);
			try{
				Thread.sleep(1000*waitTime);
			}catch(InterruptedException ex){
				Thread.currentThread().interrupt();
			}
			ballRelease.set(false);
		}
	}
}
