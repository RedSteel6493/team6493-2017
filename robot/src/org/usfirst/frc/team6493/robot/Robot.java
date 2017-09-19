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
import edu.wpi.first.wpilibj.smartdashboard.*;

public class Robot extends IterativeRobot {
	// Creating & initializing variables and objects
	
	// Drive train & motors
	CANTalon frontLeft, rearLeft, frontRight, rearRight; // Drive motors
	RobotDrive robot;
	Spark rope1 = new Spark(0); // Rope motor 1
	Spark rope2 = new Spark(1); // Rope motor 2
	double xDrive, yDrive, turnDrive, parameters; // Not currently being used, used for accel/decel
	
	// Sensors
	ADXRS450_Gyro gyro = new ADXRS450_Gyro();
	
	// Controller inputs
	Joystick stick = new Joystick(0);
	Joystick gamepad = new Joystick(1);
	boolean singleContr = false; // Determines if the buttons are mapped to the joystick or additional xbox controller
	
	/* 
	 * Toggles
	 * Toggles are used to prevent jumping back and forth when the user tries to toggle
	 * They are deactivated when the toggle button is pressed, and are reactivated when it is released
	 * While they are deactivated, that function cannot be used
	 */
	boolean mecanumDriveToggle = true;
	boolean creepModeToggle = true;
	boolean armDownToggle = true;
	boolean singleContrToggle = true;
	boolean armOpenToggle = true;
	
	// Pneumatics
	Solenoid ballRelease = new Solenoid(0, 0);
	Solenoid armLift = new Solenoid(0, 5);
	Solenoid armClamp = new Solenoid(0, 6);
	Solenoid mecanumShift = new Solenoid(0, 7);
	boolean mecanumDrive = true; // Determines if the robot is in mecanum or traction drive
	boolean creepMode = false; // Slows speed if activated
	boolean armDown = false; // Determines if arm is down or up
	boolean armOpen = false; // Determines if arm is open or closed
	boolean release = false;
	boolean armOverride = false; // Arm is forced down if rope motors are in motion
	
	// Autonomous
	boolean end = false; // Used in multiple places to determine when to end autonomous looping
	boolean leftPath, centerPath, rightPath, gyroTest, encTest, redBoiler, blueBoiler, baseline; // Dashboard input buttons for autonomous
	final double GYRO_SCALE = 1.075; // The gyro's output is not accurate to real world degrees, so the gyro scale is multiplied to the gyro angle to get it close to real world values
	final double GYRO_CORRECTION_SCALE = -0.03; // Used when driving forward and backward in autonomous, lessens the effect of the gyro to allow the robot to steer itself forward
	final double BOT_LENGTH = 36; // Used to calculate distance for autonomous
	final int TICKS_PER_ROTATION = 256; // Used to calculate distance for autonomous

	@Override
	public void robotInit() {
		gyro.calibrate();
		CameraServer.getInstance().startAutomaticCapture(); // Start USB Camera
		CameraServer.getInstance().addAxisCamera("axis-camera.local"); // Start Ethernet Camera
		
		// Initializing talons based on device IDs
		frontLeft = new CANTalon(50);
		rearLeft = new CANTalon(52);
		frontRight = new CANTalon(53);
		rearRight = new CANTalon(51);
		// Initializing robot drive system using talons
		robot = new RobotDrive(frontLeft,rearLeft,frontRight,rearRight);
		robot.setSafetyEnabled(false); // Done to prevent unnecessary errors that were not a real problem
		
		// Initializing encoders: The encoders are plugged into the front left and front right talons, thus are initialized under those talons
		frontLeft.setFeedbackDevice(FeedbackDevice.EncRising);
		frontRight.setFeedbackDevice(FeedbackDevice.EncRising);
		
	}
	

	@Override
	public void autonomousInit() {
		// Traction drive, motors are not inverted
		robot.setInvertedMotor(MotorType.kFrontLeft,false);
		robot.setInvertedMotor(MotorType.kRearLeft,false);
		
		// Set default positions for solenoids
		mecanumShift.set(false);
		armLift.set(false);
		armClamp.set(false);
		
		gyro.reset(); // Gyro reset to 0
		
		// Buttons are read from dashboard to determine which path to follow
		leftPath = SmartDashboard.getBoolean("DB/Button 0",false);
		centerPath = SmartDashboard.getBoolean("DB/Button 1",false);
		rightPath = SmartDashboard.getBoolean("DB/Button 2",false);
		redBoiler = SmartDashboard.getBoolean("DB/Button 3",false);
		blueBoiler = SmartDashboard.getBoolean("DB/Button 4",false);
		gyroTest = SmartDashboard.getBoolean("DB/Button 5",false);
		encTest = SmartDashboard.getBoolean("DB/Button 6",false);
		baseline = SmartDashboard.getBoolean("DB/Button 7",false);
		
		end = false; // Autonomous will stop looping when end is true
	}
	
	@Override
	public void autonomousPeriodic() {
		if(!end){
			if(leftPath&&!centerPath&&!rightPath&&!redBoiler&&!blueBoiler&&!gyroTest&&!encTest&&!baseline){ // Left gear path
				// Places gear on left peg from the left side of the field
				// Drive forward, turn right, drive forward, place gear, drive backward
				System.out.println("Left path selected.");
				
				driveForward(105);
				turnRight(60);
				driveForward(32);
				clampOpen(2);
				driveBackward(24);
				clampClose();
				
			}else if(!leftPath&&centerPath&&!rightPath&&!redBoiler&&!blueBoiler&&!gyroTest&&!encTest&&!baseline){ // Center gear path
				// Places gear on center peg from the center of the field
				// Drive forward, place gear, drive backward
				System.out.println("Center path selected.");
				
				driveForward(114-BOT_LENGTH);
				clampOpen(2);
				driveBackward(24);
				clampClose();
				
			}else if(!leftPath&&!centerPath&&rightPath&&!redBoiler&&!blueBoiler&&!gyroTest&&!encTest&&!baseline){ // Right gear path
				// Places gear on right peg from the right side of the field
				// Drive forward, turn left, drive forward, place gear, drive backward
				System.out.println("Right path selected.");
				
				driveForward(105);
				turnLeft(60);
				driveForward(32);
				clampOpen(2);
				driveBackward(24);
				clampClose();
				
			}else if(!leftPath&&!centerPath&&!rightPath&&redBoiler&&!blueBoiler&&!gyroTest&&!encTest&&!baseline){ // Red boiler path
				// Dumps balls into the low goal on red side (right side of the field)
				// Drive forward, turn left, dump balls, drive forward, turn left, drive forward
				System.out.println("Red boiler path selected.");
				
				driveForward(50);
				turnLeft(135);
				ballRelease(5);
				driveForward(49);
				turnLeft(45);
				driveForward(60);
				
			}else if(!leftPath&&!centerPath&&!rightPath&&!redBoiler&&blueBoiler&&!gyroTest&&!encTest&&!baseline){ // Blue boiler path
				// Dumps balls into the low goal on red side (left side of the field)
				// Drive forward, turn right, dump balls, drive forward, turn right, drive forward
				System.out.println("Blue boiler path selected.");
				
				driveForward(50);
				turnRight(135);
				driveBackward(49);
				ballRelease(5);
				driveForward(49);
				turnLeft(45);
				driveForward(60);
				
			}else if(!leftPath&&!centerPath&&!rightPath&&!redBoiler&&!blueBoiler&&gyroTest&&!encTest&&!baseline){ // Gyro test (not used in competition)
				// Used to easily fine tune gyro scale during autonomous development
				System.out.println("Gyro test selected. The robot will now rotate 360 degrees clockwise.");
				
				turnRight(360);
				
			}else if(!leftPath&&!centerPath&&!rightPath&&!redBoiler&&!blueBoiler&&!gyroTest&&encTest&&!baseline){ // Encoder test (not used in competition)
				// Used to test encoders' accuracy to real world inches during autonomous development
				System.out.println("Encoder test selected. The robot will now move 3 feet forward.");
				
				driveForward(126-BOT_LENGTH);
				
			}else if(!leftPath&&!centerPath&&!rightPath&&!redBoiler&&!blueBoiler&&!gyroTest&&!encTest&&baseline){ // Baseline path
				// Crosses the baseline without placing gears or dumping balls
				System.out.println("Baseline path selected.");
				
				driveForward(148);
				
			}else{ // Triggered if either no paths or multiple paths are selected
				System.out.println("Multiple paths (or no paths) selected. The robot will not move.");
				
				robot.drive(0,0);
				
			}
			end = true; // Prevents autonomous from looping when the necessary actions are completed
		}
	}
	@Override
	public void teleopInit() {
		end = true; // Prevents autonomous from looping after teleop has been initialized
		
		// Resets encoder positions for reading values (used for troubleshooting)
		frontLeft.setEncPosition(0);
		frontRight.setEncPosition(0);
		
		// Sets default positions for solenoids
		armDown = false;
		armOpen = false;
		mecanumDrive = false; // Defaults to traction drive
	}
	@Override
	public void teleopPeriodic() {
		/*
		 *  Toggles: Only work if button is pressed & toggle boolean is true
		 *  When the button is pressed and the toggle boolean is true, the condition toggles and the toggle boolean becomes false
		 *  When the button is released, the toggle boolean becomes true again and the condition is able to be toggled again
		 *  This is done to prevent toggling back and forth when the button is held down
		 */
		
		// Single controller toggle
		if((gamepad.getRawButton(4) || stick.getRawButton(5)) && singleContrToggle) {
			singleContr = !singleContr;
			singleContrToggle = false;
		} else if (!gamepad.getRawButton(4) && !stick.getRawButton(5))
			singleContrToggle = true;
		
		// Drive shift toggle
		if(stick.getRawButton(4) && mecanumDriveToggle) {
			mecanumDrive = !mecanumDrive;
			mecanumDriveToggle = false;
		} else if(!stick.getRawButton(4))
			mecanumDriveToggle = true;
		
		// Creep mode toggle
		if(stick.getRawButton(3) && creepModeToggle){
			creepMode = !creepMode;
			creepModeToggle = false;
		} else if (!stick.getRawButton(3))
			creepModeToggle = true;
		
		// Arm lift toggle (or individual buttons if singleContr is enabled)
		if(singleContr) {
			if(stick.getRawButton(7))
				armDown = true;
			if(stick.getRawButton(8))
				armDown = false;
		} else {
			if(gamepad.getRawButton(1) && armDownToggle){
				armDown = !armDown;
				armDownToggle = false;
			} else if(!gamepad.getRawButton(1))
				armDownToggle = true;
		}
		
		// Arm clamp toggle (or held button if singleContr is disabled)
		if(singleContr) {
			if(stick.getRawButton(2) && armOpenToggle) {
				armOpen = !armOpen;
				armOpenToggle = false;
			} else if(!stick.getRawButton(2))
				armOpenToggle = true;
		}else {
			if(gamepad.getRawAxis(2)>0.5){
				armOpen = true;
			}else{
				armOpen = false;
			}
		}
		
		// Rope motors
		// Arm is forced down if rope motors are in motion
		if(stick.getRawButton(1)){
			rope1.set(1);
			rope2.set(-1);
			armOverride = true;
		}else{
			rope1.set(0);
			rope2.set(0);
			armOverride = false;
		}
		
		
		// Updating solenoids based on booleans tied to them
		armLift.set(armDown || armOverride); // Forces arm down if rope motors are in motion
		armClamp.set(armOpen);
		ballRelease.set(release);
		
		//Drive shift solenoid
		if(mecanumDrive && !armDown){ // Forces robot into traction drive if the arm is down
			/*
			* For mecanum drive, it requires that the left side be inverted in order to function properly.
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
			//Acceleration curve to make driving smoother (acceleration is not currently being used)
			//Inputs are joystick axis value, previous drive, acceleration rate
			// yDrive = accelDecel(stick.getRawAxis(1),yDrive,0.05);
			// xDrive = accelDecel(stick.getRawAxis(0),xDrive,0.05);
			// turnDrive = accelDecel(stick.getRawAxis(2),turnDrive,0.05);
		if(mecanumDrive){
			//(x axis, twist axis, y axis, gyro angle)
			if(stick.getRawAxis(1) > 0.1 || stick.getRawAxis(1) < -0.1 || stick.getRawAxis(2) > 0.1 || stick.getRawAxis(2) < -0.1 || stick.getRawAxis(0) > 0.1 || stick.getRawAxis(0) < -0.1) {
				robot.mecanumDrive_Cartesian(stick.getRawAxis(0), stick.getRawAxis(1), stick.getRawAxis(2), 0);
			}else{
				robot.mecanumDrive_Cartesian(0, 0, 0, 0);
			}
		}else{
			if(creepMode){
				//(Move axis, rotate axis)
				robot.arcadeDrive(stick.getRawAxis(1) * 0.5, stick.getRawAxis(2) * 0.5);
			}else{
				robot.arcadeDrive(stick.getRawAxis(1), stick.getRawAxis(2));
			}
		
		System.out.println("Left enc: "+frontLeft.getEncPosition()+" Right enc: "+frontRight.getEncPosition()+" Gyro: "+gyro.getAngle()+" Creep mode: "+creepMode); // Used for troubleshooting
		}
	}

	@Override
	public void testPeriodic() {
	}
	
	public static double accelDecel(double joyValue, double previous, double accRate){
		/*
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
		parameters = ((distance/(4*Math.PI))*TICKS_PER_ROTATION);
		
		//Drive forward until encoder position is equal to the end encoder position
		while(frontLeft.getEncPosition()<parameters&&frontRight.getEncPosition()<parameters&&!end){
			robot.drive(-0.25,gyro.getAngle()*GYRO_CORRECTION_SCALE);
			
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
		parameters = (-1*(distance/(4*Math.PI))*TICKS_PER_ROTATION);
		
		//Drive backward until encoder position is equal to the end encoder position
		while(frontLeft.getEncPosition()>parameters&&frontRight.getEncPosition()>parameters&&!end){
			robot.drive(0.25,gyro.getAngle()*GYRO_CORRECTION_SCALE);
			
			end = !isAutonomous();
		}
		robot.drive(0,0);
	}
	
	public void turnRight(double degrees){
		//Reset gyro
		gyro.reset();
		
		//Turn right until gyro angle is equal to the end gyro angle
		while(gyro.getAngle()*GYRO_SCALE<degrees&&!end){
			robot.tankDrive(-0.75,0.75);
			
			end = !isAutonomous();
		}
		robot.drive(0,0);
	}
	
	public void turnLeft(double degrees){
		//Reset gyro
		gyro.reset();
		
		//Turn left until gyro angle is equal to the end gyro angle
		while(gyro.getAngle()*GYRO_SCALE>-degrees&&!end){
			robot.tankDrive(0.75,-0.75);
			
			end = !isAutonomous();
		}
		robot.drive(0,0);
	}
	
	public void clampOpen(int waitTime){
		end = !isAutonomous();
		
		if(!end){
			armClamp.set(true);
			try {
				Thread.sleep(1000*waitTime);
			} catch(InterruptedException ex) {
				Thread.currentThread().interrupt();
			}
		}
	}
	
	public void clampClose(){
		end = !isAutonomous();
		
		if(!end)
			armClamp.set(false);
	}
	
	public void ballRelease(int waitTime){
		end = !isAutonomous();
		
		if(!end){
			ballRelease.set(true);
			try {
				Thread.sleep(1000*waitTime);
			} catch(InterruptedException ex) {
				Thread.currentThread().interrupt();
			}
			ballRelease.set(false);
		}
	}
}