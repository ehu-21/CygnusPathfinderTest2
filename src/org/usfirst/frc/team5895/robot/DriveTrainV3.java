/*
 * This DriveTrain version doesn't have splines in it. It's just Pathfinder
 */

package org.usfirst.frc.team5895.robot;

import org.usfirst.frc.team5895.robot.lib.pathfinder.PathfinderGenerator;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import org.usfirst.frc.team5895.robot.lib.NavX;
import org.usfirst.frc.team5895.robot.lib.PID;
import org.usfirst.frc.team5895.robot.lib.PathfinderFollower;
import org.usfirst.frc.team5895.robot.lib.BetterEncoder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;

public class DriveTrainV3 {

	private TalonSRX leftDriveMaster, rightDriveMaster;
	private VictorSPX leftDriveFollower1, leftDriveFollower2, rightDriveFollower1, rightDriveFollower2;
	private BetterEncoder leftEncoder,rightEncoder;
	private double leftspeed, rightspeed;
	private NavX navX;
	
	private enum Mode_Type {TELEOP, AUTO_DRIVE, AUTO_PATHFINDER, AUTO_MIRROR_PATHFINDER};
	private Mode_Type mode = Mode_Type.TELEOP;
	
	private PID turnPID;
	private boolean turning;
	private PID drivePID;
	
	public static final double TURN_P = 0.030; 
	public static final double TURN_I = 0.00000001;
	private static final double capSpeed = 0.5;
	
	private PathfinderGenerator pathMaster;
	private PathfinderFollower pStraight, pInUse, pTest;
	private PathfinderFollower pCenterRightSwitchFront, pCenterLeftSwitchFront, pCenterRightScale;
	private PathfinderFollower pCenterLeftScale, pRightRightSwitchFront, pLeftLeftSwitchFront;
	private PathfinderFollower pRightLeftSwitchBack, pLeftRightSwitchBack, pRightLeftSwitchFront;
	private PathfinderFollower pLeftRightSwitchFront, pRightRightSwitchSide, pLeftLeftSwitchSide;
	private PathfinderFollower pRightSwitchBlock, pLeftSwitchBlock, pRightRightScale, pLeftLeftScale;
	private PathfinderFollower pRightScaleLeftSwitch, pLeftScaleRightSwitch, pRightLeftScale, pRightLeftDrive;
	private PathfinderFollower pLeftRightDrive, pRightScaleCube, pRightScaleCubeBack, pSCurve, pCenterRightSwitchCube;
	private PathfinderFollower pCenterRightSwitchRev, pBackwards, pRightScaleBackwards;
	private PathfinderFollower pRightScaleForwards, pRightSecondScaleCube, pLeftSecondScaleCube;
	
	// tracking
	private double posX, posY; // feet
	private double lastDistance = 0d; // distance traveled the last time update() was called
	
	public DriveTrainV3() {
	
		//initialize drive motors
		leftDriveMaster = new TalonSRX(ElectricalLayout.MOTOR_DRIVE_LEFT_MASTER);
		leftDriveFollower1 = new VictorSPX(ElectricalLayout.MOTOR_DRIVE_LEFT_FOLLOWER_1);
		leftDriveFollower2 = new VictorSPX(ElectricalLayout.MOTOR_DRIVE_LEFT_FOLLOWER_2);
		rightDriveMaster = new TalonSRX(ElectricalLayout.MOTOR_DRIVE_RIGHT_MASTER);
		rightDriveFollower1 = new VictorSPX(ElectricalLayout.MOTOR_DRIVE_RIGHT_FOLLOWER_1);
		rightDriveFollower2 = new VictorSPX(ElectricalLayout.MOTOR_DRIVE_RIGHT_FOLLOWER_2);
			
		//initialize encoders
		leftEncoder = new BetterEncoder(ElectricalLayout.ENCODER_DRIVE_LEFT_1, ElectricalLayout.ENCODER_DRIVE_LEFT_2, true, Encoder.EncodingType.k4X);
		rightEncoder = new BetterEncoder(ElectricalLayout.ENCODER_DRIVE_RIGHT_1, ElectricalLayout.ENCODER_DRIVE_RIGHT_2, false, Encoder.EncodingType.k4X);
		
		//set encoder distance per pulse so it's in feet
		leftEncoder.setDistancePerPulse(6/12.0*Math.PI/360); //correct
		rightEncoder.setDistancePerPulse(6/12.0*Math.PI/360); //correct
		
		leftEncoder.setSamplesToAverage(10);
		rightEncoder.setSamplesToAverage(10);
		
		//initialize navx
		navX = new NavX();
		
		//initializes Pathfinder
		pathMaster = new PathfinderGenerator(false);
		try {
			DriverStation.reportError("Start generating paths with pathfinder", true);
			//kp ki kd kv ka kturn
			
			pStraight = new PathfinderFollower(pathMaster.Straight(), 0.0, 0, 0, 1.0/12.75, 1.0/55.0, -0.0); //kp tuned from 0.15 to 0.045
	/*		pSCurve = new PathfinderFollower(pathMaster.RightLeftScale(), 0.045, 0, 0, 1.0/13.75, 1.0/75.0, -0.009);
			pBackwards = new PathfinderFollower(pathMaster.RightLeftScale(), 0.045, 0, 0, 1.0/13.75, 1.0/75.0, -0.009);
				
			pRightRightScale  = new PathfinderFollower(pathMaster.RightRightScale(), 0.045, 0, 0, 1.0/13.75, 1.0/75.0, -0.009);
			pRightScaleLeftSwitch = new PathfinderFollower(pathMaster.RightScaleLeftSwitch(), 0.045, 0, 0, 1.0/13.75, 1.0/75.0, -0.009);
			pRightRightSwitchFront = new PathfinderFollower(pathMaster.RightRightSwitchFront(), 0.045, 0, 0, 1.0/13.75, 1.0/75.0, -0.009);
			pRightScaleBackwards = new PathfinderFollower(pathMaster.RightScaleBackwards(), 0.045, 0, 0, 1.0/13.75, 1.0/75.0, -0.009);
			//pRightScaleForwards = new PathfinderFollower(pathMaster.RightScaleForwards(), 0.045, 0, 0, 1.0/13.75, 1.0/75.0, -0.009);
			pRightSallower(pathMaster.RightLeftScale(), 0.045, 0, 0, 1.0/13.75, 1.0/75.0, -0.009);
			pRightLeftDrive = new PathfinderFollower(pathMaster.RightLeftDrive(), 0.045, 0, 0, 1.0/13.75, 1.0/75.0, -0.009);
			
			pCenterRightSwitchFront = new PathfinderFollower(pathMaster.CenterRightSwitchFront(), 0.045, 0, 0, 1.0/13.75, 1.0/75.0, -0.009);
			pCenterLeftSwitchFront = new PathfinderFollower(pathMaster.CenterLeftSwitchFront(), 0.045, 0, 0, 1.0/13.75, 1.0/75.0, -0.009);
			pCenterRightScale = new PathfinderFollower(pathMaster.CenterRightScale(), 0.045, 0, 0, 1.0/13.75, 1.0/75.0, -0.009);
			pCenterRightSwitchCube = new PathfinderFollower(pathMaster.CenterRightSwitchCube(), 0.045, 0, 0, 1.0/13.75, 1.0/75.0, -0.009);
			pCenterRightSwitchRev = new PathfinderFollower(pathMaster.CenterRightSwitchRev(), 0.045, 0, 0, 1.0/13.75, 1.0/75.0, -0.009);
			
			pLeftLeftSwitchFront = new PathfinderFollower(pathMaster.LeftLeftSwitchFront(), 0.045, 0, 0, 1.0/13.75, 1.0/75.0, -0.009);
			pLeftSecondScaleCube = new PathfinderFollower(pathMaster.LeftSecondScaleCube(), 0.045, 0, 0, 1.0/13.75, 1.0/75.0, -0.009);
			pLeftRightSwitchBack = new PathfinderFollower(pathMaster.LeftRightSwitchBack(), 0.045, 0, 0, 1.0/13.75, 1.0/75.0, -0.009);
			pLeftRightSwitchFront = new PathfinderFollower(pathMaster.LeftRightSwitchFront(), 0.045, 0, 0, 1.0/13.75, 1.0/75.0, -0.009);
			pLeftLeftSwitchSide = new PathfinderFollower(pathMaster.LeftLeftSwitchSide(), 0.045, 0, 0, 1.0/13.75, 1.0/75.0, -0.009);
			pLeftSwitchBlock = new PathfinderFollower(pathMaster.LeftSwitchBlock(), 0.045, 0, 0, 1.0/13.75, 1.0/75.0, -0.009);
			pLeftLeftScale = new PathfinderFollower(pathMaster.LeftLeftScale(), 0.045, 0, 0, 1.0/13.75, 1.0/75.0, -0.009);
			pLeftScaleRightSwitch = new PathfinderFollower(pathMaster.LeftScaleRightSwitch(), 0.045, 0, 0, 1.0/13.75, 1.0/75.0, -0.009);
			pLeftRightDrive = new PathfinderFollower(pathMaster.LeftRightDrive(), 0.045, 0, 0, 1.0/13.75, 1.0/75.0, -0.009);
		*/
		}catch(Exception e) {
			DriverStation.reportError("Pathfinder: error generate paths", false);
		}
	}
	
	/**
	 * resets the encoder values to 0
	 */
	public void resetEncoders() {
		leftEncoder.reset();
		rightEncoder.reset();
	}
	
	/**
	 * resets the NavX to 0
	 */
	public void resetNavX() {
		navX.reset();
	}
	
	
	/**
	 * turns the robot to a certain angle using the PID.
	 * @param angle the angle that the robot should point to.
	 */
	public void turnTo(double angle) {
		turnPID.set(angle);
		turning = true;
		mode = Mode_Type.TELEOP;
	}
	
	/**
	 * makes the robot stop turning using the PID
	 */
	public void stopTurning() {
		turning = false;
	}
	
	/**
	 * returns if the robot is turning to an angle using the PID
	 * @return if the robot is turning to an angle using the PID
	 */
	public boolean isTurning() {
		return turning;	
	}
	
	/**
	 * gets the current angle of the NavX
	 * @return the NavX angle in degrees
	 */
	public double getAngle() {
		return navX.getAngle(); // navX reads angle in degree
	}
	
	/**
	 * General/Testing Pathfinder Paths
	 */
	
	public void test() {
		resetEncoders();
		pInUse = pTest;
		pInUse.reset();
		mode = Mode_Type.AUTO_PATHFINDER;
	}
	
	public void straight() {
		resetEncoders();
		pInUse = pStraight;
		pInUse.reset();
		mode = Mode_Type.AUTO_PATHFINDER;
	}
	
	public void SCurve() {
		resetEncoders();
		pInUse = pSCurve;
		pInUse.reset();
		mode = Mode_Type.AUTO_PATHFINDER;
	}
	
	/*
	 * I'm not rlly sure what this does but it was a path in the spline generator
	 * so I put it in here as well
	 */
	public void backwards() {
		resetEncoders();
		pInUse = pBackwards;
		pInUse.reset();
		mode = Mode_Type.AUTO_PATHFINDER;
	}
	
	/**
	 * Center Pathfinder paths 
	 */	
	
	public void center_right_switch_front() {
		resetEncoders();
		pInUse = pCenterRightSwitchFront;
		pInUse.reset();
		mode = Mode_Type.AUTO_PATHFINDER;
	}
	
		
	public void center_right_scale() {
			resetEncoders();
			pInUse = pCenterRightScale;
			pInUse.reset();
			mode = Mode_Type.AUTO_PATHFINDER;
	}
	
	
	public void center_left_scale() {
		resetEncoders();
		pInUse = pCenterLeftScale;
		pInUse.reset();
		mode = Mode_Type.AUTO_PATHFINDER;
	}
	
	public void center_right_switch_cube() {
		resetEncoders();
		pInUse = pCenterRightSwitchCube;
		pInUse.reset();
		mode = Mode_Type.AUTO_PATHFINDER;
	}
	
	public void center_right_switch_rev() {
		resetEncoders();
		pInUse = pCenterRightSwitchRev;
		pInUse.reset();
		mode = Mode_Type.AUTO_PATHFINDER;
	}
	
	public void center_left_switch_front() {
		resetEncoders();
		pInUse = pCenterLeftSwitchFront;
		pInUse.reset();
		mode = Mode_Type.AUTO_PATHFINDER;
	}
	
	/**
	 * Left Pathfinder paths
	 */
	
	public void left_right_switch_back() {
		resetEncoders();
		pInUse = pLeftRightSwitchBack;
		pInUse.reset();
		mode = Mode_Type.AUTO_PATHFINDER;
	}
	
	public void left_right_switch_front() {
		resetEncoders();
		pInUse = pLeftRightSwitchFront;
		pInUse.reset();
		mode = Mode_Type.AUTO_PATHFINDER;
	}
	
	public void left_left_switch_side() {
		resetEncoders();
		pInUse = pLeftLeftSwitchSide;
		pInUse.reset();
		mode = Mode_Type.AUTO_PATHFINDER;
	}
	
	public void left_left_scale() {
		resetEncoders();
		pInUse = pLeftLeftScale;
		pInUse.reset();
		mode = Mode_Type.AUTO_PATHFINDER;
	}
	
	public void left_left_switch_front() {
		resetEncoders();
		pInUse = pLeftLeftSwitchFront;
		pInUse.reset();
		mode = Mode_Type.AUTO_PATHFINDER;
	}
	
	public void left_switch_block() {
		resetEncoders();
		pInUse = pLeftSwitchBlock;
		pInUse.reset();
		mode = Mode_Type.AUTO_PATHFINDER;
	}
	
	public void left_scale_right_switch() {
		resetEncoders();
		pInUse = pLeftScaleRightSwitch;
		pInUse.reset();
		mode = Mode_Type.AUTO_PATHFINDER;
	}
	
	public void left_right_drive() {
		resetEncoders();
		pInUse = pLeftRightDrive;
		pInUse.reset();
		mode = Mode_Type.AUTO_PATHFINDER;
	}
	
	public void left_second_scale_cube() {
		resetEncoders();
		pInUse = pLeftSecondScaleCube;
		pInUse.reset();
		mode = Mode_Type.AUTO_PATHFINDER;
	}
	
	/**
	 * Right Pathfinder Paths
	 */
	
	public void right_right_switch_front() {
		resetEncoders();
		pInUse = pRightRightSwitchFront;
		pInUse.reset();
		mode = Mode_Type.AUTO_PATHFINDER;
	} 
	
	public void right_left_switch_back() {
		resetEncoders();
		pInUse = pRightLeftSwitchBack;
		pInUse.reset();
		mode = Mode_Type.AUTO_PATHFINDER;
	}
	
	public void right_left_switch_front() {
		resetEncoders();
		pInUse = pRightLeftSwitchFront;
		pInUse.reset();
		mode = Mode_Type.AUTO_PATHFINDER;
	}
	
	public void right_right_switch_side() {
		resetEncoders();
		pInUse = pRightRightSwitchSide;
		pInUse.reset();
		mode = Mode_Type.AUTO_PATHFINDER;
	}
	
	public void right_switch_block() {
		resetEncoders();
		pInUse = pRightSwitchBlock;
		pInUse.reset();
		mode = Mode_Type.AUTO_PATHFINDER;
	}
	
	public void right_right_scale() {
		resetEncoders();
		pInUse = pRightRightScale;
		pInUse.reset();
		mode = Mode_Type.AUTO_PATHFINDER;
	}
	
	public void right_scale_left_switch() {
		resetEncoders();
		pInUse = pRightScaleLeftSwitch;
		pInUse.reset();
		mode = Mode_Type.AUTO_PATHFINDER;
	}
	
	public void right_left_scale() {
		resetEncoders();
		pInUse = pRightLeftScale;
		pInUse.reset();
		mode = Mode_Type.AUTO_PATHFINDER;
	}
	
	public void right_left_drive() {
		resetEncoders();
		pInUse = pRightLeftDrive;
		pInUse.reset();
		mode = Mode_Type.AUTO_PATHFINDER;
	}
	
	public void right_scale_cube() {
		resetEncoders();
		pInUse = pRightScaleCube;
		pInUse.reset();
		mode = Mode_Type.AUTO_PATHFINDER;
	}
	
	public void right_scale_cube_back() {
		resetEncoders();
		pInUse = pRightScaleCubeBack;
		pInUse.reset();
		mode = Mode_Type.AUTO_PATHFINDER;
	}
	
	public void right_scale_backwards() {
		resetEncoders();
		pInUse = pRightScaleBackwards;
		pInUse.reset();
		mode = Mode_Type.AUTO_PATHFINDER;
	}
	
	public void right_scale_forwards() {
		resetEncoders();
		pInUse = pRightScaleForwards;
		pInUse.reset();
		mode = Mode_Type.AUTO_PATHFINDER;
	}
	
	public void right_second_scale_cube() {
		resetEncoders();
		pInUse = pRightSecondScaleCube;
		pInUse.reset();
		mode = Mode_Type.AUTO_PATHFINDER;
	}
	
	/**
	 * Mirrored (untested) pathfinder paths
	 */
	
	public void auto_left_leftScale() {
		resetEncoders();
		pInUse = pRightRightScale;
		pInUse.reset();
		mode = Mode_Type.AUTO_MIRROR_PATHFINDER;
	}
	
	public void auto_left_rightScale() {
		resetEncoders();
		pInUse = pRightLeftScale;
		pInUse.reset();
		mode = Mode_Type.AUTO_MIRROR_PATHFINDER;
	}
	
	/**
	 * normal arcade drive code for teleop use
	 * @param speed the forward speed to go at
	 * @param turn the speed to turn at
	 */
	public void arcadeDrive(double speed, double turn) {
		leftspeed = speed - turn;
		rightspeed = speed + turn;
		mode = Mode_Type.TELEOP;
		turning = false;
	}
	
	/**
	 * @return the average velocity in feet per second from the left and right encoders.
	 */
	public double getVelocity() {
		return (leftEncoder.getRate()+rightEncoder.getRate())/2;
	}
	
	/**
	 * @return the average distance traveled in feet from the left and right encoders.
	 */
	public double getDistanceTraveled() {
		return (leftEncoder.getDistance()+rightEncoder.getDistance())/2;
	}
	
	/**
	 * gets the x position of the drivetrain
	 * @return the x position of the drivetrain in feet
	 */
	public double getXPosition() {
		return posX;
	}
	
	/**
	 * gets the y position of the drivetrain
	 * @return the y position of the drivetrain in feet
	 */
	public double getYPosition() {
		return posY;
	}
	
	public double getVoltage() {
		return leftDriveMaster.getMotorOutputVoltage();
	}
	
	/**
	 * checks whether the current path is finished
	 * @return true if the path is finished, false otherwise
	 */
	public boolean isPFinished() {
		return (pInUse.isFinished());
	}
	
	/**
	 * stop turnPID when robot is at correct angle
	 * @return whether PID is at correct angle 
	 */
	public boolean atAngle() {
		return (Math.abs(turnPID.getSetpoint() - getAngle()) <= 5);
	}
	
	/**
	 * stop drivePID when robot is at correct distance
	 * @return whether PID is at correct distance 
	 */
	public boolean atDistance() {
		return (Math.abs(drivePID.getSetpoint() - getDistanceTraveled()) <= 2);
	}
	
	public void update() {
		
		//changing position and distance
		double distance = getDistanceTraveled()-lastDistance;
		
		posX += distance*Math.cos(Math.toRadians(getAngle()));
		posY += distance*Math.sin(Math.toRadians(getAngle()));
		
		distance = getDistanceTraveled();
		
		switch(mode) {
							
			//pathfinder driving 
			case AUTO_PATHFINDER:
				double[] p = pInUse.getOutput(rightEncoder.getDistance(), leftEncoder.getDistance(), getAngle()*Math.PI/180);
				
				leftDriveMaster.set(ControlMode.PercentOutput, p[1]); 
				rightDriveMaster.set(ControlMode.PercentOutput, -p[0]);
				leftDriveFollower1.set(ControlMode.PercentOutput, p[1]);
				rightDriveFollower1.set(ControlMode.PercentOutput, -p[0]);
				leftDriveFollower2.set(ControlMode.PercentOutput, p[1]);
				rightDriveFollower2.set(ControlMode.PercentOutput, -p[0]);
				break;
			
			case AUTO_MIRROR_PATHFINDER:
				double[] p_mirror = new double[2];
				p_mirror = pInUse.getOutput(leftEncoder.getDistance(), rightEncoder.getDistance(), -getAngle()*3.14/180); 

				leftDriveMaster.set(ControlMode.PercentOutput, p_mirror[0]);
				rightDriveMaster.set(ControlMode.PercentOutput, -p_mirror[1]);
				leftDriveFollower1.set(ControlMode.PercentOutput, p_mirror[0]);
				rightDriveFollower1.set(ControlMode.PercentOutput, -p_mirror[1]);
				leftDriveFollower2.set(ControlMode.PercentOutput, p_mirror[0]);
				rightDriveFollower2.set(ControlMode.PercentOutput, -p_mirror[1]);
				break;
			
			//drive straight with PID	
			case AUTO_DRIVE:
				leftspeed = drivePID.getOutput(getDistanceTraveled());
				rightspeed = -drivePID.getOutput(getDistanceTraveled());
				leftDriveMaster.set(ControlMode.PercentOutput, leftspeed);
				rightDriveMaster.set(ControlMode.PercentOutput, rightspeed);
				leftDriveFollower1.set(ControlMode.PercentOutput, leftspeed);
				rightDriveFollower1.set(ControlMode.PercentOutput, rightspeed);
				leftDriveFollower2.set(ControlMode.PercentOutput, leftspeed);
				rightDriveFollower2.set(ControlMode.PercentOutput, rightspeed);
				break;	
			
			
			//teleop driving with joystick control
			case TELEOP: 
				if (turning) {
					leftspeed = -turnPID.getOutput(navX.getAngle());
					rightspeed = turnPID.getOutput(navX.getAngle());
					
					if(leftspeed > capSpeed) {
						leftspeed = capSpeed;
					}
					if(rightspeed > capSpeed) {
						rightspeed = capSpeed;
					}
					if(leftspeed < -capSpeed) {
						leftspeed = -capSpeed;
					}
					if(rightspeed < -capSpeed) {
						rightspeed = -capSpeed;
					}
				}
				leftDriveMaster.set(ControlMode.PercentOutput, -leftspeed);
				rightDriveMaster.set(ControlMode.PercentOutput, rightspeed);
				leftDriveFollower1.set(ControlMode.PercentOutput, -leftspeed);
				rightDriveFollower1.set(ControlMode.PercentOutput, rightspeed);
				leftDriveFollower2.set(ControlMode.PercentOutput, -leftspeed);
				rightDriveFollower2.set(ControlMode.PercentOutput, rightspeed);
				break;
			
		}
	}
}

