package org.usfirst.frc.team5895.robot;

import org.usfirst.frc.team5895.robot.lib.TrajectoryDriveController;
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
import edu.wpi.first.wpilibj.Talon;

public class DriveTrainBasan {

	private Talon leftDriveMaster, rightDriveMaster;
	private BetterEncoder leftEncoder,rightEncoder;
	private double leftspeed, rightspeed;
	private NavX navX;
	
	private PathfinderGenerator pathMaster;
	private PathfinderFollower rStraight, rTest, rRightRightScale, rRightRightSwitch, rRight_SCurve; 
	private PathfinderFollower rInUse;
	
	private enum Mode_Type {TELEOP, AUTO_PATHFINDER};
	private Mode_Type mode = Mode_Type.TELEOP;
	
	public DriveTrainBasan() {
		
		//initialize drive motors
		leftDriveMaster = new Talon(0);
		rightDriveMaster = new Talon(1);
			
		//initialize encoders
		leftEncoder = new BetterEncoder(0,1, true, Encoder.EncodingType.k4X);
		rightEncoder = new BetterEncoder(2,3, false, Encoder.EncodingType.k4X);
		
		//set encoder distance per pulse so it's in feet
		leftEncoder.setDistancePerPulse(4/12.0*Math.PI/360); //correct
		rightEncoder.setDistancePerPulse(4/12.0*Math.PI/360); //correct
		
		//initialize navx
		navX = new NavX();
		
		//initializes pathfinder
		pathMaster = new PathfinderGenerator(true); 
		try {
			//numbers for Basan
			DriverStation.reportError("Start Making Trajectory", false);
			//drive straight
			rStraight = new PathfinderFollower(pathMaster.Straight(), 0.0, 0, 0, 1.0/11.35, 1.0/32.0, -0.0);
			rRightRightScale = new PathfinderFollower(pathMaster.RightRightScale(), 0.1, 0, 0, 1.0/11.5, 1.0/35.0, -0.01);
			//rRightRightSwitch = new PathfinderFollower(pathMaster.RightRightSwitch(), 0.1, 0, 0, 1.0/11.3, 1.0/35.0, -0.01);
			//rRight_SCurve = new PathfinderFollower(pathMaster.RightSCurve(), 0.1, 0, 0, 1.0/11.3, 1.0/35.0, -0.01);
			
			//test path
			//rTest = new PathfinderFollower(pathMaster.Straight(), 0.1, 0, 0, 1.0/13, 1.0/45.0, -0.01);
			
		}catch (Exception e) {
			DriverStation.reportError("Pathfinder: Error generating path!",false);
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
	 * gets the current angle of the NavX
	 * @return the NavX angle in degrees
	 */
	public double getAngle() {
		return navX.getAngle(); // navX reads angle in degree
	}
	
	/**
	 * drive straight using pathfinder
	 */
	public void auto_forward_straight() {
		resetEncoders();
		rInUse = rStraight;
		mode = Mode_Type.AUTO_PATHFINDER;
	}
	
	public void right_SCurve() {
		resetEncoders();
		rInUse = rRight_SCurve;
		mode = Mode_Type.AUTO_PATHFINDER;
	}
	
	public void right_right_scale() {
		resetEncoders();
		rInUse = rRightRightScale; 
		mode = Mode_Type.AUTO_PATHFINDER;
	}
	
	public void right_right_switch() {
		resetEncoders();
		rInUse = rRightRightSwitch;
		mode = Mode_Type.AUTO_PATHFINDER;
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
	
	public double getPathfinderVelocity() {
		return (rInUse.getSegment().velocity);
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
	}
	
	public void update() {
		
		switch(mode) { //This sign is for Basan, Invert the sign for Cygnus
				
			// pathfinder driving	
			case AUTO_PATHFINDER:
				double[] p = rInUse.getOutput(leftEncoder.getDistance(), rightEncoder.getDistance(), -getAngle()*Math.PI/180);
				
				leftDriveMaster.set(-p[0]);
				rightDriveMaster.set(p[1]);
				
				break;
				
			//teleop driving with joystick control
			case TELEOP: 
				leftDriveMaster.set(-leftspeed);
				rightDriveMaster.set(rightspeed);
				break;
			
		}
	}

}