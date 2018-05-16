package org.usfirst.frc.team5895.robot.auto;

import org.usfirst.frc.team5895.robot.Blinkin;
import org.usfirst.frc.team5895.robot.CubeIntake;
import org.usfirst.frc.team5895.robot.DriveTrain;
import org.usfirst.frc.team5895.robot.Elevator;
import org.usfirst.frc.team5895.robot.Limelight;
import org.usfirst.frc.team5895.robot.framework.Waiter;

/**
 * Left side of field, right switch, & right scale.
 * @author lalewis-19
 */
public class LRR {
	
	public static final  void run(DriveTrain drive, Elevator elevator, Limelight lime, CubeIntake intake,
			Blinkin blinkin) {
		
		// switch > far
		// scale > far
		
		drive.resetNavX();
		intake.intake();
		Waiter.waitFor(200);
		drive.autoLeftRightScale();
		Waiter.waitFor(3000);
		elevator.setTargetPosition(82/12);
		Waiter.waitFor(drive::isPFinished, 4000);
		intake.ejectSlow();
		Waiter.waitFor(500);
		elevator.setTargetPosition(0.0);
		drive.turnTo(-165);
		Waiter.waitFor(drive::atAngle, 3000);
		drive.stopTurning();
		drive.arcadeDrive(0,0);
		Waiter.waitFor(500);
		lime.autoSeek(intake, drive);
		Waiter.waitFor(1000);
		drive.arcadeDrive(0.1, 0);
		elevator.setTargetPosition(40.0/12);
		Waiter.waitFor(500);
		drive.arcadeDrive(0, 0);
		Waiter.waitFor(1500);
		intake.ejectFast();
		
	}

}
