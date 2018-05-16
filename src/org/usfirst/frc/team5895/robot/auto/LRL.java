package org.usfirst.frc.team5895.robot.auto;

import org.usfirst.frc.team5895.robot.Blinkin;
import org.usfirst.frc.team5895.robot.CubeIntake;
import org.usfirst.frc.team5895.robot.DriveTrain;
import org.usfirst.frc.team5895.robot.Elevator;
import org.usfirst.frc.team5895.robot.Limelight;
import org.usfirst.frc.team5895.robot.framework.Waiter;

/**
 * Left side of field, right switch, & left scale.
 * @author lalewis-19
 */
public class LRL {
	
	public static final void run(DriveTrain drive, Elevator elevator, Limelight lime, CubeIntake intake,
			Blinkin blinkin) {
		
		// switch > far
		// scale > near
		
		drive.resetNavX();
		intake.intake();
		Waiter.waitFor(200);
		drive.autoLeftLeftScale();
		Waiter.waitFor(1000);
		elevator.setTargetPosition(82/12);
		intake.up();
		Waiter.waitFor(drive::isPFinished, 2500);
		intake.ejectFast();
		Waiter.waitFor(500);
		drive.turnTo(120);
		elevator.setTargetPosition(0.0);
		Waiter.waitFor(drive::atAngle,3000);
		drive.stopTurning();
		drive.arcadeDrive(0, 0);
		drive.autoLeftLeftScaleRightSwitch();
		intake.down();
		intake.intake();
		Waiter.waitFor(drive::isPFinished, 3000);
		drive.arcadeDrive(0, 0);
		lime.autoSeek(intake, drive);
		Waiter.waitFor(1000);
		elevator.setTargetPosition(40.0/12);
		drive.arcadeDrive(-0.1, 0);
		Waiter.waitFor(200);
		drive.arcadeDrive(0.1, 0);
		Waiter.waitFor(200);
		intake.ejectFast();
	}

}
