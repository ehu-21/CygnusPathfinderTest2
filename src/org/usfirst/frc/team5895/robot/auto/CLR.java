package org.usfirst.frc.team5895.robot.auto;

import org.usfirst.frc.team5895.robot.Blinkin;
import org.usfirst.frc.team5895.robot.CubeIntake;
import org.usfirst.frc.team5895.robot.DriveTrain;
import org.usfirst.frc.team5895.robot.Elevator;
import org.usfirst.frc.team5895.robot.Limelight;
import org.usfirst.frc.team5895.robot.framework.Waiter;

/**
 * center of field, left switch, & right scale.
 * @author lalewis-19
 */
public class CLR {
	
	public static final void run(DriveTrain drive, Elevator elevator, Limelight lime, CubeIntake intake,
			Blinkin blinkin) {
		
		drive.resetNavX();
		intake.intake();
		Waiter.waitFor(200);
		drive.autoCenterRightScale();
		Waiter.waitFor(1000);
		elevator.setTargetPosition(82/12);
		Waiter.waitFor(drive::isPFinished, 2500);
		intake.ejectSlow();
		Waiter.waitFor(500);
		drive.turnTo(-120);
		elevator.setTargetPosition(0.0);
		Waiter.waitFor(drive::atAngle,3000);
		drive.stopTurning();
		drive.arcadeDrive(0, 0);
		drive.autoRightRightScaleLeftSwitch();
		intake.down();
		intake.intake();
		Waiter.waitFor(drive::isPFinished, 5000);
		drive.arcadeDrive(0, 0);
		lime.autoSeek(intake, drive);
		Waiter.waitFor(500);
		elevator.setTargetPosition(40.0/12);
		drive.arcadeDrive(-0.1, 0.0);
		Waiter.waitFor(500);
		drive.arcadeDrive(0, 0);
		Waiter.waitFor(2000);
		intake.ejectFast();
	}

}
