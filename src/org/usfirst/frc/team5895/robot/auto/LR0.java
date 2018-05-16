package org.usfirst.frc.team5895.robot.auto;

import org.usfirst.frc.team5895.robot.Blinkin;
import org.usfirst.frc.team5895.robot.CubeIntake;
import org.usfirst.frc.team5895.robot.DriveTrain;
import org.usfirst.frc.team5895.robot.Elevator;
import org.usfirst.frc.team5895.robot.Limelight;
import org.usfirst.frc.team5895.robot.framework.Waiter;

/**
 * center of field, left switch, & left scale.
 * @author lalewis-19
 */
public class LR0 {
	
	public static final void run(DriveTrain drive, Elevator elevator, Limelight lime, CubeIntake intake,
			Blinkin blinkin) {
		
		drive.resetNavX();
		intake.intake();
		Waiter.waitFor(200);
		elevator.setTargetPosition(40.0/12);
		intake.down();
		drive.autoLeftRightSwitchFront();
		Waiter.waitFor(drive::isPFinished, 4000);
		intake.ejectFast();
		
	}

}
