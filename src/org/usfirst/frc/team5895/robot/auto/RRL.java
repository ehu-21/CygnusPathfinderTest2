package org.usfirst.frc.team5895.robot.auto;

import org.usfirst.frc.team5895.robot.Blinkin;
import org.usfirst.frc.team5895.robot.CubeIntake;
import org.usfirst.frc.team5895.robot.DriveTrain;
import org.usfirst.frc.team5895.robot.Elevator;
import org.usfirst.frc.team5895.robot.Limelight;
import org.usfirst.frc.team5895.robot.framework.Waiter;

/**
 * Right side of field, right switch, & left scale.
 * @author lalewis-19
 */
public class RRL {

	public static final void run(DriveTrain drive, Elevator elevator, Limelight lime, CubeIntake intake,
			Blinkin blinkin) {
		
		// switch > near
		// scale > far
		
		drive.resetNavX();
		intake.intake();
		Waiter.waitFor(200);
		drive.autoRightLeftScale();
		Waiter.waitFor(3000);
		elevator.setTargetPosition(82/12);
		Waiter.waitFor(drive::isPFinished, 4000);
		intake.ejectSlow();
		
	}

}
