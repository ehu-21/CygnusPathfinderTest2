package org.usfirst.frc.team5895.robot.auto;

import org.usfirst.frc.team5895.robot.Blinkin;
import org.usfirst.frc.team5895.robot.CubeIntake;
import org.usfirst.frc.team5895.robot.DriveTrain;
import org.usfirst.frc.team5895.robot.Elevator;
import org.usfirst.frc.team5895.robot.IntakeV2;
import org.usfirst.frc.team5895.robot.Limelight;
import org.usfirst.frc.team5895.robot.framework.Waiter;

public class L0L {

	public static final void run(DriveTrain drive, Elevator elevator, Limelight lime, IntakeV2 intake,
			Blinkin blinkin) {
		
		drive.resetNavX();
		intake.intake();
		Waiter.waitFor(200);
		drive.autoLeftLeftScale();
		Waiter.waitFor(1000);
		elevator.setTargetPosition(82/12);
		Waiter.waitFor(drive::isPFinished, 5000);
		intake.ejectSlow();
		Waiter.waitFor(500);
	}
}
