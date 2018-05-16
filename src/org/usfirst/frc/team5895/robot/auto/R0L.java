package org.usfirst.frc.team5895.robot.auto;

import org.usfirst.frc.team5895.robot.Blinkin;
import org.usfirst.frc.team5895.robot.CubeIntake;
import org.usfirst.frc.team5895.robot.DriveTrain;
import org.usfirst.frc.team5895.robot.Elevator;
import org.usfirst.frc.team5895.robot.IntakeV2;
import org.usfirst.frc.team5895.robot.Limelight;
import org.usfirst.frc.team5895.robot.framework.Waiter;

public class R0L {

	public static final void run(DriveTrain drive, Elevator elevator, Limelight lime, IntakeV2 intake,
			Blinkin blinkin) {
		drive.resetNavX();
		intake.intake();
		Waiter.waitFor(200);
		drive.autoRightLeftScale();
		Waiter.waitFor(3000);
		elevator.setTargetPosition(82/12);
		Waiter.waitFor(drive::isPFinished, 4000);
		Waiter.waitFor(1000);
		intake.ejectCustom(0.7);
		drive.autoLeftScaleBackwards();
		Waiter.waitFor(drive::isPFinished, 4000);
		drive.arcadeDrive(0, 0);
}
}
