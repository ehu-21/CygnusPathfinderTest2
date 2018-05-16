package org.usfirst.frc.team5895.robot.auto;

import org.usfirst.frc.team5895.robot.Blinkin;
import org.usfirst.frc.team5895.robot.DriveTrain;
import org.usfirst.frc.team5895.robot.Elevator;
import org.usfirst.frc.team5895.robot.IntakeV2;
import org.usfirst.frc.team5895.robot.Limelight;
import org.usfirst.frc.team5895.robot.framework.Waiter;

import edu.wpi.first.wpilibj.DriverStation;

public class L0D {

	public static final void run(DriveTrain drive, Elevator elevator, Limelight lime, IntakeV2 intake,
			Blinkin blinkin) {
		DriverStation.reportError("boop", false);
		drive.resetNavX();
		intake.intake();
		Waiter.waitFor(200);
		drive.autoLeftRightDrive();
		Waiter.waitFor(3000);
		elevator.setTargetPosition(82/12);
		Waiter.waitFor(drive::isPFinished, 7000);
	}
	
}
