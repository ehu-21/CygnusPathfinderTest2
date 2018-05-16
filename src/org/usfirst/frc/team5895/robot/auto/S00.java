package org.usfirst.frc.team5895.robot.auto;

import org.usfirst.frc.team5895.robot.Blinkin;
import org.usfirst.frc.team5895.robot.CubeIntake;
import org.usfirst.frc.team5895.robot.DriveTrain;
import org.usfirst.frc.team5895.robot.Elevator;
import org.usfirst.frc.team5895.robot.IntakeV2;
import org.usfirst.frc.team5895.robot.Limelight;
import org.usfirst.frc.team5895.robot.framework.Waiter;

import edu.wpi.first.wpilibj.DriverStation;

/**
 * center of field, left switch, & left scale.
 * @author lalewis-19
 */
public class S00 {
	
	public static final void run(DriveTrain drive, Elevator elevator, Limelight lime, IntakeV2 intake,
			Blinkin blinkin) {
		
		drive.resetNavX();
		intake.intake();
		Waiter.waitFor(200);
		DriverStation.reportError("" + drive.getAngle(), false);
		drive.autoForwardStraight();
		Waiter.waitFor(drive::isPFinished, 5000);
		DriverStation.reportError("" + drive.getAngle(), false);
		drive.arcadeDrive(0, 0);
	}

}
