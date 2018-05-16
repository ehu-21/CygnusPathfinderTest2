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
 * Left side of field, left switch, & left scale.
 */
public class LLL {
	
	public static final void run(DriveTrain drive, Elevator elevator, Limelight lime, IntakeV2 intake,
			Blinkin blinkin) {
		
		drive.resetNavX();
		intake.intake();
		drive.autoLeftLeftScale();
		Waiter.waitFor(1000);
		elevator.setTargetPosition(82/12);
		Waiter.waitFor(drive::isPFinished, 2500);		
		Waiter.waitFor(500);
		intake.ejectSlow();
		DriverStation.reportError("" + drive.getAngle(), false);
		Waiter.waitFor(200);
		drive.autoLeftScaleBackwards();
		elevator.setTargetPosition(0.54);
		intake.down();
		intake.intake();
		Waiter.waitFor(drive::isPFinished, 2000);
		drive.turnTo(160);
		Waiter.waitFor(drive::atAngle, 4000);
		drive.stopTurning(); 
		drive.arcadeDrive(0, 0);
		drive.autoLeftScaleCube();
		Waiter.waitFor(drive::isPFinished, 5000);
		drive.autoLeftScaleCubeBack();
		Waiter.waitFor(drive::isPFinished, 5000);
		drive.turnTo(30);
		elevator.setTargetPosition(82/12);
		Waiter.waitFor(drive::atAngle, 4000);
		drive.stopTurning(); 
		drive.arcadeDrive(0, 0);
		drive.autoLeftScaleForwards();
		Waiter.waitFor(drive::isPFinished, 5000);
		intake.ejectSlow();
	}

}
