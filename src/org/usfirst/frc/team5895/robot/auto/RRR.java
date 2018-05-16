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
 * Right Side of field, right switch, & right scale.
 */
public class RRR {
	
	public static final void run(DriveTrain drive, Elevator elevator, Limelight lime, IntakeV2 intake,
			Blinkin blinkin) {
		
		drive.resetNavX();
		intake.intake();
		drive.autoRightRightScale();
		Waiter.waitFor(1000);
		elevator.setTargetPosition(82/12);
		Waiter.waitFor(drive::isPFinished, 2500);
		Waiter.waitFor(500);
		intake.ejectSlow();
		DriverStation.reportError("" + drive.getAngle(), false);
		Waiter.waitFor(200);
		drive.autoRightScaleBackwards();
		elevator.setTargetPosition(0.54);
		intake.down();
		intake.intake();
		Waiter.waitFor(drive::isPFinished, 2000);
		drive.turnTo(-160);
		Waiter.waitFor(drive::atAngle, 4000);
		drive.stopTurning(); 
		drive.arcadeDrive(0, 0);
		drive.autoRightScaleCube();
		Waiter.waitFor(drive::isPFinished, 5000);
		drive.autoRightScaleCubeBack();
		Waiter.waitFor(drive::isPFinished, 5000);
		intake.up();
		drive.turnTo(-30);
		elevator.setTargetPosition(82/12);
		Waiter.waitFor(drive::atAngle, 4000);
		drive.stopTurning(); 
		drive.arcadeDrive(0, 0);
		drive.autoRightScaleForwards();
		Waiter.waitFor(drive::isPFinished, 5000);
		intake.ejectSlow();
		}
	
}
