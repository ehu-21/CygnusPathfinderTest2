package org.usfirst.frc.team5895.robot;

import java.util.HashMap;

import org.usfirst.frc.team5895.robot.auto.*;
import org.usfirst.frc.team5895.robot.framework.Looper;
import org.usfirst.frc.team5895.robot.framework.LooperV2;
import org.usfirst.frc.team5895.robot.framework.Recorder;
import org.usfirst.frc.team5895.robot.framework.Waiter;
import org.usfirst.frc.team5895.robot.lib.BetterJoystick;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Robot extends IterativeRobot {

	LooperV2 loop;
	Elevator elevator;
	IntakeV2 intake;
	DriveTrainV3 drive;
	Blinkin blinkin;
	Limelight lime;
	GameData gameData;
	PowerDistributionPanel pdp;
	
	boolean fastShoot = false;
	boolean isDown = false;

	Recorder r;
	HashMap<String, Runnable> autoRoutines;
	
	BetterJoystick leftJoystick;
	BetterJoystick rightJoystick;
	BetterJoystick operatorJoystick;

	public void robotInit() {
		
		leftJoystick = new BetterJoystick(0);
		rightJoystick = new BetterJoystick(1);
		operatorJoystick = new BetterJoystick(2);
		
		intake = new IntakeV2();
		elevator = new Elevator();
		drive = new DriveTrainV3();
		//blinkin = new Blinkin();
		//gameData = new GameData();
		//lime = new Limelight();
		pdp = new PowerDistributionPanel();
		
	//	lime.setLedMode(Limelight.LedMode.ON);
		//blinkin.lightsNormal();
	
		//set up recorder
		r = new Recorder(10);
		r.add("Time", Timer::getFPGATimestamp);
		r.add("Drive Distance", drive::getDistanceTraveled);
		r.add("Drive Velocity", drive::getVelocity);
	//	r.add("Drive Left Velocity", drive::getLeftVelocity);
	//	r.add("Drive Right Velocity", drive::getRightVelocity);
		r.add("Elevator Height", elevator::getHeight);
		r.add("Elevator State", elevator::getState);
		r.add("Intake LeftClawSensor", intake::getLeftVoltage);
		r.add("Intake RightClawSensor", intake::getRightVoltage);
		r.add("Intake State", intake::getState);
		for (int i = 0; i < 16; i++) {
			final int x = i;
			r.add("Current " + i, () -> pdp.getCurrent(x));
		}
//		r.add("AVuto Routine", gameData::getAutoRoutine);
//		r.add("Game Data", gameData::getGameData);
		
		loop = new LooperV2(10);
		loop.add(elevator::update);
		loop.add(intake::update);
		loop.add(drive::update);
	//	loop.add(lime::update);
//		loop.add(r::record);
		loop.start();
		
		//set up auto map
		/*autoRoutines = new HashMap<String, Runnable>();
		autoRoutines.put("S00", () -> S00.run(drive, elevator, lime, intake, blinkin));
		autoRoutines.put("CR0", () -> CR0.run(drive, elevator, lime, intake, blinkin));
		autoRoutines.put("CL0", () -> CL0.run(drive, elevator, lime, intake, blinkin));
		
		autoRoutines.put("L0R", () -> L0R.run(drive, elevator, lime, intake, blinkin));
		autoRoutines.put("L0L", () -> L0L.run(drive, elevator, lime, intake, blinkin));
		autoRoutines.put("R0R", () -> R0R.run(drive, elevator, lime, intake, blinkin));
		autoRoutines.put("R0L", () -> R0L.run(drive, elevator, lime, intake, blinkin));
		
		autoRoutines.put("LLL", () -> LLL.run(drive, elevator, lime, intake, blinkin));
		autoRoutines.put("LRL", () -> LLL.run(drive, elevator, lime, intake, blinkin));
		autoRoutines.put("LRR", () -> L0D.run(drive, elevator, lime, intake, blinkin));
		autoRoutines.put("LLR", () -> L0D.run(drive, elevator, lime, intake, blinkin));
		
		autoRoutines.put("RRR", () -> RRR.run(drive, elevator, lime, intake, blinkin));
		autoRoutines.put("RLL", () -> R0L.run(drive, elevator, lime, intake, blinkin));
		autoRoutines.put("RLR", () -> RRR.run(drive, elevator, lime, intake, blinkin));
		autoRoutines.put("RRL", () -> R0L.run(drive, elevator, lime, intake, blinkin));
		*/
	}

	public void autonomousInit() {
		
		DriverStation.reportError("starting to record", false);
		r.startRecording();
//		drive.resetNavX();
//		drive.autoForwardStraight();
		/*String autoRoutine = gameData.getAutoRoutine();
		
		DriverStation.reportError("" + gameData.getAutoRoutine(), false);
		DriverStation.reportError("" + drive.getDistanceTraveled(), false);
		
		if (autoRoutines.containsKey(autoRoutine)) {
			autoRoutines.get(autoRoutine).run();
		} else {
			DriverStation.reportError("Auto Error: " + autoRoutine, false);
			S00.run(drive, elevator, lime, intake, blinkin);
		}	*/
		
		DriverStation.reportError("reseting navx", false);
		drive.resetNavX();
		
		DriverStation.reportError("running drive straight now", false);
		drive.straight();
		
	}

	public void teleopPeriodic() {

		DriverStation.reportError("" + intake.getLeftVoltage(), false);
		
		//teleop drive
		drive.arcadeDrive(leftJoystick.getRawAxis(1), rightJoystick.getRawAxis(0));
		
		//left joystick controls
		if(leftJoystick.getRisingEdge(1)) {
			if(intake.isDown) {
				intake.up();
			} else {
				intake.down();
			}
		} else if(leftJoystick.getRisingEdge(2)){
			if ((Math.abs(elevator.getHeight() - 0.54) < 0.2) || (Math.abs(elevator.getHeight() - 15.0/12) < 0.2)) {
				intake.openIntaking();
			}
			else {
				intake.drop();
			}
		} else if(leftJoystick.getFallingEdge(2)) {
			intake.intake();
		}
		else if(leftJoystick.getRisingEdge(3)) {
			elevator.setTargetPosition(0.54);
		}
		else if(leftJoystick.getRisingEdge(4)) {
			elevator.setTargetPosition(15.0/12);
		}  
		
		//right joystick controls
		if(rightJoystick.getRisingEdge(1)) {
			if(fastShoot) {
				intake.ejectCustom(SmartDashboard.getNumber("DB/Slider 0", 1.0));
			} else {
				intake.ejectSlow();
			}		
		} else if(rightJoystick.getRisingEdge(2)) {
			elevator.setTargetPosition(74.0/12);
		} else if(rightJoystick.getRisingEdge(3)) {
			elevator.setTargetPosition(32.0/12);
		} else if(rightJoystick.getRisingEdge(4)) {
			elevator.setTargetPosition(79.0/12);
		}
	
		//operator joystick controls
		if(operatorJoystick.getRisingEdge(1)) {
			elevator.climb();
		}
		if(operatorJoystick.getRisingEdge(2)) {
			elevator.brake();
		}
		if(operatorJoystick.getRisingEdge(3)) {
			fastShoot = true;
		} else if(operatorJoystick.getRisingEdge(4)) {
			fastShoot = false;
		}
		
	}

	public void disabledInit() {
		r.stopRecording();
		drive.arcadeDrive(0, 0);
	}

}