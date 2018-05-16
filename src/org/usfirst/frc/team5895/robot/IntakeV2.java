package org.usfirst.frc.team5895.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;

public class IntakeV2 {
	private static enum Mode_Type {INTAKING, EJECTING, HOLDING, SPINNING_RIGHT, SPINNING_LEFT, DROPPING, DISABLED, OPEN_INTAKING};
	private Mode_Type mode = Mode_Type.DISABLED;
	private VictorSPX leftClawMotor, rightClawMotor;
	private AnalogInput leftClawSensor, rightClawSensor;
	private double leftSpeed, rightSpeed;
	private double lastTime;
	private boolean isClamped, isTensioned;
	public boolean isDown;
	private Solenoid clawSolenoid, clampSolenoid, tensionSolenoid;
	private double ejectSpeed;
	
	public IntakeV2() {
		leftClawMotor = new VictorSPX(ElectricalLayout.MOTOR_CLAW_1);
		rightClawMotor = new VictorSPX(ElectricalLayout.MOTOR_CLAW_2);
		
		leftClawSensor = new AnalogInput(ElectricalLayout.SENSOR_INTAKE_LEFT);
		rightClawSensor = new AnalogInput(ElectricalLayout.SENSOR_INTAKE_RIGHT);
		

		clawSolenoid = new Solenoid(ElectricalLayout.SOLENOID_INTAKE_CLAW);
		clampSolenoid = new Solenoid(ElectricalLayout.SOLENOID_INTAKE_CLAMP);
		tensionSolenoid = new Solenoid(ElectricalLayout.SOLENOID_INTAKE_TENSION);

	    isDown = false;
	    
		leftClawMotor.setInverted(true);
		rightClawMotor.setInverted(false);
		
		}

	/**
	 * sets the claw to intaking mode
	 */
	public void intake(){
		mode = Mode_Type.INTAKING;
		}
	
	public void openIntaking(){
		mode = Mode_Type.OPEN_INTAKING;
		}
	
	/**
	 * ejects a cube at full speed
	 */
	public void ejectFast(){
		ejectSpeed = 1.0;
		mode= Mode_Type.EJECTING;
		lastTime = Timer.getFPGATimestamp();
		}
	
	/**
	 * ejects a cube at half speed
	 */
	public void ejectSlow() {
		ejectSpeed = 0.55;
		mode= Mode_Type.EJECTING;
		lastTime = Timer.getFPGATimestamp();
		}
	
	public void ejectCustom(double speed){
		ejectSpeed = speed;
		mode= Mode_Type.EJECTING;
		lastTime = Timer.getFPGATimestamp();
		}
	
	public void drop() {
		mode = Mode_Type.DROPPING;
	}
	
	/**
	 * disables claw
	 */
	public void disable(){
		mode = Mode_Type.DISABLED;
	}
	
	/**
	 * lifts claw
	 */
	public void up() {
		isDown = false;
		mode = Mode_Type.HOLDING;
	}
	
	/**
	 * drops claw
	 */
	public void down(){
		isDown = true;
		mode = Mode_Type.INTAKING;
	}
	
	public boolean hasCube() {
		return (leftClawSensor.getVoltage() > 2.7 && rightClawSensor.getVoltage() > 2.7);
	}
	
	/**
	 * @return true if the cube is turned left in the intake, false otherwise
	 */

	public double getLeftVoltage() {
		return leftClawSensor.getVoltage();
	}
	
	public double getRightVoltage() {
		return rightClawSensor.getVoltage();
	}
	
	public double getState() {
		return mode.ordinal();
	}
	
	public void update(){
		
//		DriverStation.reportError("" + getRightVoltage(), false);
		
		switch(mode) {
		
		case INTAKING:
		     
		     leftSpeed = 1.0;
		     rightSpeed = 1.0;
		     
		     if(hasCube()) { //get rid off lastHasCube
			 	mode = Mode_Type.HOLDING; //once we have the cube, we prepare to hold and clamp
			 }
			isClamped = false; //solenoid only clamps once it is holding 
			isTensioned = false; 
			break;
		
		case HOLDING:
			leftSpeed = 0.0;
			rightSpeed = 0.0;
			isClamped = true; // solenoid used to clamp cube while holding 
			isTensioned = false;
		//	DriverStation.reportError("holding", false);
			break;
		
		case EJECTING:
			leftSpeed = -ejectSpeed;
			rightSpeed = -ejectSpeed;
			isClamped = false; 
			isTensioned = false;
			double waitTime = Timer.getFPGATimestamp(); //stamps current time 
            if (waitTime - lastTime > 0.6) { //compares the time we started waiting to current time
            	mode = Mode_Type.INTAKING; //if it has been waiting for 200ms, it begins to hold
            } else {
            	mode = Mode_Type.EJECTING; //if not, it keeps waiting
            }
			break; 
		
		case DROPPING:
			leftSpeed = 0;
			rightSpeed = 0;
			isClamped = false; 
			isTensioned = true;
			break;
			
		case DISABLED:
			leftSpeed = 0;
			rightSpeed = 0;
			isClamped = false;
			isTensioned = false;
			break;
			
		case OPEN_INTAKING:
		     leftSpeed = 1.0;
		     rightSpeed = 1.0;
		     
		     if(hasCube()) { //get rid off lastHasCube
			 	mode = Mode_Type.HOLDING; //once we have the cube, we prepare to hold and clamp
			 }
			isClamped = false; //solenoid only clamps once it is holding 
			isTensioned = true; 
			break;
			
		}
		
		leftClawMotor.set(ControlMode.PercentOutput, leftSpeed);
		rightClawMotor.set(ControlMode.PercentOutput, rightSpeed);
		
		clawSolenoid.set(isDown);
		clampSolenoid.set(isClamped);
		tensionSolenoid.set(isTensioned);
		
	}
}