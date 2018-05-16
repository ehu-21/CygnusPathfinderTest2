package org.usfirst.frc.team5895.robot.lib;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;

public class ThreeMotors {
	
	BaseMotorController MotorA;
	BaseMotorController MotorB;
	BaseMotorController MotorC;
	
	public ThreeMotors(BaseMotorController A, BaseMotorController B, BaseMotorController C) {
		MotorA = A;
		MotorB = B;
		MotorC = C;
	}
	
	public void set(double speed) {
		MotorA.set(ControlMode.PercentOutput, speed);
		MotorB.set(ControlMode.PercentOutput, speed);
		MotorC.set(ControlMode.PercentOutput, speed);
	}

}
