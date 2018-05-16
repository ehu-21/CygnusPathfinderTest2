package org.usfirst.frc.team5895.robot.lib;

import edu.wpi.first.wpilibj.AnalogInput;

public class DistanceSensor {
	private AnalogInput analogSensor;
	
	public DistanceSensor(int port) {
		analogSensor = new AnalogInput(port);
	}
	
	/** 
	 * Returns the distance of the analog distance sensor
	 * 
	 * @return The distance, in inches, from the box to the claw
	 */
	public double getDistance() {
		
		double volts = analogSensor.getVoltage();
		double distance = (27.2959* Math.pow(volts,-1.18304)/2.54);
		
		return distance;
	}
}
