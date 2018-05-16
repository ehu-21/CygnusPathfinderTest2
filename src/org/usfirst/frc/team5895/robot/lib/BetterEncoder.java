package org.usfirst.frc.team5895.robot.lib;

import edu.wpi.first.wpilibj.Encoder;

public class BetterEncoder extends Encoder {
	
	double offset = 0.0;
	
	public BetterEncoder(int turretEncoder, int turretEncoder2,
			boolean reverseDirection, EncodingType type) {
		super(turretEncoder, turretEncoder2, reverseDirection, type);
	}

/**
	 * Resets the encoder to val
	 * @param val The value the encoder should currently be at
	 */
	public void setTo(double val) {
		offset = val - super.getDistance();
	}
	
	public double getDistance() {
		return offset + super.getDistance();
	}
	
	public void reset() {
		offset = 0;
		super.reset();
	}

}
