package org.usfirst.frc.team5895.robot.lib;

import edu.wpi.first.wpilibj.DigitalInput;

public class BetterDigitalInput extends DigitalInput {

	private boolean lastValRising;
	private boolean lastValFalling;
	
	public BetterDigitalInput(int port) {
		super(port);
	}
	
	public boolean getRisingEdge() {
		boolean cur = this.get();
		boolean ret = !lastValRising && cur;
		lastValRising = cur;
		return ret;
	}
	
	public boolean getFallingEdge() {
		boolean cur = this.get();
		boolean ret = lastValFalling && !cur;
		lastValFalling = cur;
		return ret;
	}

}
