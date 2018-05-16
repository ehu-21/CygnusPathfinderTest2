package org.usfirst.frc.team5895.robot.framework;

import java.util.function.Supplier;
import edu.wpi.first.wpilibj.Timer;


//To do: investigate better way to wait than Thread.sleep()
public class Waiter {
	
	/**
	 * Waits for the specified amount of time
	 * 
	 * @param time The amount of time to wait in milliseconds
	 */
	public static void waitFor(double time) {
		double timeSeconds = time / 1000;
		Timer.delay(timeSeconds);
	}
	
	/**
	 * Waits for the specified method to return true, or for the specified time to elapse
	 * with a default precision of 10 milliseconds
	 * 
	 * @param condition The method to check
	 * @param time The amount of time to wait in milliseconds
	 */
	public static void waitFor(Supplier<Boolean> condition, double time) {
		waitFor(condition, time, 10);
	}
	
	/**
	 * Waits for the specified method to return true, or for the specified time to elapse
	 * 
	 * @param condition The method to check
	 * @param time The amount of time to wait in milliseconds
	 * @param precision The amount of time in milliseconds between checks to the condition method
	 */
	public static void waitFor(Supplier<Boolean> condition, double time, double precision) {
		double timeSeconds = time / 1000;
		double precisionSeconds = precision / 1000;
		double start = Timer.getFPGATimestamp();
		while (Timer.getFPGATimestamp() - start < timeSeconds) {
			if (condition.get()) {
				return;
			}
			Timer.delay(precisionSeconds);
		}
		return;
	}
	
}
