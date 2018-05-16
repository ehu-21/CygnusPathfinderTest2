package org.usfirst.frc.team5895.robot.framework;

import java.util.Timer;
import java.util.TimerTask;
import java.util.Vector;

public class Looper {
	
	private Timer m_timer;
	private long m_period;
	private Vector<Runnable> m_methods;
	
	/**
	 * Creates a new Looper object
	 * 
	 * @param period The period (in ms) between updates
	 */
	public Looper(long period) {
		m_period = period;
		m_methods = new Vector<Runnable>();
	}
	
	private class LooperTask extends TimerTask {

        private Looper looper;

        public LooperTask(Looper looper) {
            if (looper == null) {
                throw new NullPointerException("Given Looper was null");
            }
            this.looper = looper;
        }

        public void run() {
            looper.update();
        }
    }
	
	/**
	 * Start updating
	 */
	public void start() {
		m_timer = new Timer();
		m_timer.schedule(new LooperTask(this), 0L, m_period);
	}
	
	/**
	 * Adds a method to the list to be periodically called
	 * 
	 * @param method The method to be called
	 */
	public void add(Runnable method) {
		m_methods.add(method);
	}
	
	private void update() {
		for (Runnable subsystem : m_methods) {
			subsystem.run();
		}
	}
}