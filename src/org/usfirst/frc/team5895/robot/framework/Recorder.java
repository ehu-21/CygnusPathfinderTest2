package org.usfirst.frc.team5895.robot.framework;

import java.io.File;
import java.io.FileNotFoundException;
import java.util.Formatter;
import java.util.FormatterClosedException;
import java.util.Scanner;
import java.util.Vector;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.DriverStation;


public class Recorder {		
	private boolean recordFile;
	private Formatter f;
	private Vector<Supplier<Object>> methods;
	private Vector<String> names;
	
	
	/**
	 * Makes a new Recorder
	 * @param time is the number of milliseconds between writes to the log file
	 */
	public Recorder(int time){
		recordFile = false;
		methods = new Vector<Supplier<Object>>();
		names = new Vector<String>();
	}

	/** 
	 * Adds a variable to be printed to the log file
	 * @param name
	 * @param g
	 */
	public void add(String name, Supplier<Object> g) {
		if (!recordFile) {
			methods.add(g);
			names.add(name);
		}
	}
	
	
	/**
	 * Creates a file for saving data and makes separate columns for everything
	 * @param filename The name of the log file to create, fails if file
	 * 			already exists
	 */
	public void startRecording() {
    	try {
    		if (recordFile==false) {

    			String filename = "Log" + incrementCount() + ".csv";
    			
    			f= new Formatter("/home/lvuser/logs/" + filename);
    			StringBuilder titles = new StringBuilder();
    			boolean first = true;
    			for (String name : names) {
    				if (first) {
    					first = false;
    					titles.append(name);
    				} else {
    					titles.append("," + name);
    				}
    			}
    			titles.append("\r\n");
    			DriverStation.reportError(titles.toString(), false);
    			f.format(titles.toString());
    			recordFile=true;
    		}
    	} catch (FileNotFoundException e) {
    		DriverStation.reportError(
    				"FileNotFoundException\n", true);
    	}
    }
    
	/**
	 * Stops recording and closes the file
	 */
    public void stopRecording(){
    	try {
    		if (recordFile==true){
    			f.close();
    			recordFile=false;
    		}
    	} catch (FormatterClosedException e) {
    		DriverStation.reportError(
    				"FormatterClosedException\n", true);
    	}
    }
    
    /**
     * Writes the data into assigned columns 
     */
    public void record() {
    	if (recordFile==true) {
    		boolean first = true;
    		StringBuilder line = new StringBuilder();
    		for (Supplier<Object> g : methods) {
    			if (first) {
    				first = false;
    				line.append("" + g.get().toString());
    			} else {
    				line.append("," + g.get().toString());
    			}
    		}
    		line.append("\r\n");
    		f.format(line.toString());
    	}
    }
    
    
    /**
     * Reads the number of files from another file, then increments it
     * @return the number of the file
     */
    private int incrementCount() {
    	try {
    		   Scanner sca;
    		   sca = new Scanner(new File("/home/lvuser/logs/Count.txt"));
    		   int x = sca.nextInt();
    		   Formatter count;
    		   count = new Formatter("/home/lvuser/logs/Count.txt");
    		   count.format("%d", x+1);
    		   count.close();
    		   sca.close();
    		   return x;
    	} catch (FileNotFoundException e) {
    		DriverStation.reportError("FileNotFoundExeption\n", true);
    		return -1;
    	}
       }
}