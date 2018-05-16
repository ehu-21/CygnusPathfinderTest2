package org.usfirst.frc.team5895.robot.lib.trajectory;

import java.io.BufferedReader;
import java.io.FileInputStream;
import java.io.IOException;
import java.io.InputStreamReader;

import edu.wpi.first.wpilibj.DriverStation;

/**
 * Read a text file into a string.
 *
 * @author Jared341
 */
public class TextFileReader {

  private FileInputStream file_connection_ = null;
  private BufferedReader reader_ = null;

  public TextFileReader(String uri) {
   
	  try {
      // Open the new file
    	file_connection_ = new FileInputStream(uri);

      // Make an I/O adapter sandwich to actually get some text out
    	reader_ = new BufferedReader(new InputStreamReader(file_connection_));   
	  }
    
	  catch (IOException e) {
		  e.printStackTrace();
		  System.err.println("Could not open file connection!");
		  DriverStation.reportError("Could not open file connection!", false);
		  closeFile();
		  }
	  }

  private void closeFile() {
    try {
      // If we have a file open, close it
      if (file_connection_ != null) {
        if (reader_ != null) {
          reader_.close();
        }
        file_connection_.close();
      }
    } catch (IOException e) {
      System.err.println("Could not close file");
    }
  }

  // Returns null at end of file
  public String readLine() {
    String line = null;
    try {
      line = reader_.readLine();
    } catch (IOException e) {
      e.printStackTrace();
      closeFile();
    }
    return line;
  }
  
  public String readWholeFile() {
    StringBuffer buffer = new StringBuffer();
    String line;
    while ((line = readLine()) != null) {
      buffer.append(line);
      buffer.append("\n");
    }
    return buffer.toString();
  }
}
