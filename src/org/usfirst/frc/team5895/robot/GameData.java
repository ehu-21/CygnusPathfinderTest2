package org.usfirst.frc.team5895.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class GameData {

	private String gameData; 
	private String side;
	private String doScale;
	private String doSwitch;
	private String autoRoutine;
	
	public GameData() {
	} 
		
	
	public String getGameData() { 
		return DriverStation.getInstance().getGameSpecificMessage();
	}
	
	/**
	 * gets the auto routine to run from the game data and dashboard inputs
	 * @return the auto routine to run 
	 */
	public String getAutoRoutine() {

		//gets the side that we start on from the dashboard
		side = SmartDashboard.getString("DB/String 0", "L");
		doSwitch = SmartDashboard.getString("DB/String 1", "Y");
		doScale = SmartDashboard.getString("DB/String 2", "Y");
		
		//gets the game data from the field
		gameData = getGameData();
		
		autoRoutine = "";
		
		autoRoutine = autoRoutine + side;
		if(doSwitch.toUpperCase().contains("Y")) {
			autoRoutine = autoRoutine + gameData.charAt(0);
		} else {
			autoRoutine = autoRoutine + "0";
		}
		if(doScale.toUpperCase().contains("Y")) {
			autoRoutine = autoRoutine + gameData.charAt(1);
		} else {
			autoRoutine = autoRoutine + "0";
		}
		DriverStation.reportError("" + autoRoutine, false);
		return autoRoutine;
	}
	
	/**
	 * returns the labeled dashboard inputs for logging
	 * @return the placement of the robot on the field (side)
	 * @return whether we want to do switch (do switch)
	 * @return whether we want to do scale (do scale)
	 */
	public String getDashboardInput() {
		return "Side: " + side + " Do Switch: " + doSwitch + " Do Scale: " + doScale;
	}
}