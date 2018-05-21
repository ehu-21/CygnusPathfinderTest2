package org.usfirst.frc.team5895.robot.lib.pathfinder;

import java.io.File;
import edu.wpi.first.wpilibj.DriverStation;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;

/**
 * FitMethod: HERMITE.CUBIC or HERMITE.QUINTIC
 * SAMPLE: HIGH (100 000 points), LOW (10 000 points), FAST (1 000)
 *
 */

public class PathfinderGenerator {
	private boolean saveCSV;
	private final String CSVLocation = "/home/lvuser/PathfinderFiles/"; 
	
	public PathfinderGenerator(boolean saveCSV){
		this.saveCSV = saveCSV;
	}
	 
	public PathfinderGenerator() {
		this(false);
	}
	
	public void makeCSV(Trajectory trajectory, String name) {
		if(saveCSV) {
			File myFile = new File(CSVLocation + name + ".csv");
			Pathfinder.writeToCSV(myFile, trajectory);
		}
	}
	
	public double d2r( double degree) {
		return Pathfinder.d2r(degree);
	}
	
	public Trajectory Straight() {
		String name = "Straight";
		double dt = 0.01; // in second
		double max_vel = 8.0; // in f/s
		double max_acc = 6.0; // in f/s/s
		double max_jer = 50.0; // in f/s/s/s
		/**
		 * x, y in feet
		 * angle in radian if not using d2r
		 * d2r will change angle in degree to radian
		 * positive number means (forward, left, left) 
		 */
		Waypoint[] points = new Waypoint[] { 
    		    new Waypoint(0, 0, d2r(0)),     
    		    new Waypoint(10, 0, d2r(0)),                        
		};

		Trajectory.Config config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_QUINTIC, Trajectory.Config.SAMPLES_HIGH, dt, max_vel, max_acc, max_jer);
		Trajectory trajectory = Pathfinder.generate(points, config);
		
		//DriverStation.reportError("Finish making straight", false);
		//create CSV file
    	makeCSV(trajectory, name);
    	
    	return trajectory;		
	}

/*
 * RIGHT SIDE PATHFINDER PATHS
 */
	
	public Trajectory RightRightSwitchFront() {
		String name = "RightRightSwitchFront";
		double dt = 0.01; // in second
		double max_vel = 6.0; // in f/s
		double max_acc = 6.0; // in f/s/s
		double max_jer = 50.0; // in f/s/s/s
		/**
		 * x, y in feet
		 * angle in radian if not using d2r
		 * d2r will change angle in degree to radian
		 * positive number means (forward, left, left) 
		 */
		Waypoint[] points = new Waypoint[] { 
    		    new Waypoint(0, 4, d2r(0)),     
    		    new Waypoint(4, 4, d2r(0)),       
    		    new Waypoint(9, 9, d2r(30.0)),
    		    new Waypoint(11.5, 9, d2r(0.0)),
		};

		Trajectory.Config config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC, Trajectory.Config.SAMPLES_FAST, dt, max_vel, max_acc, max_jer);
		Trajectory trajectory = Pathfinder.generate(points, config);
		
		//DriverStation.reportError("Finish making straight", false);
		//create CSV file
    	makeCSV(trajectory, name);
    	
    	return trajectory;		
	}
	
	public Trajectory RightLeftScale() {
		String name = "RightLeftScale";
		double dt = 0.01; // in second
		double max_vel = 6.0; // in f/s
		double max_acc = 6.0; // in f/s/s
		double max_jer = 50.0; // in f/s/s/s
		/**
		 * x, y in feet
		 * angle in radian if not using d2r
		 * d2r will change angle in degree to radian
		 * positive number means (forward, left, left) 
		 */
		Waypoint[] points = new Waypoint[] { 
    		    new Waypoint(0, 0, d2r(0)),     
    		    new Waypoint(10, 0, d2r(0)), 
    		    new Waypoint(16, 10, d2r(89.0)),     
    		    new Waypoint(16.0, 14, d2r(89.0)),
    		    new Waypoint(18.0, 16.0, d2r(0)),     
    		    new Waypoint(21.5, 15.5, d2r(-30.0)),
		};

		Trajectory.Config config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_QUINTIC, Trajectory.Config.SAMPLES_HIGH, dt, max_vel, max_acc, max_jer);
		Trajectory trajectory = Pathfinder.generate(points, config);
    	
		//create CSV file
    	makeCSV(trajectory, name);
    	File myFile = new File("/home/lvuser/PathfinderFiles/Straight.csv");
		Pathfinder.writeToCSV(myFile, trajectory);
    	//DriverStation.reportError("Finish Trajectory: "+ name, false);
    	
    	return trajectory;		
	}
	
	public Trajectory RightRightSwitchSide() {
		String name = "RightRightSwitchSide";
		double dt = 0.01; // in second
		double max_vel = 6.0; // in f/s
		double max_acc = 6.0; // in f/s/s
		double max_jer = 50.0; // in f/s/s/s
		/**
		 * x, y in feet
		 * angle in radian if not using d2r
		 * d2r will change angle in degree to radian
		 * positive number means (forward, left, left) 
		 */
		Waypoint[] points = new Waypoint[] { 
    		    new Waypoint(0, 4, d2r(0)),     
    		    new Waypoint(10, 3, d2r(0)), 
    		    new Waypoint(14, 6.5, d2r(89.0)),     
		};

		Trajectory.Config config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_QUINTIC, Trajectory.Config.SAMPLES_HIGH, dt, max_vel, max_acc, max_jer);
		Trajectory trajectory = Pathfinder.generate(points, config);
    	
		//create CSV file
    	makeCSV(trajectory, name);
    	File myFile = new File("/home/lvuser/PathfinderFiles/Straight.csv");
		Pathfinder.writeToCSV(myFile, trajectory);
    	//DriverStation.reportError("Finish Trajectory: "+ name, false);
    	
    	return trajectory;		
	}
	
	public Trajectory RightRightScale() {
		String name = "RightRightScale";
		double dt = 0.01; // in second
		double max_vel = 6.0; // in f/s
		double max_acc = 6.0; // in f/s/s
		double max_jer = 50.0; // in f/s/s/s
		/**
		 * x, y in feet
		 * angle in radian if not using d2r
		 * d2r will change angle in degree to radian
		 * positive number means (forward, left, left) 
		 */
		Waypoint[] points = new Waypoint[] { 
    		    new Waypoint(0, 4, d2r(0)),     
    		    new Waypoint(24, 2, d2r(0)), 
    		    new Waypoint(27, 5, d2r(89.0)),     
		};

		Trajectory.Config config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_QUINTIC, Trajectory.Config.SAMPLES_HIGH, dt, max_vel, max_acc, max_jer);
		Trajectory trajectory = Pathfinder.generate(points, config);
    	
		//create CSV file
    	makeCSV(trajectory, name);
    	//DriverStation.reportError("Finish Trajectory: "+ name, false);
    	
    	return trajectory;		
	}
	
	public Trajectory RightLeftSwitchBack() {
		String name = "RightLeftSwitchBack";
		double dt = 0.01; // in second
		double max_vel = 6.0; // in f/s
		double max_acc = 6.0; // in f/s/s
		double max_jer = 50.0; // in f/s/s/s
		/**
		 * x, y in feet
		 * angle in radian if not using d2r
		 * d2r will change angle in degree to radian
		 * positive number means (forward, left, left) 
		 */
		Waypoint[] points = new Waypoint[] { 
    		    new Waypoint(0, 4, d2r(0)),     
    		    new Waypoint(15, 4, d2r(0)), 
    		    new Waypoint(19, 10, d2r(89.99)),     
    		    new Waypoint(19, 15, d2r(89.99)),
    		    new Waypoint(17, 18, d2r(180)),
		};

		Trajectory.Config config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_QUINTIC, Trajectory.Config.SAMPLES_HIGH, dt, max_vel, max_acc, max_jer);
		Trajectory trajectory = Pathfinder.generate(points, config);
    	
		//create CSV file
    	makeCSV(trajectory, name);
    	//DriverStation.reportError("Finish Trajectory: "+ name, false);
    	
    	return trajectory;		
	}
	
	public Trajectory RightLeftSwitchFront() {
		String name = "RightLeftSwitchFront";
		double dt = 0.01; // in second
		double max_vel = 6.0; // in f/s
		double max_acc = 6.0; // in f/s/s
		double max_jer = 50.0; // in f/s/s/s
		/**
		 * x, y in feet
		 * angle in radian if not using d2r
		 * d2r will change angle in degree to radian
		 * positive number means (forward, left, left) 
		 */
		Waypoint[] points = new Waypoint[] { 
    		    new Waypoint(0, 4, d2r(0)),     
    		    new Waypoint(5, 13, d2r(89.99)), 
    		    new Waypoint(11, 18, d2r(0)),     
		};

		Trajectory.Config config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_QUINTIC, Trajectory.Config.SAMPLES_HIGH, dt, max_vel, max_acc, max_jer);
		Trajectory trajectory = Pathfinder.generate(points, config);
    	
		//create CSV file
    	makeCSV(trajectory, name);
    	//DriverStation.reportError("Finish Trajectory: "+ name, false);
    	
    	return trajectory;		
	}

	//side of the switch to the scale
	public Trajectory RightSwitchBlock() {
		String name = "RightSwitchBlock";
		double dt = 0.01; // in second
		double max_vel = 6.0; // in f/s
		double max_acc = 6.0; // in f/s/s
		double max_jer = 50.0; // in f/s/s/s
		/**
		 * x, y in feet
		 * angle in radian if not using d2r
		 * d2r will change angle in degree to radian
		 * positive number means (forward, left, left) 
		 */
		Waypoint[] points = new Waypoint[] { 
    		    new Waypoint(14, 6, d2r(89.99)),     
    		    new Waypoint(19, 2, d2r(0)), 
    		    new Waypoint(23, 2, d2r(0)),
    		    new Waypoint(27, 5, d2r(89.99)),
		};

		Trajectory.Config config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_QUINTIC, Trajectory.Config.SAMPLES_HIGH, dt, max_vel, max_acc, max_jer);
		Trajectory trajectory = Pathfinder.generate(points, config);
    	
		//create CSV file
    	makeCSV(trajectory, name);
    	//DriverStation.reportError("Finish Trajectory: "+ name, false);
    	
    	return trajectory;		
	}

	public Trajectory RightScaleLeftSwitch() {
		String name = "RightScaleLeftSwitch";
		double dt = 0.01; // in second
		double max_vel = 6.0; // in f/s
		double max_acc = 6.0; // in f/s/s
		double max_jer = 50.0; // in f/s/s/s
		/**
		 * x, y in feet
		 * angle in radian if not using d2r
		 * d2r will change angle in degree to radian
		 * positive number means (forward, left, left) 
		 */
		Waypoint[] points = new Waypoint[] { 
    		    new Waypoint(27, 5, d2r(89.99)),     
    		    new Waypoint(22.5, 2, d2r(180)), 
    		    new Waypoint(19.5, 9, d2r(89.99)),
    		    new Waypoint(20, 15.5, d2r(89.99)),
    		    new Waypoint(18, 18, d2r(180)),
		};

		Trajectory.Config config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_QUINTIC, Trajectory.Config.SAMPLES_HIGH, dt, max_vel, max_acc, max_jer);
		Trajectory trajectory = Pathfinder.generate(points, config);
    	
		//create CSV file
    	makeCSV(trajectory, name);
    	//DriverStation.reportError("Finish Trajectory: "+ name, false);
    	
    	return trajectory;		
	}

	
/*
 * CENTER PATHFINDER PATHS
 */
	
	public Trajectory CenterRightSwitchFront() {
		String name = "CenterRightSwitchFront";
		double dt = 0.01; // in second
		double max_vel = 6.0; // in f/s
		double max_acc = 6.0; // in f/s/s
		double max_jer = 50.0; // in f/s/s/s
		/**
		 * x, y in feet
		 * angle in radian if not using d2r
		 * d2r will change angle in degree to radian
		 * positive number means (forward, left, left) 
		 */
		Waypoint[] points = new Waypoint[] { 
    		    new Waypoint(0, 0, d2r(0)),     
    		    new Waypoint(9, -4, d2r(0)),   
		};

		Trajectory.Config config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_QUINTIC, Trajectory.Config.SAMPLES_HIGH, dt, max_vel, max_acc, max_jer);
		Trajectory trajectory = Pathfinder.generate(points, config);
    	
		//create CSV file
    	makeCSV(trajectory, name);
    	//DriverStation.reportError("Finish Trajectory: "+ name, false);
    	
    	return trajectory;		
	}
	
	public Trajectory CenterLeftScale() {
		String name = "CenterLeftScale";
		double dt = 0.01; // in second
		double max_vel = 6.0; // in f/s
		double max_acc = 6.0; // in f/s/s
		double max_jer = 50.0; // in f/s/s/s
		/**
		 * x, y in feet
		 * angle in radian if not using d2r
		 * d2r will change angle in degree to radian
		 * positive number means (forward, left, left) 
		 */
		Waypoint[] points = new Waypoint[] { 
    		    new Waypoint(0, 13, d2r(0)),     
    		    new Waypoint(5, 19, d2r(89.99)),   
    		    new Waypoint(9, 24, d2r(0)),
    		    new Waypoint(21, 25, d2r(0)),
    		    new Waypoint(27, 22, d2r(89.99)),
		};

		Trajectory.Config config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_QUINTIC, Trajectory.Config.SAMPLES_HIGH, dt, max_vel, max_acc, max_jer);
		Trajectory trajectory = Pathfinder.generate(points, config);
    	
		//create CSV file
    	makeCSV(trajectory, name);
    	//DriverStation.reportError("Finish Trajectory: "+ name, false);
    	
    	return trajectory;		
	}
	
	public Trajectory CenterRightScale() {
		String name = "CenterRightScale";
		double dt = 0.01; // in second
		double max_vel = 6.0; // in f/s
		double max_acc = 6.0; // in f/s/s
		double max_jer = 50.0; // in f/s/s/s
		/**
		 * x, y in feet
		 * angle in radian if not using d2r
		 * d2r will change angle in degree to radian
		 * positive number means (forward, left, left) 
		 */
		Waypoint[] points = new Waypoint[] { 
    		    new Waypoint(0, 13, d2r(0)),     
    		    new Waypoint(10, 4, d2r(0)),   
    		    new Waypoint(22, 2, d2r(0)),
    		    new Waypoint(27, 5, d2r(89.99)),
		};

		Trajectory.Config config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_QUINTIC, Trajectory.Config.SAMPLES_HIGH, dt, max_vel, max_acc, max_jer);
		Trajectory trajectory = Pathfinder.generate(points, config);
    	
		//create CSV file
    	makeCSV(trajectory, name);
    	//DriverStation.reportError("Finish Trajectory: "+ name, false);
    	
    	return trajectory;		
	}
	
	public Trajectory CenterLeftSwitchFront() {
		String name = "CenterLeftSwitchFront";
		double dt = 0.01; // in second
		double max_vel = 6.0; // in f/s
		double max_acc = 6.0; // in f/s/s
		double max_jer = 50.0; // in f/s/s/s
		/**
		 * x, y in feet
		 * angle in radian if not using d2r
		 * d2r will change angle in degree to radian
		 * positive number means (forward, left, left) 
		 */
		Waypoint[] points = new Waypoint[] { 
    		    new Waypoint(0, 13, d2r(0)),     
    		    new Waypoint(11, 18, d2r(0)),   
		};

		Trajectory.Config config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_QUINTIC, Trajectory.Config.SAMPLES_HIGH, dt, max_vel, max_acc, max_jer);
		Trajectory trajectory = Pathfinder.generate(points, config);
    	
		//create CSV file
    	makeCSV(trajectory, name);
    	//DriverStation.reportError("Finish Trajectory: "+ name, false);
    	
    	return trajectory;		
	}

/*
 * LEFT PATHFINDER PATHS
 */
	
	public Trajectory LeftLeftScale() {
		String name = "LeftLeftScale";
		double dt = 0.01; // in second
		double max_vel = 6.0; // in f/s
		double max_acc = 6.0; // in f/s/s
		double max_jer = 50.0; // in f/s/s/s
		/**
		 * x, y in feet
		 * angle in radian if not using d2r
		 * d2r will change angle in degree to radian
		 * positive number means (forward, left, left) 
		 */
		Waypoint[] points = new Waypoint[] { 
				new Waypoint(0, 23, d2r(0)),
				new Waypoint(22, 25, d2r(0)),
				new Waypoint(27, 22, d2r(89)),
		};

		Trajectory.Config config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_QUINTIC, Trajectory.Config.SAMPLES_HIGH, dt, max_vel, max_acc, max_jer);
		Trajectory trajectory = Pathfinder.generate(points, config);
    	
		//create CSV file
    	makeCSV(trajectory, name);
    	//DriverStation.reportError("Finish Trajectory: "+ name, false);
    	
    	return trajectory;		
	}
	
	public Trajectory LeftLeftSwitchFront() {
		String name = "LeftLeftSwitchFront";
		double dt = 0.01; // in second
		double max_vel = 6.0; // in f/s
		double max_acc = 6.0; // in f/s/s
		double max_jer = 50.0; // in f/s/s/s
		/**
		 * x, y in feet
		 * angle in radian if not using d2r
		 * d2r will change angle in degree to radian
		 * positive number means (forward, left, left) 
		 */
		Waypoint[] points = new Waypoint[] { 
				new Waypoint(0, 23, d2r(0)),
				new Waypoint(11, 18, d2r(0)),
		};

		Trajectory.Config config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_QUINTIC, Trajectory.Config.SAMPLES_HIGH, dt, max_vel, max_acc, max_jer);
		Trajectory trajectory = Pathfinder.generate(points, config);
    	
		//create CSV file
    	makeCSV(trajectory, name);
    	//DriverStation.reportError("Finish Trajectory: "+ name, false);
    	
    	return trajectory;		
	}
	
	public Trajectory LeftRightScale() {
		String name = "LeftRightScale";
		double dt = 0.01; // in second
		double max_vel = 6.0; // in f/s
		double max_acc = 6.0; // in f/s/s
		double max_jer = 50.0; // in f/s/s/s
		/**
		 * x, y in feet
		 * angle in radian if not using d2r
		 * d2r will change angle in degree to radian
		 * positive number means (forward, left, left) 
		 */
		Waypoint[] points = new Waypoint[] { 
				new Waypoint(0, 23, d2r(0)),
				new Waypoint(14, 23, d2r(0)),
				new Waypoint(19, 18, d2r(89)),
				new Waypoint(19, 10, d2r(89)),
				new Waypoint(24, 7, d2r(0)),
		};

		Trajectory.Config config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_QUINTIC, Trajectory.Config.SAMPLES_HIGH, dt, max_vel, max_acc, max_jer);
		Trajectory trajectory = Pathfinder.generate(points, config);
    	
		//create CSV file
    	makeCSV(trajectory, name);
    	//DriverStation.reportError("Finish Trajectory: "+ name, false);
    	
    	return trajectory;		
	} 
	
	public Trajectory LeftRightSwitchBack() {
		String name = "LeftRightSwitchBack";
		double dt = 0.01; // in second
		double max_vel = 6.0; // in f/s
		double max_acc = 6.0; // in f/s/s
		double max_jer = 50.0; // in f/s/s/s
		/**
		 * x, y in feet
		 * angle in radian if not using d2r
		 * d2r will change angle in degree to radian
		 * positive number means (forward, left, left) 
		 */
		Waypoint[] points = new Waypoint[] { 
				new Waypoint(0, 23, d2r(0)),
				new Waypoint(14, 23, d2r(0)),
				new Waypoint(20, 18, d2r(89)),
				new Waypoint(20, 12, d2r(89)),
				new Waypoint(17, 9, d2r(0)),
		};

		Trajectory.Config config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_QUINTIC, Trajectory.Config.SAMPLES_HIGH, dt, max_vel, max_acc, max_jer);
		Trajectory trajectory = Pathfinder.generate(points, config);
    	
		//create CSV file
    	makeCSV(trajectory, name);
    	//DriverStation.reportError("Finish Trajectory: "+ name, false);
    	
    	return trajectory;		
	} 
	
	public Trajectory LeftRightSwitchFront() {
		String name = "LeftRightSwitchFront";
		double dt = 0.01; // in second
		double max_vel = 6.0; // in f/s
		double max_acc = 6.0; // in f/s/s
		double max_jer = 50.0; // in f/s/s/s
		/**
		 * x, y in feet
		 * angle in radian if not using d2r
		 * d2r will change angle in degree to radian
		 * positive number means (forward, left, left) 
		 */
		Waypoint[] points = new Waypoint[] { 
				new Waypoint(0, 23, d2r(0)),
				new Waypoint(11, 8.5, d2r(0)),
		};

		Trajectory.Config config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_QUINTIC, Trajectory.Config.SAMPLES_HIGH, dt, max_vel, max_acc, max_jer);
		Trajectory trajectory = Pathfinder.generate(points, config);
    	
		//create CSV file
    	makeCSV(trajectory, name);
    	//DriverStation.reportError("Finish Trajectory: "+ name, false);
    	
    	return trajectory;		
	} 
	
	public Trajectory LeftLeftSwitchSide() {
		String name = "LeftLeftSwitchSide";
		double dt = 0.01; // in second
		double max_vel = 6.0; // in f/s
		double max_acc = 6.0; // in f/s/s
		double max_jer = 50.0; // in f/s/s/s
		/**
		 * x, y in feet
		 * angle in radian if not using d2r
		 * d2r will change angle in degree to radian
		 * positive number means (forward, left, left) 
		 */
		Waypoint[] points = new Waypoint[] { 
				new Waypoint(0, 23, d2r(0)),
				new Waypoint(10, 24, d2r(0)),
				new Waypoint(14, 20.5, d2r(89.99)),
		};

		Trajectory.Config config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_QUINTIC, Trajectory.Config.SAMPLES_HIGH, dt, max_vel, max_acc, max_jer);
		Trajectory trajectory = Pathfinder.generate(points, config);
    	
		//create CSV file
    	makeCSV(trajectory, name);
    	//DriverStation.reportError("Finish Trajectory: "+ name, false);
    	
    	return trajectory;		
	} 
	
	public Trajectory LeftScaleRightSwitch() {
		String name = "LeftScaleRightSwitch";
		double dt = 0.01; // in second
		double max_vel = 6.0; // in f/s
		double max_acc = 6.0; // in f/s/s
		double max_jer = 50.0; // in f/s/s/s
		/**
		 * x, y in feet
		 * angle in radian if not using d2r
		 * d2r will change angle in degree to radian
		 * positive number means (forward, left, left) 
		 */
		Waypoint[] points = new Waypoint[] { 
				new Waypoint(27, 22, d2r(89.99)),
				new Waypoint(22, 25, d2r(180)),
				new Waypoint(19, 20, d2r(89.99)),
				new Waypoint(20, 12, d2r(89.99)),
				new Waypoint(18, 9, d2r(180)),
		};

		Trajectory.Config config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_QUINTIC, Trajectory.Config.SAMPLES_HIGH, dt, max_vel, max_acc, max_jer);
		Trajectory trajectory = Pathfinder.generate(points, config);
    	
		//create CSV file
    	makeCSV(trajectory, name);
    	//DriverStation.reportError("Finish Trajectory: "+ name, false);
    	
    	return trajectory;		
	} 

	
	//side of switch to scale
	public Trajectory LeftSwitchBlock() {
		String name = "LeftSwitchBlock";
		double dt = 0.01; // in second
		double max_vel = 6.0; // in f/s
		double max_acc = 6.0; // in f/s/s
		double max_jer = 50.0; // in f/s/s/s
		/**
		 * x, y in feet
		 * angle in radian if not using d2r
		 * d2r will change angle in degree to radian
		 * positive number means (forward, left, left) 
		 */
		Waypoint[] points = new Waypoint[] { 
				new Waypoint(14, 21, d2r(89.99)),
				new Waypoint(18, 25, d2r(0)),
				new Waypoint(22, 25, d2r(0)),
				new Waypoint(27, 22, d2r(89.99)),
		};

		Trajectory.Config config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_QUINTIC, Trajectory.Config.SAMPLES_HIGH, dt, max_vel, max_acc, max_jer);
		Trajectory trajectory = Pathfinder.generate(points, config);
    	
		//create CSV file
    	makeCSV(trajectory, name);
    	//DriverStation.reportError("Finish Trajectory: "+ name, false);
    	
    	return trajectory;		
	} 

/*
 * OTHER PATHFINDER PATHS
 */
	public Trajectory RightSecondScaleCube() {
		String name = "RightSecondScaleCube";
		double dt = 0.01; // in second
		double max_vel = 6.0; // in f/s
		double max_acc = 6.0; // in f/s/s
		double max_jer = 50.0; // in f/s/s/s
		/**
		 * x, y in feet
		 * angle in radian if not using d2r
		 * d2r will change angle in degree to radian
		 * positive number means (forward, left, left) 
		 */
		Waypoint[] points = new Waypoint[] { 
				new Waypoint(24, 7, d2r(180)),
				new Waypoint(17, 7.5, d2r(180)),
		};

		Trajectory.Config config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_QUINTIC, Trajectory.Config.SAMPLES_HIGH, dt, max_vel, max_acc, max_jer);
		Trajectory trajectory = Pathfinder.generate(points, config);
    	
		//create CSV file
    	makeCSV(trajectory, name);
    	//DriverStation.reportError("Finish Trajectory: "+ name, false);
    	
    	return trajectory;		
	} 

	public Trajectory LeftSecondScaleCube() {
		String name = "LeftSecondScaleCube";
		double dt = 0.01; // in second
		double max_vel = 6.0; // in f/s
		double max_acc = 6.0; // in f/s/s
		double max_jer = 50.0; // in f/s/s/s
		/**
		 * x, y in feet
		 * angle in radian if not using d2r
		 * d2r will change angle in degree to radian
		 * positive number means (forward, left, left) 
		 */
		Waypoint[] points = new Waypoint[] { 
				new Waypoint(24, 20, d2r(180)),
				new Waypoint(17, 19.5, d2r(180)),
		};

		Trajectory.Config config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_QUINTIC, Trajectory.Config.SAMPLES_HIGH, dt, max_vel, max_acc, max_jer);
		Trajectory trajectory = Pathfinder.generate(points, config);
    	
		//create CSV file
    	makeCSV(trajectory, name);
    	//DriverStation.reportError("Finish Trajectory: "+ name, false);
    	
    	return trajectory;		
	} 
	
	public Trajectory RightLeftDrive() {
		String name = "RightLeftDrive";
		double dt = 0.01; // in second
		double max_vel = 6.0; // in f/s
		double max_acc = 6.0; // in f/s/s
		double max_jer = 50.0; // in f/s/s/s
		/**
		 * x, y in feet
		 * angle in radian if not using d2r
		 * d2r will change angle in degree to radian
		 * positive number means (forward, left, left) 
		 */
		Waypoint[] points = new Waypoint[] { 
				new Waypoint(0, 4, d2r(0)),
				new Waypoint(14, 4, d2r(0)),
				new Waypoint(19.5, 12, d2r(89.99)),
		};

		Trajectory.Config config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_QUINTIC, Trajectory.Config.SAMPLES_HIGH, dt, max_vel, max_acc, max_jer);
		Trajectory trajectory = Pathfinder.generate(points, config);
    	
		//create CSV file
    	makeCSV(trajectory, name);
    	//DriverStation.reportError("Finish Trajectory: "+ name, false);
    	
    	return trajectory;		
	} 

	public Trajectory RightScaleCurve() {
		String name = "RightScaleCurve";
		double dt = 0.01; // in second
		double max_vel = 6.0; // in f/s
		double max_acc = 6.0; // in f/s/s
		double max_jer = 50.0; // in f/s/s/sv
		/**
		 * x, y in feet
		 * angle in radian if not using d2r
		 * d2r will change angle in degree to radian
		 * positive number means (forward, left, left) 
		 */
		Waypoint[] points = new Waypoint[] { 
				new Waypoint(0, 23, d2r(0)),
				new Waypoint(14, 23, d2r(0)),
				new Waypoint(19.5, 17, d2r(89.99)),
		};

		Trajectory.Config config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_QUINTIC, Trajectory.Config.SAMPLES_HIGH, dt, max_vel, max_acc, max_jer);
		Trajectory trajectory = Pathfinder.generate(points, config);
    	
		//create CSV file
    	makeCSV(trajectory, name);
    	//DriverStation.reportError("Finish Trajectory: "+ name, false);
    	
    	return trajectory;		
	} 

	public Trajectory LeftRightDrive() {
		String name = "LeftRightDrive";
		double dt = 0.01; // in second
		double max_vel = 6.0; // in f/s
		double max_acc = 6.0; // in f/s/s
		double max_jer = 50.0; // in f/s/s/s
		/**
		 * x, y in feet
		 * angle in radian if not using d2r
		 * d2r will change angle in degree to radian
		 * positive number means (forward, left, left) 
		 */
		Waypoint[] points = new Waypoint[] { 
				 new Waypoint(0, 0, d2r(-160)),
			     new Waypoint(-6 * Math.cos(Math.toRadians(20)), -6 * Math.sin(Math.toRadians(20)), d2r(-160)),
		};

		Trajectory.Config config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_QUINTIC, Trajectory.Config.SAMPLES_HIGH, dt, max_vel, max_acc, max_jer);
		Trajectory trajectory = Pathfinder.generate(points, config);
    	
		//create CSV file
    	makeCSV(trajectory, name);
    	//DriverStation.reportError("Finish Trajectory: "+ name, false);
    	
    	return trajectory;		
	} 
	
/* the graphs after this comment haven't been graphed
 * on the Motion Profile Generator
 * 
 * they're just inverses of the spline waypoints and 
 * may be off
 */
	public Trajectory RightScaleCubeBack() {
	    String name = "RightScaleCubeBack";
		double dt = 0.01; // in second
		double max_vel = 6.0; // in f/s
		double max_acc = 6.0; // in f/s/s
		double max_jer = 50.0; // in f/s/s/s
	        /**
	         * x, y in feet
	         * angle in radian if not using d2r
	         * d2r will change angle in degree to radian
	         * positive number means (forward, left, left) 
	         */
	        Waypoint[] points = new Waypoint[] { 
	                new Waypoint(0, 0, d2r(-160)),
	                new Waypoint(-3 * Math.cos(Math.toRadians(20)), -3 * Math.sin(Math.toRadians(20)), d2r(-160)),
	        };
	        Trajectory.Config config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_QUINTIC, Trajectory.Config.SAMPLES_HIGH, dt, max_vel, max_acc, max_jer);
	        Trajectory trajectory = Pathfinder.generate(points, config);
	        
	        //create CSV file
	        makeCSV(trajectory, name);
	        //DriverStation.reportError("Finish Trajectory: "+ name, false);
	         
	        return trajectory;      
	    }
	    
	public Trajectory SCurve() {
	        String name = "SCurve";
			double dt = 0.01; // in second
			double max_vel = 6.0; // in f/s
			double max_acc = 6.0; // in f/s/s
			double max_jer = 50.0; // in f/s/s/s
	        /**
	         * x, y in feet
	         * angle in radian if not using d2r
	         * d2r will change angle in degree to radian
	         * positive number means (forward, left, left) 
	         */
	        Waypoint[] points = new Waypoint[] { 
	                new Waypoint(0, 0, d2r(0)),
	                new Waypoint(10, 3, d2r(0)),
	        };
	        Trajectory.Config config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_QUINTIC, Trajectory.Config.SAMPLES_HIGH, dt, max_vel, max_acc, max_jer);
	        Trajectory trajectory = Pathfinder.generate(points, config);
	        
	        //create CSV file
	        makeCSV(trajectory, name);
	        //DriverStation.reportError("Finish Trajectory: "+ name, false);
	         
	        return trajectory;      
	    }
	    
	public Trajectory CenterRightSwitchCube() {
	        String name = "CenterRightSwitchCube";
			double dt = 0.01; // in second
			double max_vel = 6.0; // in f/s
			double max_acc = 6.0; // in f/s/s
			double max_jer = 50.0; // in f/s/s/s
	        /**
	         * x, y in feet
	         * angle in radian if not using d2r
	         * d2r will change angle in degree to radian
	         * positive number means (forward, left, left) 
	         */
	        Waypoint[] points = new Waypoint[] { 
	                new Waypoint(0, 0, d2r(-60)),
	                new Waypoint(2.0, -2.0 * Math.sqrt(3.0), d2r(-60)),
	        };
	        Trajectory.Config config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_QUINTIC, Trajectory.Config.SAMPLES_HIGH, dt, max_vel, max_acc, max_jer);
	        Trajectory trajectory = Pathfinder.generate(points, config);
	        
	        //create CSV file
	        makeCSV(trajectory, name);
	        //DriverStation.reportError("Finish Trajectory: "+ name, false);
	         
	        return trajectory;      
	    }
	    
	public Trajectory CenterRightSwitchRev() {
	        String name = "CenterRightSwitchRev";
			double dt = 0.01; // in second
			double max_vel = 6.0; // in f/s
			double max_acc = 6.0; // in f/s/s
			double max_jer = 50.0; // in f/s/s/s
	        /**
	         * x, y in feet
	         * angle in radian if not using d2r
	         * d2r will change angle in degree to radian
	         * positive number means (forward, left, left) 
	         */
	        Waypoint[] points = new Waypoint[] { 
	                new Waypoint(0, 0, d2r(-60)),
	                new Waypoint(1.5, -1.5 * Math.sqrt(3.0), d2r(-60)),
	        };
	        Trajectory.Config config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_QUINTIC, Trajectory.Config.SAMPLES_HIGH, dt, max_vel, max_acc, max_jer);
	        Trajectory trajectory = Pathfinder.generate(points, config);
	        
	        //create CSV file
	        makeCSV(trajectory, name);
	        //DriverStation.reportError("Finish Trajectory: "+ name, false);
	         
	        return trajectory;      
	    }
	    
	public Trajectory RightScaleBackwards() {
	        String name = "RightScaleBackwards";
			double dt = 0.01; // in second
			double max_vel = 6.0; // in f/s
			double max_acc = 6.0; // in f/s/s
			double max_jer = 50.0; // in f/s/s/s
	        /**
	         * x, y in feet
	         * angle in radian if not using d2r
	         * d2r will change angle in degree to radian
	         * positive number means (forward, left, left) 
	         */
	        Waypoint[] points = new Waypoint[] { 
	                new Waypoint(0, 0, d2r(-30)),
	                new Waypoint(-1.0*Math.sqrt(3), 1.0, d2r(-30)),
	        };
	        Trajectory.Config config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_QUINTIC, Trajectory.Config.SAMPLES_HIGH, dt, max_vel, max_acc, max_jer);
	        Trajectory trajectory = Pathfinder.generate(points, config);
	        
	        //create CSV file
	        makeCSV(trajectory, name);
	        //DriverStation.reportError("Finish Trajectory: "+ name, false);
	         
	        return trajectory;      
	    }
}
