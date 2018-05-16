package org.usfirst.frc.team5895.robot.lib;

import org.usfirst.frc.team5895.robot.lib.trajectory.ChezyMath;
import org.usfirst.frc.team5895.robot.lib.trajectory.Path;
import org.usfirst.frc.team5895.robot.lib.trajectory.TextFileDeserializer;
import org.usfirst.frc.team5895.robot.lib.trajectory.TextFileReader;
import org.usfirst.frc.team5895.robot.lib.trajectory.Trajectory;
import org.usfirst.frc.team5895.robot.lib.trajectory.TrajectoryFollower;


/**
 * TrajectoryDriveController.java
 * This controller drives the robot along a specified trajectory.
 * @author Tom Bottiglieri
 */
public class TrajectoryDriveController {
	
	  Trajectory trajectory;
	  TrajectoryFollower followerLeft = new TrajectoryFollower("left");
	  TrajectoryFollower followerRight = new TrajectoryFollower("right");
	  double direction = 1.0;
	  double kTurn;

	  public TrajectoryDriveController(String file, double kp, double ki, double kd, double kv, double ka, double kTurn) {
		  
		  TextFileReader reader = new TextFileReader(file);
		  TextFileDeserializer deserializer = new TextFileDeserializer();
		  String text = reader.readWholeFile();
		  Path p = deserializer.deserialize(text);
		  
		  followerLeft.configure(kp, ki, kd, kv, ka);
		  followerRight.configure(kp, ki, kd, kv, ka);
		  
		  followerLeft.setTrajectory(p.getLeftWheelTrajectory());
		  followerRight.setTrajectory(p.getRightWheelTrajectory());
		  
		  this.kTurn = kTurn;
	  }

	  public boolean isFinished() {
		  return followerLeft.isFinishedTrajectory();
	  }

	  /**
	   * Also should reset drive encoders
       */
	  public void reset() {
		  followerLeft.reset();
		  followerRight.reset();
	  }
  
	  public int getFollowerCurrentSegment() {
		  return followerLeft.getCurrentSegment();
	  }
  
	  public int getNumSegments() {
		  return followerLeft.getNumSegments();
	  }
  
	  /**
	   * 
	   * @param leftEncDist
	   * @param rightEncDist
	   * @param angleRads
	   * @return An array with two doubles, the first is the left motor value, the second is the right
	   */
	  public double[] getOutput(double leftEncDist, double rightEncDist, double angleRads) {
		  double[] output = new double[2];

		  if (isFinished()) {
			  output[0] = 0;
			  output[1] = 0;
			  return output;
		  } else  {
			  double distanceL = direction * leftEncDist;
			  double distanceR = direction * rightEncDist;

			  double speedLeft = direction * followerLeft.calculate(distanceL);
			  double speedRight = direction * followerRight.calculate(distanceR);

			  double goalHeading = followerLeft.getHeading();
			  double observedHeading = angleRads;

			  double angleDiffRads = ChezyMath.getDifferenceInAngleRadians(observedHeading, goalHeading);
			  double angleDiff = Math.toDegrees(angleDiffRads); //why???

			  double turn = kTurn * angleDiff;

			  output[0] = speedLeft + turn;
			  output[1] = speedRight - turn;
			  return output;

	    }
	  }
  
	  public void setTrajectory(Trajectory t) {
	    this.trajectory = t;
	  }
	
	  public double getGoal() {
	    return 0;
	  }
}
