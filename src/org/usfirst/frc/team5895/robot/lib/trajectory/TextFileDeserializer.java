package org.usfirst.frc.team5895.robot.lib.trajectory;




/**
 *
 * @author Jared341
 */
public class TextFileDeserializer implements IPathDeserializer {

  public Path deserialize(String serialized) {
    String[] lines = serialized.split("\n");
    System.out.println("Parsing path string...");
    System.out.println("String has " + serialized.length() + " chars");
    
    String name = lines[0];
    int num_elements = Integer.parseInt(lines[1]);
    
    Trajectory left = new Trajectory(num_elements);
    for (int i = 0; i < num_elements; i++) {
      Trajectory.Segment segment = new Trajectory.Segment();
      String[] number = lines[i+2].split(" ");
      
      segment.pos = FastParser.parseFormattedDouble(number[0]);
      segment.vel = FastParser.parseFormattedDouble(number[1]);
      segment.acc = FastParser.parseFormattedDouble(number[2]);
      segment.jerk = FastParser.parseFormattedDouble(number[3]);
      segment.heading = FastParser.parseFormattedDouble(number[4]);
      segment.dt = FastParser.parseFormattedDouble(number[5]);
      segment.x = FastParser.parseFormattedDouble(number[6]);
      segment.y = FastParser.parseFormattedDouble(number[7]);
      
      left.setSegment(i, segment);
    }
    Trajectory right = new Trajectory(num_elements);
    for (int i = 0; i < num_elements; i++) {
      Trajectory.Segment segment = new Trajectory.Segment();
      String[] number = lines[i+2+num_elements].split(" ");
      
      segment.pos = FastParser.parseFormattedDouble(number[0]);
      segment.vel = FastParser.parseFormattedDouble(number[1]);
      segment.acc = FastParser.parseFormattedDouble(number[2]);
      segment.jerk = FastParser.parseFormattedDouble(number[3]);
      segment.heading = FastParser.parseFormattedDouble(number[4]);
      segment.dt = FastParser.parseFormattedDouble(number[5]);
      segment.x = FastParser.parseFormattedDouble(number[6]);
      segment.y = FastParser.parseFormattedDouble(number[7]);
      
      right.setSegment(i, segment);
    }
    
    System.out.println("...finished parsing path from string.");
    return new Path(name, new Trajectory.Pair(left, right));
  }
  
}
