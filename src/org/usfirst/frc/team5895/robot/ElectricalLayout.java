package org.usfirst.frc.team5895.robot;

public class ElectricalLayout {
    
	// PWM
	public static final int LIGHTS = 0;
	
    // CAN
    public static final int MOTOR_DRIVE_LEFT_MASTER = 1;
    public static final int MOTOR_DRIVE_LEFT_FOLLOWER_1 = 2;
    public static final int MOTOR_DRIVE_LEFT_FOLLOWER_2 = 3;
    public static final int MOTOR_DRIVE_RIGHT_MASTER = 4;
    public static final int MOTOR_DRIVE_RIGHT_FOLLOWER_1 = 5;
    public static final int MOTOR_DRIVE_RIGHT_FOLLOWER_2 = 6;
    public static final int MOTOR_ELEVATOR_MASTER = 7;
    public static final int MOTOR_ELEVATOR_FOLLOWER_1 = 8;
    public static final int MOTOR_ELEVATOR_FOLLOWER_2 = 9;
    public static final int MOTOR_CLAW_1 = 10;
    public static final int MOTOR_CLAW_2 = 11;
    
    // Solenoids
    public static final int SOLENOID_INTAKE_TENSION = 3;
    public static final int SOLENOID_INTAKE_CLAMP = 2;
    public static final int SOLENOID_INTAKE_CLAW = 1;
    public static final int SOLENOID_ELEVATOR_BRAKE = 0;
    
    // Digital Inputs
    public static final int SENSOR_ELEVATOR_TOP = 0;
    public static final int SENSOR_ELEVATOR_BOTTOM = 0;
    
    //Analog Inputs
    public static final int SENSOR_INTAKE_LEFT = 0;
    public static final int SENSOR_INTAKE_RIGHT = 1;
    public static final int SENSOR_ELEVATOR_DISTANCE_LEFT = 2;
    public static final int SENSOR_ELEVATOR_DISTANCE_RIGHT = 3;
    
    // Encoders
    public static final int ENCODER_DRIVE_LEFT_1 = 0;
    public static final int ENCODER_DRIVE_LEFT_2 = 1;
    public static final int ENCODER_DRIVE_RIGHT_1 = 2;
    public static final int ENCODER_DRIVE_RIGHT_2 = 3;
    
}