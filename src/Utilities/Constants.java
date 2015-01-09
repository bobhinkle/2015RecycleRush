/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package Utilities;

/**
 *
 * @author xpsl05x
 */
public class Constants {
    
    public static final boolean LOW_GEAR  = true; // Drivetrain low gear
    public static final double MIN_DT_POWER = 0.2;
    public static final int autonSelect = 2;
    //0 = 2 Ball - Turn with Kinect
    //1 = 1 Ball from CVR
    //2 = Kinect with Blocker
    //3 = 2 Ball - Straight
    
    public static final double TURN_KP = 0.0175; //0.020
    public static final double TURN_KI = 0.000005;
    public static final double TURN_KD = 0.00;//0.02
    public static final double TURN_KFV = 0.00;
    public static final double TURN_KFA = 0.00;
    public static final double TURN_ON_TARGET_DEG = 1;
    public static final double TURN_MAX_ACCEL = 90.0;
    public static final double TURN_MAX_VEL = 90.0;
    
    public static final double DIST_KP = 0.04;
    public static final double DIST_KI = 0.0002; 
    public static final double DIST_KD = 0.000; 
    
    public static final double DIST_KP_BIG = 0.018;
    public static final double DIST_KI_BIG = 0.00001; 
    public static final double DIST_KD_BIG = 0.5; 
    
    public static final double DIST_SMALL = 10;
    
    public static final double DIST_KP_2BALL = 0.07;
    public static final double DIST_KI_2BALL = 0.00; 
    public static final double DIST_KD_2BALL = 0.045;
    public static final double STRAIGHT_KP = 0.009;//.012
    public static final double STRAIGHT_KI = 0.0;
    public static final double STRAIGHT_KD = 0.0;
    public static final double DISTANCE_TOLERANCE = 1.0;
    
    public static final int DRIVETRAIN_VERTICAL_POS = 0;
    public static final int DRIVETRAIN_HORIZONTAL_POS = 90;
    
    public static final double ELEVATOR_MAX_HEIGHT  = 12.5;   // MAXIMUM ELEVATOR HEIGHT 10
    public static final double ELEVATOR_MIN_HEIGHT  = -10;
    public static final double ELEVATOR_P = 0.295;
    public static final double ELEVATOR_I = 0.000;
    public static final double ELEVATOR_D = 0.000;
    public static final double ELEVATOR_DOWN_P = 0.025; //0.003
    public static final double ELEVATOR_DOWN_I = 0.0001; 
    public static final double ELEVATOR_DOWN_D = 0.005;
    public static final double ELEVATOR_AUTON_FLOOR_SHOT = 5.0;
    public static final double ELEVATOR_SHOT = 12.0;
    public static final double ELEVATOR_MIN_POWER = 0.0; //0.0 
    public static final double ELEVATOR_MAX_POWER = 1.0; //0.75
    public static final double ELEVATOR_TOLERANCE = 0.0;
    
    public static final double DRIVE_DISTANCE_PER_PULSE = 0.0173135189727312*2;      //0.03420833;
    public static final double ELEVATOR_DISTANCE_PER_PULSE = (13/4561.0)*4.0;
    
    public static final double VOLTS_TO_PSI = 53.18;
    
    public static final int GYRO_INIT = 0;
    public static final int GYRO_READY  = 1;
    public static final int INTAKEON = 3;
    public static final int NOTREADYTOSHOOT = 4;
    public static final int READYTOSHOOT    = 5;
    
    public static final int LEFT_GOAL = 1;
    public static final int RIGHT_GOAL = 2;
    
    public static final double AMBIENT_LIGHT = 350;
}