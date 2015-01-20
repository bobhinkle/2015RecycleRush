package Utilities;

/**
 *
 * @author Rohi Zacharia
 */
public class Constants {
    
    public static final boolean LOW_GEAR  = true; // Drivetrain low gear
    public static final double MIN_DT_POWER = 0.2;
    public static final int autonSelect = 2;
    
    public static final double STEERING_P = 0.015; //0.0
    public static final double STEERING_I = 0.0;  //
    public static final double STEERING_D = 0.0;  //0.0
    public static final double INPUT_DELAY = 0.25;
    
    public static final double DIST_KP = 0.04;
    public static final double DIST_KI = 0.0002; 
    public static final double DIST_KD = 0.000; 
    
    public static final double DIST_KP_BIG = 0.018;
    public static final double DIST_KI_BIG = 0.00001; 
    public static final double DIST_KD_BIG = 0.5; 
    
    public static final double DIST_SMALL = 10;
    
    public static final double STRAIGHT_KP = 0.009;//.012
    public static final double STRAIGHT_KI = 0.0;
    public static final double STRAIGHT_KD = 0.0;
    
    public static final double DISTANCE_TOLERANCE = 0.2; //auton distance pid
    ////////////////////////////////////////////////////////////////////////////////////////////
    public static final double ELEVATOR_MAX_HEIGHT  = 57.0;   // MAXIMUM ELEVATOR HEIGHT 57
    public static final double ELEVATOR_MIN_HEIGHT  = -10;
    public static final double ELEVATOR_DISTANCE_PER_PULSE = (56/20780.0)*4.0;
    public static final double ELEVATOR_CORRECTION = -0.5;
    public static final double ELEVATOR_P = 1.2;
    public static final double ELEVATOR_I = 0.000;
    public static final double ELEVATOR_D = 0.800;
    public static final double ELEVATOR_DOWN_P = 0.025; //0.003
    public static final double ELEVATOR_DOWN_I = 0.0001; 
    public static final double ELEVATOR_DOWN_D = 0.005;
    public static final double ELEVATOR_MIN_POWER = 0.0; 
    public static final double ELEVATOR_MAX_POWER = 1.0; 
    public static final double ELEVATOR_TOLERANCE = 0.1;
    public static final double ELEVATOR_INDEX_STATIONARY = 16.8 + ELEVATOR_CORRECTION;
    public static final double ELEVATOR_INDEX_PRE_TOTE = 11.9  + ELEVATOR_CORRECTION;
    public static final double ELEVATOR_INDEX_LOADED = ELEVATOR_MIN_HEIGHT  + ELEVATOR_CORRECTION;
    /////////////////////////////////////////////////////////////////////////////////////////////////
    public static final double DRIVE_DISTANCE_PER_PULSE = 0.0173135189727312*2;      //0.03420833;
    public static final double VOLTS_TO_PSI = 53.18;
    
    public static final int GYRO_INIT = 0;
    public static final int GYRO_READY  = 1;
    
    public static final double WHEELBASE_LENGTH = 33.1625;
    public static final double WHEELBASE_WIDTH  = 21.5;
    
    public static final double POWER_STALL = 10.0;
}