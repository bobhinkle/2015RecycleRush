/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package Utilities;

/**
 *
 * @author xpsl05x
 */
public class Ports {
    
    //PWM
    public static final int FRONT_RIGHT_DRIVE   = 0; 
    public static final int FRONT_RIGHT_ROTATION= 2; 
    public static final int FRONT_LEFT_DRIVE    = 1; 
    public static final int FRONT_LEFT_ROTATION = 3;
    public static final int REAR_LEFT_DRIVE     = 5;
    public static final int REAR_LEFT_ROTATION  = 7;
    public static final int REAR_RIGHT_DRIVE    = 4;
    public static final int REAR_RIGHT_ROTATION = 6;
    
    public static final int ELEVATOR        	= 8;
    public static final int ELEVATOR2			= 10;
    public static final int INTAKE_ROLLER1  	= 9;
    public static final int INTAKE_ROLLER2  	= 11;
    
    //DIGITAL INPUTS
    public static final int ELEVATOR_BOTTOM_LIMIT = 0;
    public static final int ELEVATOR_TOP_LIMIT = 4;
    public static final int ELEVATOR_ENC = 10; 
    public static final int NAV_X_ENC = 14;
    public static final int NAV_Y_ENC = 16;
    public static final int TOTE_BUMPER = 1;
    //ANALOG INPUTS
    
    public static final int FRONT_RIGHT_MA3 = 0;
    public static final int FRONT_LEFT_MA3  = 1;
    public static final int REAR_LEFT_MA3   = 2;
    public static final int REAR_RIGHT_MA3  = 3;
    public static final int REAR_LINE_BREAK = 6;
    public static final int GYRO		   = 5;
    public static final int GYRO2		   = 1;
    
    //SOLENOIDS
    public static final int STATIONARY_WANG = 0;
    public static final int INTAKE_ARM = 6;
    public static final int TOP_REVERSE_CARRIAGE_ARMS = 7;
    public static final int TOTEPUSH = 5;
}
