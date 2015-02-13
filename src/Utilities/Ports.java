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
    
    public static final int ELEVATOR        	= 9;
    public static final int INTAKE_ROLLER1  	= 10;
    public static final int INTAKE_ROLLER2  	= 11;
    
    //DIGITAL INPUTS
    public static final int ELEVATOR_BOTTOM_LIMIT = 0;
    public static final int ELEVATOR_TOP_LIMIT = 1;
    public static final int ELEVATOR_ENC = 2; //1 AND 2
    public static final int NAV_X_ENC = 4;
    public static final int NAV_Y_ENC = 6;
    public static final int TOTE_BUMPER = 9;
    //ANALOG INPUTS
    
    public static final int FRONT_RIGHT_MA3 = 0;
    public static final int FRONT_LEFT_MA3  = 1;
    public static final int REAR_LEFT_MA3   = 2;
    public static final int REAR_RIGHT_MA3  = 3;
    public static final int REAR_LINE_BREAK = 4;
    public static final int GYRO		   = 5;
    public static final int GYRO2		   = 6;
    
    //SOLENOIDS
    public static final int STATIONARY_WANG = 0;
    public static final int INTAKE_ARM = 1;
    public static final int INTAKE_ARM2 = 3;
}
