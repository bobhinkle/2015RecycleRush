package IO;

import edu.wpi.first.wpilibj.*;

/** extends the joystick class and adds methods for each button on an xbox
 * controller
 *
 * @author Robotics
 */

public class Xbox extends Joystick
{
    private static final double PRESS_THRESHHOLD = 0.80;
    public static final double DEAD_BAND = 0.15;

    public Xbox(int usb)   { super(usb); }

    public boolean getAButton()      { return getRawButton(1); }
    public boolean getBButton()      { return getRawButton(2); }
    public boolean getXButton()      { return getRawButton(3); }
    public boolean getYButton()      { return getRawButton(4); }
    public boolean getLeftBumper()   { return getRawButton(5); }
    public boolean getRightBumper()  { return getRawButton(6); }
    public boolean getBackButton()   { return getRawButton(7); }
    public boolean getStartButton()  { return getRawButton(8); }
    public double  getLeftStickX()   { return getRawAxis(1); }
    public double  getLeftStickY()   { return getRawAxis(2); }
    public double  getRightStickX()  { return getRawAxis(4); }
    public double  getRightStickY()  { return getRawAxis(5); }
    public double  getDPADX()        { return getRawAxis(6); }
    public double  getDPADY()        { return getRawAxis(7); }
    public boolean getLeftStick()    { return getRawButton(9); }
    public boolean getRightStick()   { return getRawButton(10); }
    public boolean getRightTrigger() 
    { 
        return getRawAxis(3) < -PRESS_THRESHHOLD;
    }
    public boolean getLeftTrigger()  
    { 
        return getRawAxis(3) > PRESS_THRESHHOLD;
    }
    
}