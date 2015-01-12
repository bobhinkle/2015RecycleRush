
package ControlSystem;

import Sensors.Arduino;
import SubSystems.DriveTrain;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PowerDistributionPanel;

public class RoboSystem{
    
    private static RoboSystem instance = null;
    public PowerDistributionPanel pdp = new PowerDistributionPanel();
    private Compressor comp;
    public DriveTrain dt;
    public Arduino ahrs;
    public static RoboSystem getInstance()
    {
        if( instance == null )
            instance = new RoboSystem();
        return instance;
    }
    
    public RoboSystem(){
    	comp = new Compressor(0);
    	comp.setClosedLoopControl(true);
    	dt = DriveTrain.getInstance();
    	ahrs = Arduino.getInstance();
    }
    
}
