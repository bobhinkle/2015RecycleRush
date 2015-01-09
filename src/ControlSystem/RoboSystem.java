
package ControlSystem;

import SubSystems.DriveTrain;
import Utilities.Ports;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.Victor;

public class RoboSystem{
    
    private static RoboSystem instance = null;
    
    private Victor motor0;
    private Victor motor1;
    private Victor motor2;
    private Victor motor3;
    private Victor motor4;
    private Victor motor5;
    private Victor motor6;
    private Victor motor7;
    private Victor motor8;
    private Victor motor9;
    public PowerDistributionPanel pdp = new PowerDistributionPanel();
    private Compressor comp;
    public DriveTrain dt;
    
    public static RoboSystem getInstance()
    {
        if( instance == null )
            instance = new RoboSystem();
        return instance;
    }
    
    public RoboSystem(){
    	motor8 = new Victor(Ports.MOTOR8);
    	motor9 = new Victor(Ports.MOTOR9);
    	comp = new Compressor(0);
    	comp.setClosedLoopControl(true);
    	dt = DriveTrain.getInstance();
    }
    
}
