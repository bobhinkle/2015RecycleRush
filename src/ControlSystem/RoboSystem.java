
package ControlSystem;

import Sensors.Arduino;
import SubSystems.DriveTrain;
import SubSystems.Elevator;
import Utilities.Ports;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Victor;

public class RoboSystem{
    
    private static RoboSystem instance = null;
    private Compressor comp;
    public DriveTrain dt;
    public Arduino ahrs;
    public Elevator elevator;
    private Intake intake1, intake2;
    
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
    	elevator = Elevator.getInstance();
    	intake1 = new Intake(Ports.INTAKE_ARM,Ports.INTAKE_ROLLER1);
    	intake2 = new Intake(Ports.INTAKE_ARM2,Ports.INTAKE_ROLLER2);
    }
    
    public void intakeRollersForward(){
    	intake1.rollerForward();
    	intake2.rollerForward();
    }
    
    public void intakeRollersReverse(){
    	intake1.rollersReverse();
    	intake2.rollersReverse();
    }
    
    public void intakeRollersStop(){
    	intake1.rollersStop();
    	intake2.rollersStop();
    }
    public void actuateArm(){
    	intake1.actuateArm();
    	intake2.actuateArm();
    }
    public void openArm(){
    	intake1.extendArm();
    	intake2.extendArm();
    }
    public void closeArm(){
    	intake1.retractArm();
    	intake2.retractArm();
    }
    private class Intake{
    	private Solenoid arm;
    	private Victor roller;
    	private boolean intakeExtended = false;
    	
    	public Intake(int armPort, int rollerPort){
    		arm = new Solenoid(armPort);
    		roller = new Victor(rollerPort);
    	}
    	
    	public void rollerForward(){
        	roller.set(1);
        }
        
        public void rollersReverse(){
        	roller.set(-1);
        }
        
        public void rollersStop(){
        	roller.set(0);
        }
        
        public void extendArm(){
        	arm.set(true);
        	intakeExtended = true;
        }
        public void retractArm(){
        	intakeExtended = false;
        	arm.set(false);
        }
        public void actuateArm(){
        	if(intakeExtended){
        		retractArm();
        	}else{
        		extendArm();
        	}
        }
    }
}
