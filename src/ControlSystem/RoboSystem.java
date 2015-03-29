
package ControlSystem;

import SubSystems.DriveTrain;
import SubSystems.Elevator;
import SubSystems.Lifter;
import Utilities.Ports;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.VictorSP;

public class RoboSystem{
	private Solenoid totePush;
	
    
    private static RoboSystem instance = null;
    private Compressor comp;
    public DriveTrain dt;
    public Elevator elevator;
    private Intake intake;
    private Solenoid follower_wheel;
    private Solenoid latchRelease;
    public Lifter lift;
    private VictorSP port11;
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
    	elevator = Elevator.getInstance();
    	intake = new Intake(Ports.INTAKE_ARM,Ports.INTAKE_ROLLER);
    	totePush = new Solenoid(Ports.TOTEPUSH);
    	follower_wheel = new Solenoid(Ports.FOLLOWER_WHEEL);
    	latchRelease = new Solenoid(Ports.LATCHRELEASE);
    	lift = Lifter.getInstance();
    	port11 = new VictorSP(10);
    }
    public void extendFollowerWheel(){
    	follower_wheel.set(true);
    }
    public void retarctFollowerWheel(){
    	follower_wheel.set(false);
    }
    public void actuateHLArm(){
    	if(follower_wheel.get())
    		follower_wheel.set(false);
    	else
    		follower_wheel.set(true);
    }
    public void extendtotePush(){
    	totePush.set(true);
    }
    public void retracttotePush(){
    	totePush.set(false);
    }
    public void retractlatchRelease(){
    	System.out.println("RELEASED");
    	latchRelease.set(true);
    }
    public void extendlatchRelease(){
    	latchRelease.set(false);
    }
    
    public void intakeRollersForward(){
    	intake.rollerForward();
    }
    
    public void intakeRollersReverse(){
    	intake.rollersReverse();
    }
    
    public void intakeRollersStop(){
    	intake.rollersStop();
    }
    public void actuateArm(){
    	intake.actuateArm();
    }
    public void openArm(){
    	intake.extendArm();
    }
    public void closeArm(){
    	intake.retractArm();
    }
    private class Intake{
    	private Solenoid arm;
    	private VictorSP roller;
    	private boolean intakeExtended = false;
    	
    	public Intake(int armPort, int rollerPort){
    		arm = new Solenoid(armPort);
    		roller = new VictorSP(rollerPort);
    	}
    	
    	public void rollerForward(){
        	roller.set(-1);
        }
        
        public void rollersReverse(){
        	roller.set(1);
        }
        
        public void rollersStop(){
        	roller.set(0);
        }
        public void rollerManualPower(double power){
        	roller.set(power);
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
