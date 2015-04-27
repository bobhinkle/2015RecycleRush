
package ControlSystem;

import SubSystems.CanLift;
import SubSystems.DriveTrain;
import SubSystems.Lifter;
import Utilities.Ports;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.VictorSP;

public class RoboSystem{
    private static RoboSystem instance = null;
    private Compressor comp;
    public DriveTrain dt;
    public CanLift canLift;
    private Intake intake;
    private Solenoid follower_wheel;
    private Solenoid latchRelease;
    private Solenoid canGrabber;
    private Solenoid canGrabberRelease;
    public Lifter lift;
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
    	canLift = CanLift.getInstance();
    	intake = new Intake(Ports.INTAKE_ARM,Ports.INTAKE_ROLLER);
    	follower_wheel = new Solenoid(1,Ports.FOLLOWER_WHEEL);
    	latchRelease = new Solenoid(1,Ports.LATCHRELEASE);
    	lift = Lifter.getInstance();
    	canGrabber = new Solenoid(0,Ports.CAN_GRABBERS);
    	canGrabberRelease = new Solenoid(1,Ports.CAN_GRABBER_RELEASE);
    }
    public void retractCanGrabber(){
    	canGrabber.set(false);
    	canGrabberRelease.set(true);
    }
    public void extendCanGrabber(){
    	canGrabber.set(true);
    	canGrabberRelease.set(false);
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
    public void openIntake(){
    	intake.extendArm();
    }
    public void closeIntake(){
    	intake.retractArm();
    }
    private class Intake{
    	private Solenoid arm;
    	private VictorSP roller;
    	private boolean intakeExtended = false;
    	
    	public Intake(int armPort, int rollerPort){
    		arm = new Solenoid(1,armPort);
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
