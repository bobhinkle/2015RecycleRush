package SubSystems;

import Sensors.SuperEncoder;
import Utilities.Constants;
import Utilities.Ports;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
	
public class Lifter{
	private static Lifter instance;
	private VictorSP drive;
	private DigitalInput loadingPoint;
	private State currentState = State.WAITING;
	private boolean running = false;
	private liftTimeout timeout;
	private boolean threadRunning = false;
	private boolean halfLoaded = false;
	private boolean lowerPosition = false;
	private SuperEncoder enc;
	private double distance = 0.0;
	private boolean encReset = false;
	public class liftTimeout extends Thread{
		private double endTime = 0;
		public liftTimeout(double timeout){
			endTime = timeout;
		}
		public void run(){
			threadRunning = true;
			Timer.delay(endTime);
			System.out.println("STOPPING");
			currentState = Lifter.State.STOP;
			threadRunning = false;
		}
	}
	public enum State{
		FORWARD_LOAD, REVERSE_LOAD, REVERSE_WAITING_FOR_RETURN,FORWARD_WAITING_ON_RETURN,FIRST_TRIGGER,
		WAITING,STOP,REVERSE_OFF_POINT,FORWARD_WAIT_FOR_PASS, REVERSE_WAIT_FOR_PASS,FORWARD_HALF_LIFT,DUMP_AND_WAIT,FORWARD_LOAD_HALF_WAIT,
		MANUAL_UP,MANUAL_DOWN,TOP_LIMIT
	}
	public Lifter(){
		drive = new VictorSP(Ports.LIFT);
		loadingPoint = new DigitalInput(Ports.LIFTER_LOWER_LIMIT);
		enc = new SuperEncoder(Ports.LIFT_ENC,Ports.LIFT_ENC+1,true,SuperEncoder.HIGH_RESOLUTION);
		enc.start();
	}
	public static Lifter getInstance(){
		if(instance == null){
			instance = new Lifter();
		}
		return instance;
	}
	public void reset(){
		encReset = true;
		enc.reset();
	}
	public boolean halfLoaded(){
		return halfLoaded;
	}
	public void clearHalfLoad(){
		halfLoaded = false;
	}
	public boolean threadRunning(){
		return threadRunning;
	}
	public void forwardLoad(){
		currentState = State.FORWARD_LOAD;
	}
	public void reverseLoad(){
		
	}
	public void setState(State state){
		if((state == State.FORWARD_LOAD || state == State.FORWARD_HALF_LIFT) && running){
			System.out.println("CALLED FORWARD");
		}
		currentState = state;
	}
	public State getState(){
		return currentState;
	}
	public void forward(){
		running = true;
		drive.set(1.0);
	}
	public void reverse(){
		running = true;
		drive.set(-0.6);
	}
	public void stopLift(){
		running = false;
		drive.set(0);
	}
	public boolean running(){
		return running;
	}
	//hooks wait at the bottom reversed just passed the limit switch. When loading pass the limit switch once then stop on second pass. Then reverse until 
	//not on switch anymore
	public void run(){
		lowerPosition = onBottom();
		distance = enc.getDistance();
		if(lowerPosition || (distance > Constants.LIFT_MAX_DISTANCE)){
			reset();
		}
		switch(currentState){
		case MANUAL_UP:
			forward();
			break;
		case MANUAL_DOWN:
			reverse();
			break;
		case WAITING:
			SmartDashboard.putString("LIFT STATE", "WAITING");
			break;
		case STOP:
			SmartDashboard.putString("LIFT STATE", "STOPPED");
			stopLift();
			setState(State.WAITING);
			break;
		case FORWARD_HALF_LIFT:
			forward();
			SmartDashboard.putString("LIFT STATE", "FHL");
			if(distance > Constants.LIFT_HALF_LOAD && distance < Constants.LIFT_DELATCH){
				setState(State.STOP);
			}
			System.out.println("HALF");
			break;
		case FORWARD_LOAD_HALF_WAIT:
			SmartDashboard.putString("LIFT STATE", "FHL WAIT");
			halfLoaded = true;
			System.out.println("HALF WAIT");
			break;
		case FORWARD_LOAD:
			SmartDashboard.putString("LIFT STATE", "FL");
			forward();
			if(distance > Constants.LIFT_TOP_CLEAR && distance < Constants.LIFT_LOWER_STOP){
				setState(State.TOP_LIMIT);
			}
		break;
		case FORWARD_WAITING_ON_RETURN:
			SmartDashboard.putString("LIFT STATE", "FWONP");
			if(lowerPosition){
				reverse();
				currentState = State.REVERSE_OFF_POINT;
			}
			break;
		case REVERSE_OFF_POINT:
			SmartDashboard.putString("LIFT STATE", "ROP");
			reverse();
			if(distance < Constants.LIFT_REVERSE_OFF){
				setState(State.STOP);				
			}
			break;
		case REVERSE_WAIT_FOR_PASS:
			SmartDashboard.putString("LIFT STATE", "RWFP");
			if(!threadRunning){
				timeout = new liftTimeout(1.0);
				timeout.start();
			}
			break;
		case FORWARD_WAIT_FOR_PASS:
			SmartDashboard.putString("LIFT STATE", "FWFP");
			if(!lowerPosition){
				currentState = State.FORWARD_WAITING_ON_RETURN;
			}
			break;
		case DUMP_AND_WAIT:
			SmartDashboard.putString("LIFT STATE", "DAW");
			forward();
			if(distance > Constants.LIFT_DELATCH){
				setState(State.STOP);
			}
			break;
		case TOP_LIMIT:
			if(distance > Constants.LIFT_LOWER_STOP){
				setState(State.STOP);
			}
			break;
		}		
		SmartDashboard.putBoolean("LIFTER", onBottom());
		SmartDashboard.putBoolean("Lift Running", running);
		SmartDashboard.putNumber("LIFT_DIST", enc.getDistance());
	}
	
	public class dumpThread extends Thread{
		private double endTime = 0;
		public dumpThread(double timeout){
			endTime = timeout;
		}
		public void run(){
			threadRunning = true;
			Timer.delay(endTime);
			System.out.println("STOPPING");
			currentState = Lifter.State.STOP;
			threadRunning = false;
		}
	}
	private boolean onBottom(){
		return !loadingPoint.get();
	}
}