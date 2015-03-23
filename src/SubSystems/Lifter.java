package SubSystems;

import Utilities.Ports;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
	
public class Lifter{
	private static Lifter instance;
	private Victor drive;
	private DigitalInput loadingPoint;
	private boolean loading = false;
	private boolean waitingForReturn = false;
	private State currentState = State.WAITING;
	private boolean running = false;
	private liftTimeout timeout;
	private boolean threadRunning = false;
	private boolean halfLoaded = false;
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
		WAITING,STOP,REVERSE_OFF_POINT,FORWARD_WAIT_FOR_PASS, REVERSE_WAIT_FOR_PASS,FORWARD_HALF_LIFT,DUMP_AND_REVERSE,FORWARD_LOAD_HALF_WAIT,
		MANUAL_UP,MANUAL_DOWN
	}
	public Lifter(){
		drive = new Victor(Ports.LIFT);
		loadingPoint = new DigitalInput(Ports.LIFTER_LOWER_LIMIT);
	}
	public static Lifter getInstance(){
		if(instance == null){
			instance = new Lifter();
		}
		return instance;
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
		drive.set(1);
	}
	public void reverse(){
		running = true;
		drive.set(-1);
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
			if(!threadRunning){
				timeout = new liftTimeout(0.35);
				timeout.start();
			}
			currentState = State.FORWARD_LOAD_HALF_WAIT;
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
			if(onPoint()){
				setState(State.STOP);
			}
		break;
		case FORWARD_WAITING_ON_RETURN:
			SmartDashboard.putString("LIFT STATE", "FWONP");
			if(onPoint()){
				reverse();
				currentState = State.REVERSE_OFF_POINT;
			}
			break;
		case REVERSE_OFF_POINT:
			SmartDashboard.putString("LIFT STATE", "ROP");
			reverse();
			if(onPoint()){
				currentState = State.STOP;				
			}
			break;
		case REVERSE_WAIT_FOR_PASS:
			SmartDashboard.putString("LIFT STATE", "RWFP");
			if(onPoint()){
				currentState = State.REVERSE_OFF_POINT;
			}
			break;
		case FORWARD_WAIT_FOR_PASS:
			SmartDashboard.putString("LIFT STATE", "FWFP");
			if(!onPoint()){
				currentState = State.FORWARD_WAITING_ON_RETURN;
			}
			break;
		case DUMP_AND_REVERSE:
			SmartDashboard.putString("LIFT STATE", "DAR");
			forward();
			System.out.println("DUMP&RELEASE1");
			if(!threadRunning){
				timeout = new liftTimeout(0.2);
				timeout.start();
			}
			System.out.println("DUMP&RELEASE2");
			break;
		}
		SmartDashboard.putBoolean("LIFTER", onPoint());
		SmartDashboard.putBoolean("Lift Running", running);
	}
	
	private boolean onPoint(){
		return !loadingPoint.get();
	}
}