package ControlSystem;

import SubSystems.Lifter;
import SubSystems.Navigation;
import Utilities.Constants;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class FSM {

	public enum State{
    	DEFAULT, INIT, PRE_TOTE,
    	LOAD_TOTE,LOAD_TOTE_WAITING,LOAD_TOTE_STATIONARY_WAITING,INTAKING_TOTE,
    	RC_LOAD, RC_TOP, RC_LOAD_WAITING,RC_INTAKING,RC_LOADED, RC_WAITING_ON_LINE_BREAK,
    	WAITING_FOR_TOTE,
    	DROP_TOTES,PUSH_TOTES_AFTER_DROP, LOWER_TO_DROP_TOTES, MANUAL_TOTE,
    	TOTE_DROP_WAIT_FOR_COMPLETION, TOTE_DROP_RESET,    	
    	SCORING, SCORING_WAIT,BYPASS
    }
	private RoboSystem robot;
	private static FSM instance = null;
	public partsUpdate pu;
	private State currentState = State.INIT;
    private State goalState = State.BYPASS;
    private State prevState = State.BYPASS;
    public Navigation nav;
	private boolean bypassState = false;
	private long totePushDelay = 1000;
	private long totePushDelayStart = 0;
	private boolean lastTote = false;
	public static FSM getInstance()
    {
        if( instance == null )
            instance = new FSM();
        return instance;
    }
        
    public FSM() {
    	SmartDashboard.putString("FSM", "STARTED");
        robot = RoboSystem.getInstance();
        pu = new partsUpdate();
    	pu.start();
    	nav = Navigation.getInstance();
    }
    public void lastTote(){
    	lastTote = true;
    }
    public void clearLastTote(){
    	lastTote = false;
    }
    public void setGoalState(State goal) {
        if(currentState == goal){
            currentState = State.DEFAULT;
            goalState = goal;
        }else{
            goalState = goal;
        }
    }
    public void bypassState(){
    	bypassState = true;
    }
    public State getCurrentState() {
        return currentState;
    }
    private void stateComplete(State state){
        prevState = state;
        currentState = State.PRE_TOTE;
    }
    public State previousState(){
    	return prevState;
    }
    public class partsUpdate extends Thread{
        private boolean keepRunning = true;
    	public void run(){
    		SmartDashboard.putString("FSM", "THREAD STARTED");
    		while(keepRunning){
				update();
				robot.canLift.run();
				robot.dt.run();
				nav.run();
				robot.dt.distance.run();
				robot.dt.heading.run();
				robot.lift.run();
				Timer.delay(0.005); 
    		}
        }
        public void kill(){
        	keepRunning = false;
        }
    }
    public void fsmStopState(){
    	switch(goalState){
    	case LOAD_TOTE_WAITING:
    		setGoalState(State.LOAD_TOTE);
    		break;
    	case LOAD_TOTE_STATIONARY_WAITING:
    		setGoalState(State.PRE_TOTE);
    		break;
    	case WAITING_FOR_TOTE:
    		setGoalState(State.PRE_TOTE);
    		break;
    	case RC_LOAD_WAITING:
    		setGoalState(State.RC_LOAD_WAITING);
    		break;
    	case DROP_TOTES:
    		setGoalState(State.DROP_TOTES);
    		break;
    	default:
    		setGoalState(State.DEFAULT);
    	break;
    	}
    }
    /*
    public void nextState(){
    	switch(previousState()){
    	
		case RC_LOAD:
			setGoalState(FSM.State.RC_TOP);
			break;
		}
    }*/
    public void update(){ 
        switch(goalState){
            case INIT:
                SmartDashboard.putString("FSM_STATE", "INIT");
                stateComplete(State.INIT);
                break;
            case PRE_TOTE:
            	SmartDashboard.putString("FSM_STATE", "PRETOTE");
//            	System.out.println("PRE_TOTE");
            	robot.intakeRollersStop();
            	stateComplete(State.PRE_TOTE);
            	break;
            case LOAD_TOTE:   //START LOAD TOTE SEQUENCE STEP 1
            	SmartDashboard.putString("FSM_STATE", "BOTTOM");
//            	System.out.println("LOAD_TOTE");
            	if(robot.lift.getState() == Lifter.State.TOP_LIMIT){
            		setGoalState(State.LOAD_TOTE_WAITING);
            	}
            	
            	break;
            case LOAD_TOTE_WAITING: //LOAD TOTE SEQUENCE STEP 2
            	SmartDashboard.putString("FSM_STATE", "LOAD_TOTE_WAITING");
//            	System.out.println("LOAD_TOTE_WAITING");
            	if(robot.lift.getState() == Lifter.State.WAITING && !robot.lift.threadRunning()){
            		setGoalState(State.LOAD_TOTE_STATIONARY_WAITING);
            	}
            	break;
            case LOAD_TOTE_STATIONARY_WAITING: //LOAD TOTE SEQUENCE STEP 3
            	SmartDashboard.putString("FSM_STATE", "LOAD_TOTE_STAT"); 
 //           	System.out.println("LOAD_TOTE_STATIONARY_WAITING");
            	if(robot.lift.halfLoaded())
    				setGoalState(State.PRE_TOTE);
    			else
    				setGoalState(State.INTAKING_TOTE);
            	break;
            case WAITING_FOR_TOTE:
            	SmartDashboard.putString("FSM_STATE", "WAITING FOR TOTE");
//            	System.out.println("WAITING_FOR_TOTE");
            	robot.extendlatchRelease();
            	robot.closeIntake();
            	robot.intakeRollersForward();
            	setGoalState(State.INTAKING_TOTE);
            	break;
            case INTAKING_TOTE:
            	SmartDashboard.putString("FSM_STATE", "INTAKING TOTE");
            	if(robot.canLift.toteOnBumper() || bypassState){
            		setGoalState(State.LOAD_TOTE);
            		bypassState = false;
            		if(lastTote || robot.canLift.topLimit()){
            			lastTote();
            			robot.lift.setState(Lifter.State.FORWARD_HALF_LIFT);
            		}
            		else
            			robot.lift.setState(Lifter.State.FORWARD_LOAD);
            	}else{
            		stateComplete(State.INTAKING_TOTE);
            	}            	
            	break;
            case SCORING_WAIT:
            	stateComplete(State.SCORING_WAIT);
            	break;
            case RC_LOAD:
            	SmartDashboard.putString("FSM_STATE", "RC LOAD"); 
            	robot.canLift.openClapper();
            	robot.openIntake();
            	robot.retractlatchRelease();
            	Timer.delay(0.5);
            	robot.canLift.setGoal(Constants.CAN_LOAD);
            	setGoalState(State.RC_LOADED);
            	break;            
            case RC_TOP:
            	SmartDashboard.putString("FSM_STATE", "RC_ARM_CLOSE");
            	System.out.println("RCAMC");
            	robot.canLift.openClapper();
            	robot.openIntake();
            	robot.retractlatchRelease();
            	robot.intakeRollersForward();
            	Timer.delay(0.5);
            	robot.canLift.setGoal(Constants.CAN_TOP);
            	setGoalState(State.RC_LOAD_WAITING);
            	break;
            case RC_LOAD_WAITING:
            	SmartDashboard.putString("FSM_STATE", "RC WAITING");
            	if(robot.canLift.onTargetNow()){
            		SmartDashboard.putString("FSM_STATE", "RC WAITING2");            		
            		setGoalState(State.RC_LOADED);
            	}
            	if(robot.canLift.getAngle() > Constants.CAN_TOP - 5.0){
            		robot.canLift.closeClapper();
            		robot.intakeRollersStop();
            	}
            	break;
            case RC_LOADED:
            	SmartDashboard.putString("FSM_STATE", "RCLOADED");
            	break;
            case RC_INTAKING:
            	SmartDashboard.putString("FSM_STATE", "RC_INTAKING");
            	System.out.println("RC_INTATKING");
            	setGoalState(State.RC_LOADED);
            	break;
            
            case LOWER_TO_DROP_TOTES:
            	robot.openIntake();
            	robot.lift.setState(Lifter.State.DUMP_AND_WAIT);		
            	setGoalState(State.DROP_TOTES);
            	break;
            case DROP_TOTES:
            	if(robot.lift.getState() == Lifter.State.WAITING){
					robot.retractlatchRelease();
					robot.canLift.openClapper();
            		robot.openIntake();
					Timer.delay(0.5);
					robot.lift.setState(Lifter.State.REVERSE_OFF_POINT);
                	totePushDelayStart = System.currentTimeMillis() + totePushDelay;                	
	            	setGoalState(State.PUSH_TOTES_AFTER_DROP);
				}	
            	break;
            case PUSH_TOTES_AFTER_DROP:
            	if(System.currentTimeMillis() > totePushDelayStart){
            		robot.intakeRollersReverse();
	        		robot.lift.clearHalfLoad();
	        		setGoalState(State.TOTE_DROP_WAIT_FOR_COMPLETION);
            	}
            	break;
            case TOTE_DROP_WAIT_FOR_COMPLETION:
            	stateComplete(State.TOTE_DROP_WAIT_FOR_COMPLETION);
            	break;
            case TOTE_DROP_RESET:
            	robot.intakeRollersStop();
            	robot.extendlatchRelease();
            	setGoalState(State.PRE_TOTE);
            	break;
            default:
            	SmartDashboard.putString("FSM_STATE", "WAITING");
            	break;
        }
    }
}