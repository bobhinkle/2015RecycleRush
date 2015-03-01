package ControlSystem;

import SubSystems.Navigation;
import Utilities.Constants;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class FSM {

	public enum State{
    	DEFAULT, INIT, PRE_TOTE,
    	LOAD_TOTE,LOAD_TOTE_WAITING,LOAD_TOTE_STATIONARY_WAITING,INTAKING_TOTE,
    	RC_LOAD, RC_LOAD_WAITING,RC_INTAKING,RC_TO_PRE_TOTE, RC_WAITING_ON_LINE_BREAK,
    	WAITING_FOR_TOTE,
    	DROP_TOTES,PUSH_TOTES_AFTER_DROP, LOWER_TO_DROP_TOTES,
    	ZERO_ELEVATOR, MANUAL_TOTE,
    	TOTE_DROP_WAIT_FOR_COMPLETION, TOTE_DROP_RESET
    }
	private RoboSystem robot;
	private static FSM instance = null;
	public partsUpdate pu;
	private State currentState = State.INIT;
    private State goalState = State.DEFAULT;
    private State prevState = State.DEFAULT;
    public Navigation nav;
	private boolean bypassState = false;
	private long totePushDelay = 500;
	private long totePushDelayStart = 0;
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
    private boolean checkStateChange(){
        if(goalState != currentState){
            return true;
       }
        return false;
    }
    public State getCurrentState() {
        return currentState;
    }
    private void stateComplete(State state){
        prevState = state;
        currentState = State.DEFAULT;
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
				robot.elevator.run();
				robot.dt.run();
				nav.run();
				Timer.delay(0.02); 
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
    	default:
    		setGoalState(State.DEFAULT);
    	break;
    	}
    }
    public void update(){ 
        switch(goalState){
            case INIT:
                SmartDashboard.putString("FSM_STATE", "INIT");
                stateComplete(State.INIT);
                break;
            case PRE_TOTE:
            	SmartDashboard.putString("FSM_STATE", "FLOOR_PICKUP");
            	robot.elevator.setOnTargetThreshHold(20);
            	robot.elevator.setGoal(Constants.ELEVATOR_INDEX_PRE_TOTE);
            	stateComplete(State.PRE_TOTE);
            	break;
            case LOAD_TOTE:   //START LOAD TOTE SEQUENCE STEP 1
            	SmartDashboard.putString("FSM_STATE", "BOTTOM");
            	robot.elevator.setOnTargetThreshHold(15);
            	robot.elevator.setGoal(Constants.ELEVATOR_INDEX_LOADED);
            	setGoalState(State.LOAD_TOTE_WAITING);
            	break;
            case LOAD_TOTE_WAITING: //LOAD TOTE SEQUENCE STEP 2
            	SmartDashboard.putString("FSM_STATE", "LOAD_TOTE_WAITING");
            	if(robot.elevator.onTarget()){
            		if(robot.elevator.getToteCount() == 6){
            			robot.elevator.setOnTargetThreshHold(20);
                		robot.elevator.setGoal(Constants.ELEVATOR_LAST_TOTE);
                		stateComplete(State.LOAD_TOTE_WAITING);
            		}else{
                		robot.elevator.setOnTargetThreshHold(20);
                		robot.elevator.setGoal(Constants.ELEVATOR_INDEX_STATIONARY);
                		setGoalState(State.LOAD_TOTE_STATIONARY_WAITING);
            		}
            	}
            	break;
            case LOAD_TOTE_STATIONARY_WAITING: //LOAD TOTE SEQUENCE STEP 3
            	SmartDashboard.putString("FSM_STATE", "LOAD_TOTE_STAT"); 
            	if(robot.elevator.onTarget()){
            		robot.intakeRollersStop();
            		setGoalState(State.PRE_TOTE);
            	}
            	break;
            case WAITING_FOR_TOTE:
            	SmartDashboard.putString("FSM_STATE", "WAITING FOR TOTE");
            	robot.openArm();
            	robot.intakeRollersForward();
            	stateComplete(State.WAITING_FOR_TOTE);
            	break;
            case INTAKING_TOTE:
            	SmartDashboard.putString("FSM_STATE", "INTAKING TOTE");
            	if(robot.elevator.toteOnBumper() || bypassState){
            		setGoalState(State.LOAD_TOTE);
            		robot.openArm();
            		bypassState = false;
            		robot.elevator.increaseToteCount();
            	}else{
            		robot.closeArm();
            	}
            	break;
            case RC_LOAD:
            	SmartDashboard.putString("FSM_STATE", "RC LOAD");
            	robot.elevator.setOnTargetThreshHold(25);
            	robot.elevator.setGoal(Constants.ELEVATOR_INDEX_LOADED);
            	robot.openArm();
            	setGoalState(State.RC_LOAD_WAITING);
            	break;
            case RC_LOAD_WAITING:
            	SmartDashboard.putString("FSM_STATE", "RC WAITING");
            	if(robot.elevator.onTarget()){
            		robot.intakeRollersForward();
            		setGoalState(State.RC_WAITING_ON_LINE_BREAK);
            	}
            	break;
            case RC_WAITING_ON_LINE_BREAK:
            	if(robot.elevator.lineBreakTrigger()){
            		setGoalState(State.RC_INTAKING);
            	}
//                	stateComplete(FSM.RC_WAITING_ON_LINE_BREAK);
            	break;
            case RC_INTAKING:
            	SmartDashboard.putString("FSM_STATE", "RC DROP OFF");
            	robot.elevator.setOnTargetThreshHold(50);
            	robot.elevator.setGoal(Constants.ELVEVATOR_RC_INDEXED);
            	setGoalState(State.RC_TO_PRE_TOTE);
            	break;
            case RC_TO_PRE_TOTE:
            	SmartDashboard.putString("FSM_STATE", "RC DROPPED TO PRETOTE");
            	if(robot.elevator.onTarget()){
            		robot.intakeRollersStop();
            		setGoalState(State.PRE_TOTE);
            	}
            	break;
            case LOWER_TO_DROP_TOTES:
            	robot.elevator.setOnTargetThreshHold(10);
            	robot.elevator.setGoal(Constants.ELEVATOR_INDEX_LOADED);
            	setGoalState(State.DROP_TOTES);
            	break;
            case DROP_TOTES:
            	if(robot.elevator.onTarget()){
                	robot.closeArm();
                	robot.elevator.stackRelease();
                	totePushDelayStart = System.currentTimeMillis() + totePushDelay;
            		setGoalState(State.PUSH_TOTES_AFTER_DROP);
            	}
            	break;
            case PUSH_TOTES_AFTER_DROP:
            	if(System.currentTimeMillis() > totePushDelayStart){
	        		robot.intakeRollersReverse();
	            	robot.elevator.resetToteCounter();
	            	robot.elevator.resetManualToteCount();
	            	robot.extendtotePush();
	        		setGoalState(State.TOTE_DROP_WAIT_FOR_COMPLETION);
            	}
            	break;
            case TOTE_DROP_WAIT_FOR_COMPLETION:
            	stateComplete(State.TOTE_DROP_WAIT_FOR_COMPLETION);
            	break;
            case TOTE_DROP_RESET:
            	robot.intakeRollersStop();
            	robot.retracttotePush();
            	robot.elevator.stackHold();
            	setGoalState(State.PRE_TOTE);
            	break;
            case ZERO_ELEVATOR:
            	robot.elevator.setGoal(Constants.EVEVATOR_ZERO);
            	setGoalState(State.DEFAULT);
            	break;
            case MANUAL_TOTE:
            	switch(robot.elevator.getManualToteCount()){
            	case 0:
            		robot.elevator.setGoal(Constants.ELEVATOR_INDEX_LOADED);
            		break;
            	case 1:
            		robot.elevator.setGoal(Constants.ELEVATOR_INDEX_PRE_TOTE);
            		break;
            	case 2:
            		robot.elevator.setGoal(Constants.TOTE_2);
            		break;
            	case 3:
            		robot.elevator.setGoal(Constants.TOTE_3);
            		break;
            	default:
            		robot.elevator.setGoal(Constants.ELEVATOR_INDEX_PRE_TOTE);
            	}
            	setGoalState(State.DEFAULT);
            	break;
            case DEFAULT:
            	SmartDashboard.putString("FSM_STATE", "WAITING");
            	break;
        }
    }
}