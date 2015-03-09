package ControlSystem;

import SubSystems.Navigation;
import Utilities.Constants;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class FSM {

	public enum State{
    	DEFAULT, INIT, PRE_TOTE,
    	LOAD_TOTE,LOAD_TOTE_WAITING,LOAD_TOTE_STATIONARY_WAITING,INTAKING_TOTE,PRE_TOTE_ARM_OPEN,LOAD_TOTE_STATIONARY_WAITING_PRE2,
    	RC_LOAD, RC_ARM_CLOSE, RC_LOAD_WAITING,RC_INTAKING,RC_TO_PRE_TOTE, RC_WAITING_ON_LINE_BREAK,
    	WAITING_FOR_TOTE,
    	DROP_TOTES,PUSH_TOTES_AFTER_DROP, LOWER_TO_DROP_TOTES,
    	ZERO_ELEVATOR, MANUAL_TOTE,
    	TOTE_DROP_WAIT_FOR_COMPLETION, TOTE_DROP_RESET,
    	HUMAN_LOAD_START,HUMAN_LOAD_WAITING,HUMAN_LOAD_STAGE1,HUMAN_LOAD_STAGE2,HUMAN_LOAD_STAGE3,HUMAN_LOAD_FULL,
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
	private boolean armsOpen = true;
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
    public void toPreToteArmOpen(boolean response){
    	armsOpen = response;
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
				robot.elevator.run();
				robot.dt.run();
				nav.run();
				robot.dt.distance.run();
				robot.dt.heading.run();
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
    public void nextState(){
    	switch(previousState()){
    	
		case RC_LOAD_WAITING:
			setGoalState(FSM.State.RC_ARM_CLOSE);
			break;
		case RC_ARM_CLOSE:
			setGoalState(FSM.State.RC_INTAKING);
			break;
		case INTAKING_TOTE:
			setGoalState(FSM.State.WAITING_FOR_TOTE);
			break;
		case WAITING_FOR_TOTE:
			setGoalState(FSM.State.INTAKING_TOTE);
			break;
		case PRE_TOTE_ARM_OPEN:
			setGoalState(FSM.State.PRE_TOTE);
			break;
		case PRE_TOTE:
			setGoalState(FSM.State.WAITING_FOR_TOTE);
			break;
		case TOTE_DROP_WAIT_FOR_COMPLETION:
			setGoalState(FSM.State.TOTE_DROP_RESET);
			break;
		case HUMAN_LOAD_WAITING:
			setGoalState(FSM.State.HUMAN_LOAD_STAGE1);
			break;
		case HUMAN_LOAD_STAGE1:
			setGoalState(FSM.State.HUMAN_LOAD_STAGE2);
			break;
		case HUMAN_LOAD_STAGE2:
			setGoalState(FSM.State.HUMAN_LOAD_STAGE3);
			break;
		default:
			setGoalState(FSM.State.PRE_TOTE);
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
//            	System.out.println("PRE_TOTE");
            	robot.retractlatchRelease();
            	robot.elevator.setOnTargetThreshHold(20);
            	robot.elevator.closeTopStackHook();
            	robot.elevator.setGoal(Constants.ELEVATOR_INDEX_PRE_TOTE);
            	stateComplete(State.PRE_TOTE);
            	break;
            case PRE_TOTE_ARM_OPEN:
            	SmartDashboard.putString("FSM_STATE", "FLOOR_PICKUP");
//            	System.out.println("PRE_TOTE");
            	robot.retractlatchRelease();
            	robot.elevator.setOnTargetThreshHold(20);
            	robot.elevator.openTopStackHook();
            	robot.elevator.setGoal(Constants.ELEVATOR_INDEX_PRE_TOTE);
            	stateComplete(State.PRE_TOTE_ARM_OPEN);
            	break;
            case HUMAN_LOAD_START:
            	SmartDashboard.putString("FSM_STATE", "HL_START");
            	robot.elevator.setOnTargetThreshHold(20);
            	robot.retractlatchRelease();
            	robot.elevator.closeTopStackHook();
            	robot.elevator.setGoal(Constants.TOTE_4);
            	robot.retarctFollowerWheel();
            	setGoalState(State.HUMAN_LOAD_WAITING);
            	break;
            case HUMAN_LOAD_WAITING:
            	SmartDashboard.putString("FSM_STATE", "HL_WAITING");
            	if(robot.elevator.onTarget()){
            		stateComplete(State.HUMAN_LOAD_WAITING);
            	}
            	break;
            case HUMAN_LOAD_STAGE1:
            	SmartDashboard.putString("FSM_STATE", "HL_STAGE1");
            	robot.extendFollowerWheel();
            	robot.intakeRollersForward();
            	stateComplete(State.HUMAN_LOAD_STAGE1);
            	break;
            case HUMAN_LOAD_STAGE2:
            	SmartDashboard.putString("FSM_STATE", "HL_STAGE2");
            	robot.elevator.setOnTargetThreshHold(10);
            	robot.elevator.setGoal(Constants.ELEVATOR_HL_PICKUP);
            	setGoalState(State.HUMAN_LOAD_STAGE3);
            	break;
            case HUMAN_LOAD_STAGE3:
            	SmartDashboard.putString("FSM_STATE", "HL_STAGE3");
            	
            	if(robot.elevator.onTarget()){
                	robot.elevator.setOnTargetThreshHold(10);
                	robot.elevator.setGoal(Constants.TOTE_3);
                	setGoalState(State.HUMAN_LOAD_START);
            	}
            	break;
            case LOAD_TOTE:   //START LOAD TOTE SEQUENCE STEP 1
            	SmartDashboard.putString("FSM_STATE", "BOTTOM");
//            	System.out.println("LOAD_TOTE");
            	robot.elevator.setOnTargetThreshHold(20);
            	robot.elevator.setGoal(Constants.ELEVATOR_INDEX_LOADED);
            	setGoalState(State.LOAD_TOTE_WAITING);
            	break;
            case LOAD_TOTE_WAITING: //LOAD TOTE SEQUENCE STEP 2
            	SmartDashboard.putString("FSM_STATE", "LOAD_TOTE_WAITING");
//            	System.out.println("LOAD_TOTE_WAITING");
            	if(robot.elevator.onTarget()){
            		robot.openArm();
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
 //           	System.out.println("LOAD_TOTE_STATIONARY_WAITING");
            	if(robot.elevator.onTarget()){
            		robot.intakeRollersStop();
            		if(armsOpen){
            			setGoalState(State.PRE_TOTE_ARM_OPEN);
            		}else{
            			setGoalState(State.PRE_TOTE);
            		}
            	}
            	break;
            case LOAD_TOTE_STATIONARY_WAITING_PRE2:
            	SmartDashboard.putString("FSM_STATE", "LOAD_TOTE_STAT"); 
 //           	System.out.println("LOAD_TOTE_STATIONARY_WAITING");
            	if(robot.elevator.onTarget()){
            		robot.intakeRollersStop();
            		setGoalState(State.PRE_TOTE_ARM_OPEN);
            	}
            	break;
            case WAITING_FOR_TOTE:
            	SmartDashboard.putString("FSM_STATE", "WAITING FOR TOTE");
//            	System.out.println("WAITING_FOR_TOTE");
            	robot.openArm();
            	robot.intakeRollersForward();
            	stateComplete(State.WAITING_FOR_TOTE);
            	break;
            case INTAKING_TOTE:
            	SmartDashboard.putString("FSM_STATE", "INTAKING TOTE");
//            	System.out.println("INTAKING_TOTE");
            	if(robot.elevator.toteOnBumper() || bypassState){
            		setGoalState(State.LOAD_TOTE);
            		bypassState = false;
            		robot.elevator.increaseToteCount();
            		robot.intakeRollersStop();
            	}else{
            		robot.closeArm();
            		stateComplete(State.INTAKING_TOTE);
            	}
            	break;
            case SCORING:
            	robot.elevator.setOnTargetThreshHold(20);
            	robot.elevator.setGoal(Constants.ELEVATOR_LAST_TOTE);
            	setGoalState(State.SCORING_WAIT);
            	break;
            case SCORING_WAIT:
            	stateComplete(State.SCORING_WAIT);
            	break;
            case RC_LOAD:
            	SmartDashboard.putString("FSM_STATE", "RC LOAD");
            	robot.elevator.setOnTargetThreshHold(20);
            	robot.elevator.setGoal(Constants.ELEVATOR_INDEX_LOADED);
            	robot.openArm();
            	robot.retracttotePush();
            	robot.elevator.openTopStackHook();
        		robot.intakeRollersForward();   
            	setGoalState(State.RC_LOAD_WAITING);
            	break;
            case RC_LOAD_WAITING:
            	SmartDashboard.putString("FSM_STATE", "RC WAITING");
            	if(robot.elevator.onTarget()){
            		SmartDashboard.putString("FSM_STATE", "RC WAITING2");
            		stateComplete(State.RC_LOAD_WAITING);
            		System.out.println("RC_WAITING");
                	totePushDelayStart = System.currentTimeMillis();
            	}
            	break;
            case RC_ARM_CLOSE:
            	SmartDashboard.putString("FSM_STATE", "RC_ARM_CLOSE");
            	System.out.println("RCAMC");
            	robot.elevator.closeTopStackHook();
            	if(System.currentTimeMillis() > totePushDelay + totePushDelayStart ){
            		setGoalState(State.RC_INTAKING);
            	}
            	break;
            case RC_INTAKING:
            	SmartDashboard.putString("FSM_STATE", "RC_INTAKING");
            	robot.elevator.setOnTargetThreshHold(20);
            	robot.elevator.setGoal(Constants.ELVEVATOR_RC_INDEXED);
            	System.out.println("RC_INTATKING");
            	setGoalState(State.RC_TO_PRE_TOTE);
            	break;
            case RC_TO_PRE_TOTE:
            	SmartDashboard.putString("FSM_STATE", "RCPRETOTE");
            	if(robot.elevator.onTarget()){
            		robot.intakeRollersStop();   
            		robot.elevator.closeTopStackHook();
            		System.out.println("RC_PRETOTE");
            		setGoalState(State.PRE_TOTE);
            	}
            	break;
            case LOWER_TO_DROP_TOTES:
            	robot.elevator.setOnTargetThreshHold(20);
            	robot.elevator.setGoal(Constants.ELEVATOR_INDEX_LOADED);
            	robot.openArm();
				Timer.delay(0.5);
            	setGoalState(State.DROP_TOTES);
            	break;
            case DROP_TOTES:
            	if(robot.elevator.onTarget()){
                	robot.closeArm();
                	robot.elevator.openTopStackHook();
                	robot.extendlatchRelease();
                	totePushDelayStart = System.currentTimeMillis() + totePushDelay;
            		setGoalState(State.PUSH_TOTES_AFTER_DROP);
            		robot.elevator.resetToteCounter();
	            	robot.elevator.resetManualToteCount();
            	}else{
            		stateComplete(State.DROP_TOTES);
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
            	robot.elevator.closeTopStackHook();
            	robot.retractlatchRelease();
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