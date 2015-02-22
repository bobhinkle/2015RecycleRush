package ControlSystem;

import SubSystems.Navigation;
import Utilities.Constants;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class FSM {

	private RoboSystem robot;
	private static FSM instance = null;
	public partsUpdate pu;
	private int currentState = INIT;
    private int goalState = DEFAULT;
    private int prevState = DEFAULT;
    public Navigation nav;
	public final static int DEFAULT = -1;
	public final static int INIT = 0;
	public final static int PRE_TOTE = 1;
	public final static int LOAD_TOTE = 2;
	public final static int LOAD_TOTE_WAITING = 3;
	public final static int LOAD_TOTE_STATIONARY_WAITING = 4;
	public final static int INTAKING_TOTE = 5;
	public final static int RC_LOAD = 6;
	public final static int RC_LOAD_WAITING = 7;
	public final static int RC_INTAKING     = 8;
	public final static int RC_TO_PRE_TOTE  = 9;
	public final static int RC_WAITING_ON_LINE_BREAK = 10;
	public final static int WAITING_FOR_TOTE = 11;
	public final static int DROP_TOTES       = 12;
	public final static int PUSH_TOTES_AFTER_DROP = 13;
	public final static int LOWER_TO_DROP_TOTES     = 14;
	public final static int ZERO_ELEVATOR   = 15;
	public final static int MANUAL_TOTE = 16;
	private boolean bypassState = false;
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
    public void setGoalState(int goal) {
        if(currentState == goal){
            currentState = FSM.DEFAULT;
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
    public int getCurrentState() {
        return currentState;
    }
    private void stateComplete(int state){
        prevState = state;
        currentState = DEFAULT;
    }
    public int previousState(){
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
    
    public void update(){ 
        switch(goalState){
            case INIT:
                SmartDashboard.putString("FSM_STATE", "INIT");
                stateComplete(FSM.INIT);
                break;
            case PRE_TOTE:
            	SmartDashboard.putString("FSM_STATE", "FLOOR_PICKUP");
            	robot.elevator.setOnTargetThreshHold(20);
            	robot.elevator.setGoal(Constants.ELEVATOR_INDEX_PRE_TOTE);
            	stateComplete(FSM.PRE_TOTE);
            	break;
            case LOAD_TOTE:   //START LOAD TOTE SEQUENCE STEP 1
            	SmartDashboard.putString("FSM_STATE", "BOTTOM");
            	robot.elevator.setOnTargetThreshHold(15);
            	robot.elevator.setGoal(Constants.ELEVATOR_INDEX_LOADED);
            	setGoalState(FSM.LOAD_TOTE_WAITING);
            	break;
            case LOAD_TOTE_WAITING: //LOAD TOTE SEQUENCE STEP 2
            	SmartDashboard.putString("FSM_STATE", "LOAD_TOTE_WAITING");
            	if(robot.elevator.onTarget()){
            		if(robot.elevator.getToteCount() == 6){
            			robot.elevator.setOnTargetThreshHold(20);
                		robot.elevator.setGoal(Constants.ELEVATOR_LAST_TOTE);
                		stateComplete(FSM.LOAD_TOTE_WAITING);
            		}else{
                		robot.elevator.setOnTargetThreshHold(20);
                		robot.elevator.setGoal(Constants.ELEVATOR_INDEX_STATIONARY);
                		setGoalState(FSM.LOAD_TOTE_STATIONARY_WAITING);
            		}
            	}
            	break;
            case LOAD_TOTE_STATIONARY_WAITING: //LOAD TOTE SEQUENCE STEP 3
            	SmartDashboard.putString("FSM_STATE", "LOAD_TOTE_STAT"); //adriana says this makes sense. George says the opposite.
            	if(robot.elevator.onTarget()){
            		robot.intakeRollersStop();
            		setGoalState(FSM.PRE_TOTE);
            	}
            	break;
            case WAITING_FOR_TOTE:
            	SmartDashboard.putString("FSM_STATE", "WAITING FOR TOTE");
            	robot.openArm();
            	robot.intakeRollersForward();
            	stateComplete(FSM.WAITING_FOR_TOTE);
            	break;
            case INTAKING_TOTE:
            	SmartDashboard.putString("FSM_STATE", "INTAKING TOTE");
            	if(robot.elevator.toteOnBumper() || bypassState){
            		setGoalState(FSM.LOAD_TOTE);
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
            	setGoalState(FSM.RC_LOAD_WAITING);
            	break;
            case RC_LOAD_WAITING:
            	SmartDashboard.putString("FSM_STATE", "RC WAITING");
            	if(robot.elevator.onTarget()){
            		robot.intakeRollersForward();
            		setGoalState(FSM.RC_WAITING_ON_LINE_BREAK);
            	}
            	break;
            case FSM.RC_WAITING_ON_LINE_BREAK:
            	if(robot.elevator.lineBreakTrigger()){
            		setGoalState(FSM.RC_INTAKING);
            	}
//                	stateComplete(FSM.RC_WAITING_ON_LINE_BREAK);
            	break;
            case RC_INTAKING:
            	SmartDashboard.putString("FSM_STATE", "RC DROP OFF");
            	robot.elevator.setOnTargetThreshHold(50);
            	robot.elevator.setGoal(Constants.ELVEVATOR_RC_INDEXED);
            	setGoalState(FSM.RC_TO_PRE_TOTE);
            	break;
            case RC_TO_PRE_TOTE:
            	SmartDashboard.putString("FSM_STATE", "RC DROPPED TO PRETOTE");
            	if(robot.elevator.onTarget()){
            		robot.intakeRollersStop();
            		setGoalState(FSM.PRE_TOTE);
            	}
            	break;
            case LOWER_TO_DROP_TOTES:
            	robot.elevator.setOnTargetThreshHold(10);
            	robot.elevator.setGoal(Constants.ELEVATOR_INDEX_LOADED);
            	setGoalState(FSM.DROP_TOTES);
            	break;
            case DROP_TOTES:
            	if(robot.elevator.onTarget()){
                	robot.closeArm();
            		setGoalState(FSM.PUSH_TOTES_AFTER_DROP);
            	}
            	break;
            case PUSH_TOTES_AFTER_DROP:
        		robot.intakeRollersReverse();
            	robot.elevator.resetToteCounter();
            	robot.elevator.resetManualToteCount();
        		stateComplete(PUSH_TOTES_AFTER_DROP);
            	break;
            case ZERO_ELEVATOR:
            	robot.elevator.setGoal(Constants.EVEVATOR_ZERO);
            	setGoalState(DEFAULT);
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
            	setGoalState(DEFAULT);
            	break;
            case DEFAULT:
            	SmartDashboard.putString("FSM_STATE", "WAITING");
            	break;
        }
    }
}