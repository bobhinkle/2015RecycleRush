package ControlSystem;

import Utilities.Constants;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class FSM {

	private RoboSystem robot;
	private static FSM instance = null;
	public partsUpdate pu;
	private int currentState = INIT;
    private int goalState = DEFAULT;
    
	public final static int DEFAULT = -1;
	public final static int INIT = 0;
	public final static int PRE_TOTE = 1;
	public final static int LOAD_TOTE = 2;
	public final static int LOAD_TOTE_WAITING = 3;
	public final static int LOAD_TOTE_STATIONARY_WAITING = 4;
	public final static int INTAKING_TOTE = 5;
	
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
    }	
    public void setGoalState(int goal) {
        if(currentState == goal){
            currentState = FSM.DEFAULT;
            goalState = goal;
        }else{
            goalState = goal;
        }
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
        currentState = state;
    }
    	
    public class partsUpdate extends Thread{
        private boolean keepRunning = true;
    	public void run(){
    		SmartDashboard.putString("FSM", "THREAD STARTED");
    		while(keepRunning){
				update();
				robot.elevator.run();
				robot.dt.run();
				Timer.delay(0.02); 
    		}
        }
        public void kill(){
        	keepRunning = false;
        }
    }
    
    public void update(){ 
        if(checkStateChange()){
            switch(goalState){
                case INIT:
                    SmartDashboard.putString("FSM_STATE", "INIT");
                    stateComplete(FSM.INIT);
                    break;
                case PRE_TOTE:
                	SmartDashboard.putString("FSM_STATE", "FLOOR_PICKUP");
                	robot.elevator.setOnTargetThreshHold(40);
                	robot.elevator.setGoal(Constants.ELEVATOR_INDEX_PRE_TOTE);
                	stateComplete(FSM.PRE_TOTE);
                	break;
                case LOAD_TOTE:   //START LOAD TOTE SEQUENCE STEP 1
                	SmartDashboard.putString("FSM_STATE", "BOTTOM");
                	robot.elevator.setOnTargetThreshHold(25);
                	robot.elevator.setGoal(Constants.ELEVATOR_INDEX_LOADED);
                	setGoalState(FSM.LOAD_TOTE_WAITING);
                	break;
                case LOAD_TOTE_WAITING: //LOAD TOTE SEQUENCE STEP 2
                	SmartDashboard.putString("FSM_STATE", "LOAD_TOTE_WAITING");
                	if(robot.elevator.onTarget()){
                		robot.elevator.setOnTargetThreshHold(50);
                		robot.elevator.setGoal(Constants.ELEVATOR_INDEX_STATIONARY);
                		setGoalState(FSM.LOAD_TOTE_STATIONARY_WAITING);
                	}
                	break;
                case LOAD_TOTE_STATIONARY_WAITING: //LOAD TOTE SEQUENCE STEP 3
                	SmartDashboard.putString("FSM_STATE", "LOAD_TOTE_STAT"); //adriana says this makes sense. George says the opposite.
                	if(robot.elevator.onTarget()){
                		robot.intakeRollersStop();
                		setGoalState(FSM.PRE_TOTE);
                	}
                	break;
                case INTAKING_TOTE:
                	if(robot.elevator.toteOnBumper()){
                		setGoalState(FSM.LOAD_TOTE);
                		robot.openArm();
                	}else{
                		robot.closeArm();
                	}
                	break;
                case DEFAULT:
                	SmartDashboard.putString("FSM_STATE", "WAITING");
                	break;
            }
        }
    }
}