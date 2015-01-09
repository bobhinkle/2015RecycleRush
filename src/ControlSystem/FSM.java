package ControlSystem;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class FSM {

	private RoboSystem robot;
	private static FSM instance = null;

	private int currentState = INIT;
    private int goalState = DEFAULT;
    
	public final static int INIT = 0;
	public final static int DEFAULT = -1;
	
	
	public static FSM getInstance()
    {
        if( instance == null )
            instance = new FSM();
        return instance;
    }
        
    public FSM() {
        robot = RoboSystem.getInstance();
    }	
    public void setGoalState(int goal) {
        if(currentState == goal){
            currentState = FSM.DEFAULT;
            goalState = goal;
        }else{
            goalState = goal;
        }
        run();
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
    	
    
    public void run(){
        if(checkStateChange()){
            switch(goalState){
                case INIT:
                    SmartDashboard.putString("FSM_STATE", "INIT");
                    stateComplete(FSM.INIT);
                    break;
            }
        }
    }
}