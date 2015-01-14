package ControlSystem;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class FSM {

	private RoboSystem robot;
	private static FSM instance = null;
	private partsUpdate pu;
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
    	
    private class partsUpdate extends Thread{
        private boolean keepRunning = true;
    	public void run(){
    		System.out.println("Started");
    		while(keepRunning){
    			SmartDashboard.putString("AHRS", robot.ahrs.getData());
    			update();
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
                case DEFAULT:
                	
                	break;
            }
        }
    }
}