
package com.team1323.recyclerush2015;

import ControlSystem.FSM;
import ControlSystem.RoboSystem;
import IO.TeleController;
import SubSystems.Navigation;
import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.Timer;


public class Robot extends SampleRobot {
	public enum State{
		THREE_TOTE		
	}
	private RoboSystem robot;
	private TeleController controllers;
	private FSM fsm;
	private Navigation nav;
    public Robot() {
        robot = RoboSystem.getInstance();
        controllers = TeleController.getInstance();
        fsm = FSM.getInstance();
        nav.getInstance();
    }

    public void autonomous() {
    	State autonSelect = State.THREE_TOTE;
    	nav.resetRobotPosition(0, 0, 0, true);
    	
    	switch(autonSelect){
	    	case THREE_TOTE:
	    		//Step 1
	    		fsm.setGoalState(FSM.State.LOAD_TOTE);	
	    		
	    		//Step 2
	    		robot.closeArm();
	    		robot.elevator.closeTopStackHook();
	    		robot.intakeRollersReverse();
	    		
	    		//Step 3
	    		robot.dt.setHeading(135);
	    		
	    		//Step 4
	    		robot.elevator.openTopStackHook();
	    		
	    		//Step 5
	    		//Drive 18 in.
	    		
	    		//Step 6
	    		robot.dt.setHeading(90);
	    		fsm.setGoalState(FSM.State.WAITING_FOR_TOTE);
	    		
	    		//Step 7
	    		//Drive 6 in from tote
	    		
	    		//Step 8
	    		fsm.setGoalState(FSM.State.INTAKING_TOTE);
	    		
	    		//Step 9
	    		//drive 6 more in.
	    		
	    		//repeat 3-9
	    		//Step 3
	    		robot.dt.setHeading(135);
	    		
	    		//Step 4
	    		robot.elevator.openTopStackHook();
	    		
	    		//Step 5
	    		//Drive 18 in.
	    		
	    		//Step 6
	    		robot.dt.setHeading(90);
	    		fsm.setGoalState(FSM.State.WAITING_FOR_TOTE);
	    		
	    		//Step 7
	    		//Drive 6 in from tote
	    		
	    		//Step 8
	    		fsm.setGoalState(FSM.State.INTAKING_TOTE);
	    		
	    		//Step 9
	    		//drive 6 more in.
	    		
	    		//Step 10
	    		robot.dt.setHeading(0);
	    		
	    		//Step 11
	    		//drive back 4 ft
	    		
	    		//Step 12 
	    		fsm.setGoalState(FSM.State.DROP_TOTES);
	    		
	    		//Step 13
	    		//drive to cans
	    		
	    		
	    	
	    		
	    		break;
    	}
    }
   
    public void operatorControl() {
        while (isOperatorControl() && isEnabled()) {
        	controllers.update();            
            Timer.delay(0.1);		// wait for a motor update time
        }
    }

    public void test() {
    	
    }
}
