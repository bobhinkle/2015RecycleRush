
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
        nav = nav.getInstance();
    }

    public void autonomous() {
    	State autonSelect = State.THREE_TOTE;
    	nav.resetRobotPosition(0, 0, 0, false);
    	robot.dt.heading.setGoal(0);
    	robot.dt.distance.setGoal(18);
		System.out.println("Driving");
		while(!robot.dt.distance.onTarget()){
			System.out.println("distance: " + nav.getY());
			Timer.delay(0.01);
		}
    	switch(autonSelect){
	    	case THREE_TOTE:
	    		int runs = 2;
	    		//Step 1
//	    		fsm.setGoalState(FSM.State.LOAD_TOTE);	
//	    		Timer.delay(2);
	    		//Step 2
//	    		robot.closeArm();
//	    		robot.elevator.closeTopStackHook();
//	    		robot.intakeRollersReverse();
/*	    		while(runs > 0){
		    		//Step 3
//		    		robot.dt.setHeading(135);
//		    		Timer.delay(1);
		    		//Step 4
//		    		robot.elevator.openTopStackHook();
		    		
		    		//Step 5
		    		robot.dt.distance.setGoal(18);
		    		System.out.println("Driving");
		    		while(!robot.dt.distance.onTarget()){
		    			System.out.println("distance: " + nav.getY());
		    			Timer.delay(0.01);
		    		}
		    		//Step 6
//		    		robot.dt.setHeading(90);
		    		fsm.setGoalState(FSM.State.WAITING_FOR_TOTE);
		    		
		    		//Step 7
		    		robot.dt.distance.setGoal(18);
		    		while(!robot.dt.distance.onTarget()){
		    			Timer.delay(0.01);
		    		}
		    		
		    		//Step 8
		    		fsm.setGoalState(FSM.State.INTAKING_TOTE);
		    		
		    		//Step 9
		    		robot.dt.distance.setGoal(6);
		    		while(!robot.dt.distance.onTarget()){
		    			Timer.delay(0.01);
		    		}
	    		runs--;
	    		}*/
	    		
	    	   	//Step 10
//	    		robot.dt.setHeading(0);
	    		
	    		//Step 11
//	    		robot.dt.distance.setGoal(-48);
//	    		while(!robot.dt.distance.onTarget()){
//	    			Timer.delay(0.01);
//	    		}
	    		
	    		//Step 12 
//	    		fsm.setGoalState(FSM.State.DROP_TOTES);
//	    		Timer.delay(1);
	    		//Step 13
//	    		robot.dt.distance.setGoal(-18);
//	    		while(!robot.dt.distance.onTarget()){
//	    			Timer.delay(0.01);
//	    		}
	    		break;
    	}
    }
   
    public void operatorControl() {
        while (isOperatorControl() && isEnabled()) {
        	robot.retarctFollowerWheel();
        	controllers.update();            
            Timer.delay(0.1);		// wait for a motor update time
        }
    }

    public void test() {
    	
    }
}
