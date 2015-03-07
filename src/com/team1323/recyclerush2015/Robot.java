
package com.team1323.recyclerush2015;

import ControlSystem.FSM;
import ControlSystem.RoboSystem;
import IO.TeleController;
import SubSystems.DriveTrain.Axis;
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
    	nav.resetRobotPosition(0, 0, 0, true);
    	robot.dt.heading.setGoal(-6);
    	double timeout = 0;
    	switch(autonSelect){
	    	case THREE_TOTE:
	    		//Step 1
	    		fsm.setGoalState(FSM.State.PRE_TOTE);
	    		System.out.println("Step 1");
	    		robot.dt.driveDistanceHoldingHeading(-12.0,Axis.Y,20.0,false);
	    		while(!robot.dt.distance.onTarget()){
	    			Timer.delay(0.01);
	    		}	    		
	    		System.out.println("Step 2");
	    		robot.dt.driveDistanceHoldingHeading(20.0,Axis.BOTH,0.0,false);
	    		while(!robot.dt.distance.onTarget()){
	    			Timer.delay(0.01);
	    		}
	    		System.out.println("Step 4");
	    		while(nav.getX() > 0 && isAutonomous()){
	    			robot.dt.sendInput(-1.0, -0.35, 0.0, false, false,false);
	    			Timer.delay(0.01);
	    		}	    		
	    		robot.dt.sendInput(0.0, 0.0, 0.0, false, false,false);
	    		System.out.println("drive to tote");
	    		fsm.nextState();
	    		robot.intakeRollersForward();
	    		robot.dt.driveDistanceHoldingHeading(20.0,Axis.Y,0.0,false);
	    		System.out.println("Step 5");
	    		while(!robot.dt.distance.onTarget()){	
	    			robot.intakeRollersForward();
	    			Timer.delay(0.01);
	    		}	        					
				System.out.println("Step 3.8"); //last tote ready
				fsm.nextState(); // close arms intake tote
				double timeout3 = System.currentTimeMillis() + 1500;
				while(timeout3 > System.currentTimeMillis() && isAutonomous() && !robot.elevator.toteOnBumper()){
					Timer.delay(0.01);
				}
				System.out.println("Step 4"); //drive past can 2
	    		robot.dt.driveDistanceHoldingHeading(20.0,Axis.BOTH,0.0,false);
	    		while(!robot.dt.distance.onTarget() && isAutonomous() ){
	    			Timer.delay(0.01);
	    		}
	    		robot.dt.heading.setGoal(0);
	    		robot.dt.driveDistanceHoldingHeading(10.0,Axis.Y,0.0,false);
	    		timeout = System.currentTimeMillis() + 2000;
	    		while(!robot.dt.distance.onTarget() && isAutonomous()){
	    			Timer.delay(0.01);
	    		}
	    		while(nav.getX() > 8  && isAutonomous()){
	    			robot.dt.sendInput(-1.0, -0.25, 0.0, false, false,false);
	    			Timer.delay(0.01);
	    		}
	    		robot.dt.sendInput(0.0, 0.0, 0.0, false, false,false);
	    		robot.intakeRollersForward();
	    		System.out.println("Step 4.5"); //drive past can 2
	    		robot.dt.driveDistanceHoldingHeading(36.0,Axis.Y,0.0,false);
	    		timeout = System.currentTimeMillis() + 2000;
	    		while(!robot.dt.distance.onTarget() && isAutonomous()){
	    			Timer.delay(0.01);
	    		}
	    		robot.intakeRollersForward();
	    		System.out.println("Step 7"); //last tote ready
	    		fsm.nextState(); //at last tote
	    		Timer.delay(0.25);
	    		robot.intakeRollersForward();
	    		System.out.println("Step 7.5"); //last tote ready
	    		fsm.nextState(); //close arms
	    		Timer.delay(1.0);
	    		System.out.println("Step 7.6"); //last tote ready
	    		Timer.delay(0.5);
				System.out.println("Step 8");
				double timeout2 = System.currentTimeMillis() + 500;
	    		while(fsm.previousState() != FSM.State.PRE_TOTE && isAutonomous() && (timeout2 > System.currentTimeMillis())){
	    			Timer.delay(0.01);
	    		}
	    		
	    		System.out.println("Step 9"); //last tote ready
	    		robot.dt.distance.setAxis(Axis.X);
	    		timeout = System.currentTimeMillis() + 2000;
	    		boolean setDrop = true;
	    		while((timeout > System.currentTimeMillis()) && isAutonomous() && nav.getX() < 120){
	    			robot.dt.sendInput(1.0, 0, 0, false, true,false);
	    			Timer.delay(0.01);
	    		}
	    		robot.dt.sendInput(0.0, 0, 0, true, true,false);
	    		System.out.println("Step 10");
	    		fsm.setGoalState(FSM.State.LOWER_TO_DROP_TOTES);
	    		Timer.delay(0.25);
	    		System.out.println("Step 11");
	    		timeout = System.currentTimeMillis() + 750;
	    		while((timeout > System.currentTimeMillis()) && isAutonomous()){
	    			robot.dt.sendInput(0.0, 0.7, 0, true, true,false);
	    			Timer.delay(0.01);
	    		}
	    		robot.dt.sendInput(0.0, 0, 0, true, true,false);
	    		System.out.println("Step 13");
	    		robot.intakeRollersStop();
	    		break;
    	}
    }
   
    public void operatorControl() {
    	robot.dt.heading.setGoal(nav.getRawHeading());
        while (isOperatorControl() && isEnabled()) {
        	robot.retarctFollowerWheel();
        	controllers.update();            
            Timer.delay(0.1);		// wait for a motor update time
        }
    }

    public void test() {
    	
    }
}
