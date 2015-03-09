
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
		THREE_TOTE,NO_TOTE,THREE_TOTE_STRAIGHT,
		LAST_TOTE_STRAFE, TOTE_GRABBER
	}
	private RoboSystem robot;
	private TeleController controllers;
	private FSM fsm;
	private Navigation nav;
    public Robot() {
        robot = RoboSystem.getInstance();
        controllers = TeleController.getInstance();
        fsm = FSM.getInstance();
        nav = Navigation.getInstance();
    }

    public void autonomous() {
    	State autonSelect = State.TOTE_GRABBER;
    	nav.resetRobotPosition(0, 0, 0, true);
    	robot.dt.heading.setGoal(-6);
    	double timeout = 0;
    	double timeout2;
    	switch(autonSelect){
    		case TOTE_GRABBER:
    			robot.extendFollowerWheel();
    			timeout = System.currentTimeMillis() + 1000;
    			while(isAutonomous() & timeout > System.currentTimeMillis()){
    				robot.toteGrabber(0.5);
    			}
    			robot.toteGrabber(0);
/*	    		timeout = System.currentTimeMillis() + 2000;
    			while(isAutonomous() & timeout > System.currentTimeMillis()){
    				robot.toteGrabber(-0.5);
    			}
    			robot.toteGrabber(0);*/
    		break;
    		case LAST_TOTE_STRAFE:
    			if(isAutonomous()){
    				fsm.setGoalState(FSM.State.PRE_TOTE);
	    		}
				timeout = System.currentTimeMillis() + 2000;
				timeout2 = System.currentTimeMillis() + 500;
	    		while(fsm.previousState() != FSM.State.PRE_TOTE && isAutonomous() && (timeout2 > System.currentTimeMillis()) && (timeout > System.currentTimeMillis())){
	    			Timer.delay(0.01);
	    		}
	    		
	    		System.out.println("Step 9"); //last tote ready
	    		robot.dt.distance.setAxis(Axis.X);
	    		timeout = System.currentTimeMillis() + 2000;
	    		while((timeout > System.currentTimeMillis()) && isAutonomous() && nav.getX() < 120){
	    			robot.dt.sendInput(1.0, 0, 0, false, true,false);
	    			Timer.delay(0.01);
	    		}
	    		robot.dt.sendInput(0.0, 0, 0, true, true,false);
	    		System.out.println("Step 10");
	    		fsm.setGoalState(FSM.State.LOWER_TO_DROP_TOTES);
	    		Timer.delay(0.75);
	    		System.out.println("Step 11");
	    		timeout = System.currentTimeMillis() + 750;
	    		while((timeout > System.currentTimeMillis()) && isAutonomous()){
	    			robot.dt.sendInput(0.0, 0.7, 0, true, true,false);
	    			Timer.delay(0.01);
	    		}
	    		robot.dt.killDistanceController();
	    		robot.dt.sendInput(0.0, 0, 0, true, true,false);
	    		System.out.println("Step 13");
	    		robot.intakeRollersStop();
    		break;
    		case THREE_TOTE_STRAIGHT:
    			fsm.toPreToteArmOpen(true);
    			robot.extendFollowerWheel();
    			Timer.delay(0.5);
    			robot.extendFollowerWheel();
    			System.out.println("3");
    			fsm.nextState();
    			Timer.delay(0.5);
    			fsm.nextState();
    			Timer.delay(0.5);
    			System.out.println("4");
    			fsm.nextState();
	    		while(fsm.previousState() != FSM.State.PRE_TOTE_ARM_OPEN){
	    			Timer.delay(0.01);
	    		}
	    		robot.elevator.openTopStackHook();
	    		robot.closeArm();
	    		robot.intakeRollersReverse();
	    		Timer.delay(0.5);
	    		robot.dt.driveDistanceHoldingHeading(60.0,Axis.Y,0.0,false,3000);
	    		while(robot.dt.runDC() && isAutonomousKill()){
	    			Timer.delay(0.01);
	    		}	   
	    		fsm.setGoalState(FSM.State.PRE_TOTE_ARM_OPEN);
	    		Timer.delay(1);
    			System.out.println("3");
    			System.out.println("3");
    			fsm.nextState();
    			Timer.delay(0.5);
    			fsm.nextState();
    			Timer.delay(0.5);
    			System.out.println("4");
    			fsm.nextState();
	    		while(fsm.previousState() != FSM.State.PRE_TOTE_ARM_OPEN){
	    			Timer.delay(0.01);
	    		}
	    		robot.elevator.openTopStackHook();
	    		robot.closeArm();
	    		robot.intakeRollersReverse();
	    		Timer.delay(0.5);
    			robot.dt.driveDistanceHoldingHeading(40.0,Axis.Y,0.0,false,2000);
	    		while(robot.dt.runDC() && isAutonomousKill()){
	    			Timer.delay(0.01);
	    		}	   
	    		fsm.setGoalState(FSM.State.PRE_TOTE_ARM_OPEN);
	    		Timer.delay(1);
    			System.out.println("5");
    			fsm.nextState();
    			Timer.delay(1);
    			System.out.println("6");
    			fsm.nextState();
    			Timer.delay(1);
    			robot.closeArm();
	    		robot.intakeRollersReverse();
    			robot.dt.driveDistanceHoldingHeading(40.0,Axis.Y,0.0,false,2000);
	    		while(robot.dt.runDC() && isAutonomousKill()){
	    			Timer.delay(0.01);
	    		}	   
	    		fsm.setGoalState(FSM.State.PRE_TOTE);
    		break;	
    		case NO_TOTE:
    //			robot.extendFollowerWheel();
    		break;
	    	case THREE_TOTE:
	    		//Step 1
	    		fsm.setGoalState(FSM.State.PRE_TOTE);
	    		System.out.println("Step 1");
	    		robot.dt.driveDistanceHoldingHeading(-12.0,Axis.Y,20.0,false,2000);
	    		while(robot.dt.runDC() && isAutonomousKill()){
	    			Timer.delay(0.01);
	    		}	    		
	    		System.out.println("Step 2");
	    		robot.dt.driveDistanceHoldingHeading(20.0,Axis.BOTH,0.0,false,2000);
	    		while(robot.dt.runDC() && isAutonomousKill()){
	    			Timer.delay(0.01);
	    		}	
	    		System.out.println("Step 4");
	    		timeout = System.currentTimeMillis() + 2000;
	    		while(nav.getX() > 0 && isAutonomous()&& (timeout > System.currentTimeMillis())){
	    			robot.dt.sendInput(-1.0, -0.35, 0.0, false, false,false);
	    			Timer.delay(0.01);
	    		}	    		
	    		robot.dt.sendInput(0.0, 0.0, 0.0, false, false,false);
	    		System.out.println("drive to tote");
	    		fsm.nextState();
	    		robot.intakeRollersForward();
	    		robot.dt.driveDistanceHoldingHeading(20.0,Axis.Y,0.0,false,2000);
	    		System.out.println("Step 5");
	    		timeout = System.currentTimeMillis() + 2000;
	    		while(robot.dt.runDC() && isAutonomousKill()){
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
	    		robot.dt.driveDistanceHoldingHeading(20.0,Axis.BOTH,0.0,false,1500);
				while(robot.dt.runDC() && isAutonomousKill()){
	    			Timer.delay(0.01);
	    		}	
	    		robot.dt.heading.setGoal(0);
	    		robot.dt.driveDistanceHoldingHeading(10.0,Axis.Y,0.0,false,2000);
	    		while(robot.dt.runDC() && isAutonomousKill()){
	    			Timer.delay(0.01);
	    		}	
	    		timeout = System.currentTimeMillis() + 2000;
	    		while(nav.getX() > 8  && isAutonomous()  && (timeout > System.currentTimeMillis())){
	    			robot.dt.sendInput(-1.0, -0.25, 0.0, false, false,false);
	    			Timer.delay(0.01);
	    		}
	    		robot.dt.sendInput(0.0, 0.0, 0.0, false, false,false);
	    		robot.intakeRollersForward();
	    		System.out.println("Step 4.5"); //drive past can 2
	    		robot.dt.driveDistanceHoldingHeading(36.0,Axis.Y,0.0,false,2000);
	    		while(robot.dt.runDC() && isAutonomousKill()){
	    			Timer.delay(0.01);
	    		}	
	    		if(isAutonomous()){
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
	    		}
				timeout = System.currentTimeMillis() + 2000;
				timeout2 = System.currentTimeMillis() + 500;
	    		while(fsm.previousState() != FSM.State.PRE_TOTE && isAutonomous() && (timeout2 > System.currentTimeMillis()) && (timeout > System.currentTimeMillis())){
	    			Timer.delay(0.01);
	    		}
	    		
	    		System.out.println("Step 9"); //last tote ready
	    		robot.dt.distance.setAxis(Axis.X);
	    		timeout = System.currentTimeMillis() + 2000;
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
	    		robot.dt.killDistanceController();
	    		robot.dt.sendInput(0.0, 0, 0, true, true,false);
	    		System.out.println("Step 13");
	    		robot.intakeRollersStop();
	    		break;
    	}
    }
    public boolean isAutonomousKill(){
    	if(isAutonomous()){    		
    		return true;
    	}else{
    		robot.dt.killDistanceController();
    		return false;
    	}
    }
    public void operatorControl() {
    	fsm.toPreToteArmOpen(false);
    	robot.intakeRollersStop();
    	fsm.fsmStopState();
        while (isOperatorControl() && isEnabled()) {
        	robot.retarctFollowerWheel();
        	controllers.update();            
            Timer.delay(0.1);		// wait for a motor update time
        }
    }

    public void test() {
    	
    }
}
