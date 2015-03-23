
package com.team1323.recyclerush2015;

import ControlSystem.FSM;
import ControlSystem.RoboSystem;
import IO.TeleController;
import SubSystems.DriveTrain.Axis;
import SubSystems.Lifter;
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
    	State autonSelect = State.THREE_TOTE;
    	nav.resetRobotPosition(0, 0, 0, true);
    	robot.dt.heading.setGoal(-6);
    	double timeout = 0;
    	double timeout2;
    	switch(autonSelect){
    		case TOTE_GRABBER:
    			
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
    			robot.extendFollowerWheel();
    			System.out.println("1");
    			fsm.setGoalState(FSM.State.WAITING_FOR_TOTE);
	    		while(robot.lift.getState() != Lifter.State.WAITING){
	    			Timer.delay(0.01);
	    		}
	    		robot.intakeRollersReverse();
	    		Timer.delay(0.1);
	    		robot.dt.driveDistanceHoldingHeading(60.0,Axis.Y,0.0,false,3000,true);
	    		while(robot.dt.runDC() && isAutonomousKill()){
	    			Timer.delay(0.01);
	    		}	   
	    		/*
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
	    		fsm.setGoalState(FSM.State.PRE_TOTE);8*/
    		break;	
    		case NO_TOTE:
    			fsm.lastTote();
    			fsm.setGoalState(FSM.State.WAITING_FOR_TOTE);
    			Timer.delay(2);
    			while(robot.lift.getState() != Lifter.State.WAITING){
	    			Timer.delay(0.001);
	    		}
    			fsm.clearLastTote();
    		break;
	    	case THREE_TOTE:
	    		//Step 1
	    		robot.extendFollowerWheel();
	    		fsm.clearLastTote();
    			System.out.println("1");
    			fsm.setGoalState(FSM.State.WAITING_FOR_TOTE);
	    		while(robot.lift.getState() != Lifter.State.WAITING){
	    			Timer.delay(0.001);
	    		}
	    		robot.intakeRollersStop();
	    		Timer.delay(0.1);
	    		robot.dt.driveDistanceHoldingHeading(13.0,Axis.X,0.0,false,1000,false);
	    		while(robot.dt.runDC() && isAutonomousKill()){
	    			Timer.delay(0.001);
	    		}
	    		robot.intakeRollersReverse();
	    		robot.dt.driveDistanceHoldingHeading(35.0,Axis.Y,0.0,false,3000,false);
	    		while(robot.dt.runDC() && isAutonomousKill()){
	    			Timer.delay(0.001);
	    		}
	    		robot.dt.sendInput(0, 0, 0, true, false, true);
	    		robot.dt.driveDistanceHoldingHeading(-6.0,Axis.X,0.0,false,3000,false);
	    		while(robot.dt.runDC() && isAutonomousKill()){
	    			Timer.delay(0.001);
	    		}
	    		robot.dt.driveDistanceHoldingHeading(0.0,Axis.X,0.0,false,3000,false);
	    		while(robot.dt.runDC() && isAutonomousKill()){
	    			Timer.delay(0.001);
	    		}
	    		robot.dt.sendInput(0, 0, 0, true, false, true);
	    		fsm.setGoalState(FSM.State.WAITING_FOR_TOTE);	    		
	    		robot.dt.driveDistanceHoldingHeading(70.0,Axis.Y,0.0,false,3000,false);
	    		while(robot.dt.runDC() && isAutonomousKill()){
	    			Timer.delay(0.001);
	    		}
	    		Timer.delay(0.1);
	    		robot.dt.driveDistanceHoldingHeading(15.0,Axis.X,0.0,false,3000,false);
	    		while(robot.dt.runDC() && isAutonomousKill()){
	    			Timer.delay(0.001);
	    		}
	    		robot.dt.driveDistanceHoldingHeading(120.0,Axis.Y,0.0,false,3000,false);
	    		while(robot.dt.runDC() && isAutonomousKill()){	    			
	    			Timer.delay(0.01);
	    		}
	    		robot.dt.sendInput(0, 0, 0, true, false, true);
	    		robot.dt.driveDistanceHoldingHeading(-6.0,Axis.X,0.0,false,2500,false);
	    		while(robot.dt.runDC() && isAutonomousKill()){
	    			Timer.delay(0.001);
	    		}
	    		robot.dt.driveDistanceHoldingHeading(0.0,Axis.X,0.0,false,2000,false);
	    		while(robot.dt.runDC() && isAutonomousKill()){
	    			Timer.delay(0.001);
	    		}
	    		robot.dt.sendInput(0, 0, 0, true, false, true);
	    		fsm.lastTote();
	    		fsm.setGoalState(FSM.State.WAITING_FOR_TOTE);	    		
	    		robot.dt.driveDistanceHoldingHeading(168.0,Axis.Y,0.0,false,2000,false);
	    		while(robot.dt.runDC() && isAutonomousKill()){
	    			Timer.delay(0.001);
	    		}
	    		robot.dt.driveDistanceHoldingHeading(85.0,Axis.X,0.0,false,2000,false);
	    		while(robot.dt.runDC() && isAutonomousKill()){
	    			Timer.delay(0.001);
	    		}
	    		robot.dt.sendInput(0, 0, 0, true, false, true);
	    		fsm.setGoalState(FSM.State.LOWER_TO_DROP_TOTES);
	    		robot.dt.driveDistanceHoldingHeading(0.0,Axis.Y,0.0,false,3000,false);
	    		while(robot.dt.runDC() && isAutonomousKill()){
	    			Timer.delay(0.001);
	    		}
	    		robot.intakeRollersStop();
	    		fsm.clearLastTote();
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
