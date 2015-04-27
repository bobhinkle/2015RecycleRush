
package com.team1323.recyclerush2015;

import ControlSystem.FSM;
import ControlSystem.RoboSystem;
import IO.TeleController;
import SubSystems.DriveTrain.Axis;
import SubSystems.Lifter;
import SubSystems.Navigation;
import Utilities.Constants;
import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.Timer;


public class Robot extends SampleRobot {
	public enum State{
		THREE_TOTE,NO_TOTE,CAN_GRABBER
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
    	State autonSelect = State.CAN_GRABBER;
    	nav.resetRobotPosition(0, 0, 0, false);
    	switch(autonSelect){
    		case CAN_GRABBER:
    			robot.extendCanGrabber();
    			robot.dt.sendInput(0, 0.5, 0, false, true, true);
    			Timer.delay(.1);
    			robot.dt.sendInput(0, 0.0, 0, false, true, true);
    			Timer.delay(0.42);
    			robot.dt.sendInput(0, -1.0, 0, false, true, true);
    			Timer.delay(1.5);
    			robot.dt.sendInput(0, 0.0, 0, false, true, true);
    			break;
    		case NO_TOTE:
    			
    		break;
	    	case THREE_TOTE:
	    		//Step 1
	    		robot.extendFollowerWheel();
	    		fsm.clearLastTote();
    			System.out.println("1");
    			fsm.setGoalState(FSM.State.WAITING_FOR_TOTE);
	    		while(robot.lift.getState() != Lifter.State.FORWARD_LOAD){
	    			Timer.delay(0.01);
	    		}
	    		robot.intakeRollersStop();
	    		Timer.delay(0.1);
	    		robot.dt.distance.setMaxVelAccel(Constants.X_MOVE, Constants.X_MOVE_ACCEL);
	    		robot.dt.driveDistanceHoldingHeading(25.0,Axis.X,0.0,false,1000,false);
	    		while(robot.dt.runDC() && isAutonomousKill()){
	    			Timer.delay(0.01);
	    		}
	    		robot.intakeRollersReverse();
	    		robot.dt.distance.setMaxVelAccel(Constants.DIST_MAX_VEL, 320);
	    		robot.dt.driveDistanceHoldingHeading(35.0,Axis.Y,0.0,false,2000,false);
	    		while(robot.dt.runDC() && isAutonomousKill()){
	    			Timer.delay(0.01);
	    		}
	    		robot.dt.sendInput(0, 0, 0, true, false, true);
	    		robot.dt.distance.setMaxVelAccel(Constants.X_MOVE, Constants.X_MOVE_ACCEL);
	    		robot.dt.driveDistanceHoldingHeading(-4.0,Axis.X,-6.0,false,2000,false);
	    		while(robot.dt.runDC() && isAutonomousKill()){
	    			Timer.delay(0.01);
	    		}
	    		robot.dt.distance.setMaxVelAccel(Constants.DIST_MAX_VEL, 320);
	    		robot.intakeRollersForward();
	    		robot.dt.driveDistanceHoldingHeading(65.0,Axis.Y,0.0,false,2000,false);
	    		while(robot.dt.runDC() && isAutonomousKill()){
	    			Timer.delay(0.01);
	    		}
	    		Timer.delay(0.1);
	    		robot.dt.distance.setMaxVelAccel(Constants.X_MOVE, Constants.X_MOVE_ACCEL);
	    		robot.dt.driveDistanceHoldingHeading(25.0,Axis.X,-6.0,false,2000,false);
	    		while(robot.dt.runDC() && isAutonomousKill()){
	    			Timer.delay(0.01);
	    		}
	    		robot.dt.distance.setMaxVelAccel(Constants.DIST_MAX_VEL, 320);
	    		robot.dt.driveDistanceHoldingHeading(138.0,Axis.Y,0.0,false,2000,false);
	    		while(robot.dt.runDC() && isAutonomousKill()){	    			
	    			Timer.delay(0.01);
	    		}
	    		robot.dt.sendInput(0, 0, 0, true, false, true);
	    		robot.dt.distance.setMaxVelAccel(Constants.X_MOVE, Constants.X_MOVE_ACCEL);
	    		robot.dt.driveDistanceHoldingHeading(-3.0,Axis.X,0.0,false,1500,false);
	    		while(robot.dt.runDC() && isAutonomousKill()){
	    			Timer.delay(0.01);
	    		}
	    		robot.dt.distance.setMaxVelAccel(Constants.DIST_MAX_VEL, 320);
	    		fsm.lastTote();
	    		robot.intakeRollersForward();	    		
	    		robot.dt.driveDistanceHoldingHeading(150.0,Axis.Y,0.0,false,2000,false);
	    		while(robot.dt.runDC() && isAutonomousKill()){
	    			Timer.delay(0.01);
	    		}
	    		robot.dt.distance.setMaxVelAccel(4800, 320);
	    		robot.dt.driveDistanceHoldingHeading(100.0,Axis.X,0.0,false,3000,false);
	    		while(robot.dt.runDC() && isAutonomousKill()){
	    			Timer.delay(0.01);
	    		}
	    		robot.intakeRollersStop();
	    		robot.dt.sendInput(0, 0, 0, true, false, true);
//	    		nav.resetRobotPosition(0, 0, 0, true);
	    		/*fsm.setGoalState(FSM.State.LOWER_TO_DROP_TOTES);
	    		
	    		robot.dt.driveDistanceHoldingHeading(-30.0,Axis.Y,0.0,false,2500,false);
	    		while(robot.dt.runDC() && isAutonomousKill()){
	    			Timer.delay(0.01);
	    		}*/
	    		fsm.setGoalState(FSM.State.LOWER_TO_DROP_TOTES);
	    		double timerrout = System.currentTimeMillis() + 3000;
	    		while(isAutonomous() && timerrout > System.currentTimeMillis()){
	    			robot.dt.sendInput(0.0, 0.5, 0, false, false,false);
	    			Timer.delay(0.01);
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
    	robot.intakeRollersStop();
//    	robot.retractCanGrabber();
    	fsm.fsmStopState();
    	robot.retarctFollowerWheel();
        while (isOperatorControl() && isEnabled()) {        	
        	controllers.update();            
            Timer.delay(0.1);		// wait for a motor update time
        }
    }

    public void test() {
    	
    }
}
