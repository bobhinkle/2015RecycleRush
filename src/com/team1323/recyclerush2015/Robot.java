
package com.team1323.recyclerush2015;

import ControlSystem.FSM;
import ControlSystem.RoboSystem;
import IO.TeleController;
import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Robot extends SampleRobot {
    
	private RoboSystem robot;
	private TeleController controllers;
	private FSM fsm;
    public Robot() {
        robot = RoboSystem.getInstance();
        controllers = TeleController.getInstance();
        fsm.getInstance();
    }

    /**
     * Drive left & right motors for 2 seconds then stop
     */
    public void autonomous() {
    	
    }
    

    /**
     * Runs the motors with arcade steering.
     */
    public void operatorControl() {
        while (isOperatorControl() && isEnabled()) {

            double current = robot.pdp.getCurrent(14);
            SmartDashboard.putNumber("PDP0", current);
            Timer.delay(0.005);		// wait for a motor update time
        }
    }

    /**
     * Runs during test mode
     */
    public void test() {
    	
    }
}
