
package com.team1323.recyclerush2015;

import ControlSystem.FSM;
import ControlSystem.RoboSystem;
import IO.TeleController;
import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.Timer;


public class Robot extends SampleRobot {
    
	private RoboSystem robot;
	private TeleController controllers;
	private FSM fsm;
    public Robot() {
        robot = RoboSystem.getInstance();
        controllers = TeleController.getInstance();
        fsm = FSM.getInstance();
    }

    public void autonomous() {
    	
    }
   
    public void operatorControl() {
        while (isOperatorControl() && isEnabled()) {
        	controllers.update();            
            Timer.delay(0.05);		// wait for a motor update time
        }
    }

    public void test() {
    	
    }
}
