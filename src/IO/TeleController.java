package IO;import Utilities.Util;
import edu.wpi.first.wpilibj.Joystick.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import ControlSystem.FSM;
import ControlSystem.RoboSystem;

/** Handles the input from an xbox controller in order to calculate what the
 *  forebar angles and claw state should be. It is designed to keep the logic for
 *  deciding what to do apart from the logic for how to do it
 *
 * @author Robotics
 */ 
public class TeleController
{
    public static final double STICK_DEAD_BAND = 0.2;

    private Xbox codriver,driver;
    private FSM fsm;
    private RoboSystem robot;
    private static TeleController instance = null;
    public TeleController(){
        driver = new Xbox(0);
        driver.init();
        codriver  = new Xbox(1);
        codriver.init();
        robot = RoboSystem.getInstance();
        fsm = FSM.getInstance();
        System.out.println("CONTROLS STARTED");
    }
    public static TeleController getInstance(){
        if(instance == null){
            instance = new TeleController();
        }
        return instance;
    }    
    public void coDriver(){
        if(codriver.aButton.isPressed()){
    		switch(fsm.previousState()){
    		case RC_WAITING_ON_LINE_BREAK:
    			fsm.setGoalState(FSM.State.RC_INTAKING);
    			break;
    		case INTAKING_TOTE:
    			fsm.setGoalState(FSM.State.LOAD_TOTE);
    			break;
    		case WAITING_FOR_TOTE:
    			fsm.setGoalState(FSM.State.INTAKING_TOTE);
    			break;
    		case PRE_TOTE:
    			fsm.setGoalState(FSM.State.WAITING_FOR_TOTE);
    			break;
    		case TOTE_DROP_WAIT_FOR_COMPLETION:
    			fsm.setGoalState(FSM.State.TOTE_DROP_RESET);
    			break;
    		default:
    			fsm.setGoalState(FSM.State.PRE_TOTE);
    			break;
    		}
        }
        //////////////////////////////////////////
        if(codriver.bButton.isPressed()){
        	fsm.setGoalState(FSM.State.PRE_TOTE);
        }
        ////////////////////////////////////////
        if(codriver.xButton.isPressed()){
        	fsm.bypassState();
        }
        ///////////////////////////////////////
        if(codriver.yButton.isPressed()){
           robot.elevator.resetToteCounter();
        }
        /////////////////////////////////////////////

        if(codriver.rightTrigger.isPressed()){ 
        	
        }
        //////////////////////////////////
        if(codriver.rightBumper.isPressed()) {
        	robot.intakeRollersForward();
        }
        ///////////////////////////////////////////////////////
        if(codriver.leftTrigger.isPressed()){
        	robot.actuateArm();
        }
        //////////////////////////////////////////////////////////////////// 
        if(codriver.leftBumper.isPressed()){ 
        	robot.intakeRollersReverse();
        }
        //////////////////////////////////////////////////////
        if(codriver.backButton.isPressed()){  // stop all 
        	robot.elevator.resetManualToteCount();
        	fsm.fsmStopState();
        	robot.intakeRollersStop();
        	driver.setRumble(RumbleType.kLeftRumble, (float)0.0);
        }
        ////////////////////////////////////////////////////////
        if(codriver.startButton.isPressed()){
        	fsm.setGoalState(FSM.State.INTAKING_TOTE);
        }
        ////////////////////////////////////////////////////////        
        if (codriver.getButtonAxis(Xbox.RIGHT_STICK_Y) > 0 || codriver.getButtonAxis(Xbox.RIGHT_STICK_Y) < 0) {
            
        }
        ///////////////////////////////////////////////
        if (codriver.getButtonAxis(Xbox.LEFT_STICK_Y) > 0 || codriver.getButtonAxis(Xbox.LEFT_STICK_Y) < 0) {
            
        } 
        ///////////////////////////////////////////////
        if(codriver.leftCenterClick.isPressed()){
        	fsm.setGoalState(FSM.State.ZERO_ELEVATOR); //LOAD TOTE SEQUENCE
        }     
        ///////////////////////////////////////////////
        if(codriver.rightCenterClick.isPressed()) {
        	
        }
        if(codriver.getPOV() == 0){
        	robot.elevator.increaseManualToteCount();
        	fsm.setGoalState(FSM.State.MANUAL_TOTE);
        }
        if(codriver.getPOV() ==180){
        	robot.elevator.decreaseManualToteCount();
        	fsm.setGoalState(FSM.State.MANUAL_TOTE);
        }
        if((robot.elevator.lineBreakTrigger() || codriver.getPOV() == 90) && fsm.getCurrentState() == FSM.State.RC_LOAD_WAITING){
        	fsm.setGoalState(FSM.State.RC_INTAKING);
        }
    }
    
    public void driver() {
    	
    	if(driver.aButton.isPressed()){       
    		robot.dt.setHeading(180);
        }else if(driver.bButton.isPressed()){  
        	robot.dt.setHeading(90);
        }else if(driver.xButton.isPressed()){
        	robot.dt.setHeading(270);
        }else if(driver.yButton.isPressed()){
        	robot.dt.setHeading(0);
        }else{
        	robot.dt.sendInput(Util.deadBand(driver.getButtonAxis(Xbox.LEFT_STICK_X),0.15), Util.deadBand(driver.getButtonAxis(Xbox.LEFT_STICK_Y),0.15), Util.deadBand(driver.getButtonAxis(Xbox.RIGHT_STICK_X), 0.2),driver.leftTrigger.isHeld());
        }        
    	
    	if(driver.aButton.isPressed()){
    		
    	}
    	if(driver.bButton.isPressed()){
    		
    	}
        /////////////////////////////////////////////
    	
        if(driver.leftTrigger.isPressed()){ 
        	
        }
        //////////////////////////////////
        if(driver.rightTrigger.isPressed()) {
        	fsm.setGoalState(FSM.State.LOWER_TO_DROP_TOTES);
        }
        ///////////////////////////////////////////////////////
        if(driver.rightBumper.isPressed()){
        	
        }
        //////////////////////////////////////////////////////////////////// 
        if(driver.leftBumper.isPressed()){ //reverse rollers
            
        }
        //////////////////////////////////////////////////////
        if(driver.backButton.isPressed()){  // stop all 
        	fsm.nav.resetRobotPosition(0, 0, 0, true);
        }
        ////////////////////////////////////////////////////////
        if(driver.startButton.isPressed()){
        	
        }
        if(driver.leftCenterClick.isPressed()){
        	
        }
        if(driver.rightCenterClick.isPressed()){
        	
        }
        if(driver.getPOV() == 0){
        	
        }
        
    }
    public void update(){
    	codriver.run();
    	driver.run();
    	coDriver();
    	driver();
    }
    
}
