package IO;import ControlSystem.FSM;
import ControlSystem.RoboSystem;
import ControlSystem.FSM.State;
import SubSystems.Lifter;
import Utilities.Util;
import edu.wpi.first.wpilibj.Joystick.RumbleType;

/** Handles the input from an xbox controller in order to calculate what the
 *  forebar angles and claw state should be. It is designed to keep the logic for
 *  deciding what to do apart from the logic for how to do it
 *
 * @author Robotics
 */ 
public class TeleController
{
    public static final double STICK_DEAD_BAND = 0.2;

    private Xbox codriver,driver,driver2;
    private FSM fsm;
    private RoboSystem robot;
    private static TeleController instance = null;
    private boolean robotCentric = false;
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
        	fsm.clearLastTote();
        	fsm.setGoalState(FSM.State.WAITING_FOR_TOTE);        		
        }
        //////////////////////////////////////////
        if(codriver.bButton.isPressed()){
        	robot.lift.setState(Lifter.State.FORWARD_LOAD);
        }
        ////////////////////////////////////////
        if(codriver.xButton.isPressed()){
        	fsm.lastTote();
        	fsm.setGoalState(FSM.State.WAITING_FOR_TOTE);
        }
        ///////////////////////////////////////
        if(codriver.yButton.isPressed()){
        	switch(fsm.previousState()){
        	case RC_LOAD_WAITING:
        		fsm.setGoalState(State.RC_ARM_CLOSE);
        		break;
        	case RC_ARM_CLOSE:
        		fsm.setGoalState(State.RC_INTAKING);
        		break;
        	default:
        		fsm.setGoalState(FSM.State.RC_LOAD);
        		break;
        	}        	
        }
        /////////////////////////////////////////////

        if(codriver.rightTrigger.isPressed()){ 
        	fsm.nextState();
        }
        //////////////////////////////////
        if(codriver.rightBumper.isPressed()) {
        	robot.intakeRollersForward();
        	fsm.setGoalState(FSM.State.INTAKING_TOTE);
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
        	fsm.fsmStopState();
        	robot.lift.setState(Lifter.State.STOP);
        	robot.intakeRollersStop();
        }
        ////////////////////////////////////////////////////////
        if(codriver.startButton.isPressed()){
        	robot.elevator.resetManualToteCount();
        	fsm.fsmStopState();
        }
        ////////////////////////////////////////////////////////        
        if (codriver.getButtonAxis(Xbox.RIGHT_STICK_Y) > 0.2) {
        	robot.lift.setState(Lifter.State.MANUAL_DOWN);
        }else if(codriver.getButtonAxis(Xbox.RIGHT_STICK_Y) < -0.2){
        	robot.lift.setState(Lifter.State.MANUAL_UP);
        }else if(robot.lift.getState() == Lifter.State.MANUAL_DOWN || robot.lift.getState() == Lifter.State.MANUAL_UP){
        	robot.lift.setState(Lifter.State.STOP);
        }
        ///////////////////////////////////////////////
        if (codriver.getButtonAxis(Xbox.LEFT_STICK_Y) > 0.3) {
            robot.elevator.manualDown(Xbox.LEFT_STICK_Y);
        }else if( codriver.getButtonAxis(Xbox.LEFT_STICK_Y) < -0.3){
        	robot.elevator.manualUp(Xbox.LEFT_STICK_Y);
        }
        ///////////////////////////////////////////////
        if(codriver.leftCenterClick.isPressed()){
        	fsm.setGoalState(FSM.State.ZERO_ELEVATOR); //LOAD TOTE SEQUENCE
        }     
        ///////////////////////////////////////////////
        if(codriver.rightCenterClick.isPressed()) {
        	fsm.setGoalState(FSM.State.SCORING);
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
        	robot.dt.setHeading(-90);
        }else if(driver.yButton.isPressed()){
        	robot.dt.setHeading(0);
        }else{
        	robot.dt.sendInput(Util.controlSmoother(driver.getButtonAxis(Xbox.LEFT_STICK_X)), Util.controlSmoother(driver.getButtonAxis(Xbox.LEFT_STICK_Y)), Util.turnControlSmoother(driver.getButtonAxis(Xbox.RIGHT_STICK_X)),driver.leftTrigger.isHeld(),robotCentric,true);
//        	robot.dt.sendInputHeadingHold(Util.controlSmoother(driver.getButtonAxis(Xbox.LEFT_STICK_X)), Util.controlSmoother(driver.getButtonAxis(Xbox.LEFT_STICK_Y)),Util.turnControlSmoother(driver.getButtonAxis(Xbox.RIGHT_STICK_X)),driver.leftTrigger.isHeld());
        }        
    	
        //////////////////////////////////
        if(driver.rightTrigger.isPressed()) {
        	fsm.setGoalState(FSM.State.LOWER_TO_DROP_TOTES);
        }
        /////////////////////////////////////////////////////
        if(driver.leftBumper.isPressed()){
        	robotCentric = false;
        }
        ///////////////////////////////////////////////////////
        if(driver.rightBumper.isPressed()){
        	robotCentric = true;
        }
        
        //////////////////////////////////////////////////////
        if(driver.backButton.isHeld()){  // 
        	fsm.nav.resetRobotPosition(0, 0, 0, true);
        	robot.dt.heading.setGoal(0.0);
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
