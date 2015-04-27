package IO;import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import ControlSystem.FSM;
import ControlSystem.FSM.State;
import ControlSystem.RoboSystem;
import SubSystems.Lifter;
import Utilities.Constants;
import Utilities.Util;

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
        	robot.retractCanGrabber();
        }
        ////////////////////////////////////////
        if(codriver.xButton.isPressed()){
        	fsm.lastTote();
        	fsm.setGoalState(FSM.State.WAITING_FOR_TOTE);
        }
        ///////////////////////////////////////
        if(codriver.yButton.isPressed()){
        	fsm.setGoalState(FSM.State.RC_LOAD);   	
        }
        /////////////////////////////////////////////

        if(codriver.rightTrigger.isPressed()){ 
        	fsm.setGoalState(FSM.State.RC_TOP); 
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
        	robot.intakeRollersStop();
        }
        ////////////////////////////////////////////////////////
        if(codriver.startButton.isPressed()){
        	fsm.fsmStopState();
        	robot.lift.setState(Lifter.State.STOP);
        	robot.canLift.rawPower(0);
        }
        ////////////////////////////////////////////////////////        
        if (codriver.getButtonAxis(Xbox.RIGHT_STICK_Y) > 0.15) {
        	robot.lift.setState(Lifter.State.MANUAL_DOWN);
        }else if(codriver.getButtonAxis(Xbox.RIGHT_STICK_Y) < -0.2){
        	robot.lift.setState(Lifter.State.MANUAL_UP);
        }else if(robot.lift.getState() == Lifter.State.MANUAL_DOWN || robot.lift.getState() == Lifter.State.MANUAL_UP){
        	robot.lift.setState(Lifter.State.STOP);
        }
        ///////////////////////////////////////////////
        if (codriver.getButtonAxis(Xbox.LEFT_STICK_Y) > 0.3) {
        	
        }else if( codriver.getButtonAxis(Xbox.LEFT_STICK_Y) < -0.3){
        	
        }else{
        	
        }
        ///////////////////////////////////////////////
        if(codriver.leftCenterClick.isPressed()){
        	robot.extendCanGrabber();
        }     
        ///////////////////////////////////////////////
        if(codriver.rightCenterClick.isPressed()) {
        	robot.canLift.cycleClapper();
        }
        if(codriver.getPOV() == 0){
        	robot.canLift.manualUp(10);
        }
        if(codriver.getPOV() ==180){
        	robot.canLift.manualDown(10);
        }
    }
    
    public void driver() {
    	
    	if(driver.aButton.isPressed()){       
    		robot.dt.setHeading(180);
        }else if(driver.bButton.isPressed()){  
        	robot.dt.setHeading(20);
        }else if(driver.xButton.isPressed()){
        	robot.dt.setHeading(-20);
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
        if(robotCentric)
        	SmartDashboard.putString("RobotControl", "ROBOT");
        else
            SmartDashboard.putString("RobotControl","FIELD");
    }
    public void update(){
    	codriver.run();
    	driver.run();
    	coDriver();
    	driver();
    }
    
}
