package IO;import ControlSystem.FSM;
import ControlSystem.RoboSystem;

/** Handles the input from an xbox controller in order to calculate what the
 *  forebar angles and claw state should be. It is designed to keep the logic for
 *  deciding what to do apart from the logic for how to do it
 *
 * @author Robotics
 */ 
public class TeleController
{
    public static final double STICK_DEAD_BAND = 0.1;

    private Xbox codriver,driver;
    private FSM fsm;
    private RoboSystem robot;
    private static TeleController instance = null;
    public TeleController(){
        driver = new Xbox(0);
        codriver  = new Xbox(1);
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
        	
        }
        //////////////////////////////////////////
        if(codriver.bButton.isPressed()){
        	
        }
        ////////////////////////////////////////
        if(codriver.xButton.isPressed()){
        	
        }
        ///////////////////////////////////////
        if(codriver.yButton.isPressed()){
           
        }
        /////////////////////////////////////////////

        if(codriver.rightTrigger.isPressed()){ 
            
        }
        //////////////////////////////////
        if(codriver.rightBumper.isPressed()) {
           
        }
        ///////////////////////////////////////////////////////
        if(codriver.leftTrigger.isPressed()){
        	
        }
        //////////////////////////////////////////////////////////////////// 
        if(codriver.leftBumper.isPressed()){ 
            
        }
        //////////////////////////////////////////////////////
        if(codriver.backButton.isPressed()){  // stop all 
          
        }
        ////////////////////////////////////////////////////////
        if(codriver.startButton.isPressed()){
        	
        }
        ////////////////////////////////////////////////////////        
        if (codriver.getButtonAxis(Xbox.RIGHT_STICK_Y) > 0 || codriver.getButtonAxis(Xbox.RIGHT_STICK_Y) < 0) {
            
        }
        ///////////////////////////////////////////////
        if (codriver.getButtonAxis(Xbox.LEFT_STICK_Y) > 0 || codriver.getButtonAxis(Xbox.LEFT_STICK_Y) < 0) {
            
        } 
        ///////////////////////////////////////////////
        if(codriver.leftCenterClick.isPressed()){
            
        }     
        ///////////////////////////////////////////////
        if(codriver.rightCenterClick.isPressed()) {
            
        }
        if(codriver.getButtonAxis(Xbox.DPADX) > 0 || codriver.getButtonAxis(Xbox.DPADX) < 0){
          
        }
    }
    
    public void driver() {

    	if(driver.aButton.isPressed()){        	
    		robot.dt.sendInput(0.0,-1.0,0.0);
        }else if(driver.bButton.isPressed()){       	
        	robot.dt.sendInput(1.0,0.0,0.0);
        }else if(driver.xButton.isPressed()){
        	robot.dt.sendInput(-1.0,0.0,0.0);
        }else if(driver.yButton.isPressed()){
        	robot.dt.sendInput(0.0,1.0,0.0);
        }else{
        	robot.dt.sendInput(driver.getButtonAxis(Xbox.RIGHT_STICK_Y), -driver.getButtonAxis(Xbox.RIGHT_STICK_X), -driver.getButtonAxis(Xbox.LEFT_STICK_X));
        }        
        /////////////////////////////////////////////
        if(driver.leftTrigger.isPressed()){ 
        	fsm.setGoalState(FSM.PRE_TOTE);
        }
        //////////////////////////////////
        if(driver.rightTrigger.isPressed()) {
        	fsm.setGoalState(FSM.LOAD_TOTE); //LOAD TOTE SEQUENCE
        }
        ///////////////////////////////////////////////////////
        if(driver.rightBumper.isPressed()){
        	robot.intakeRollersForward();
        }
        //////////////////////////////////////////////////////////////////// 
        if(driver.leftBumper.isPressed()){ //reverse rollers
            robot.intakeRollersReverse();
        }
        //////////////////////////////////////////////////////
        if(driver.backButton.isPressed()){  // stop all 
        	robot.intakeRollersStop();
        }
        ////////////////////////////////////////////////////////
        if(driver.startButton.isPressed()){
        	fsm.setGoalState(FSM.PRE_TOTE);
        }
        if(driver.leftCenterClick.isPressed()){
        	fsm.setGoalState(FSM.LOAD_TOTE); //LOAD TOTE SEQUENCE
        }
        if(driver.rightCenterClick.isPressed()){
        	robot.actuateArm();
        }

//        
    }
    public void update(){
    	codriver.run();
    	driver.run();
    	coDriver();
    	driver();
    }
    
}
