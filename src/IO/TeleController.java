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
        if(codriver.getAButton()){
        	
        }
        //////////////////////////////////////////
        if(codriver.getBButton()){
        	
        }
        ////////////////////////////////////////
        if(codriver.getXButton()){
        	
        }
        ///////////////////////////////////////
        if(codriver.getYButton()){
           
        }
        /////////////////////////////////////////////

        if(codriver.getRightTrigger()){ 
            
        }
        //////////////////////////////////
        if(codriver.getRightBumper()) {
           
        }
        ///////////////////////////////////////////////////////
        if(codriver.getLeftTrigger()){
        	
        }
        //////////////////////////////////////////////////////////////////// 
        if(codriver.getLeftBumper()){ //reverse rollers
            
        }
        //////////////////////////////////////////////////////
        if(codriver.getBackButton()){  // stop all 
          
        }
        ////////////////////////////////////////////////////////
        if(codriver.getStartButton()){
        	
        }

        
        if (codriver.getRightStickY() > 0.4) {
            
        } else if (codriver.getRightStickY() < -0.4) {
            
        } else {
             
        }
        ///////////////////////////////////////////////
        if (codriver.getLeftStickY() > 0.4) {
            
        } else if (codriver.getLeftStickY() < -0.4) {
            
        } else {
            
        }
        ///////////////////////////////////////////////
        if(codriver.getLeftStick()){
            
        }     
        ///////////////////////////////////////////////
        if(codriver.getRightStick()) {
            
        }
        if(codriver.getDPADX() > 0){
          
        }else if(codriver.getDPADX() < 0){
          
        }else{
            
        }
    }
    
    public void driver() {

    	if(driver.getAButton()){
        	fsm.setGoalState(FSM.PRE_TOTE);
        }
        //////////////////////////////////////////
        if(driver.getBButton()){
        	fsm.setGoalState(FSM.LOAD_TOTE); //LOAD TOTE SEQUENCE
        }
        ////////////////////////////////////////
        if(driver.getXButton()){
        	
        }
        ///////////////////////////////////////
        if(driver.getYButton()){
           
        }
        /////////////////////////////////////////////

        if(driver.getRightTrigger()){ 
            
        }
        //////////////////////////////////
        if(driver.getRightBumper()) {
           
        }
        ///////////////////////////////////////////////////////
        if(driver.getLeftTrigger()){
        	
        }
        //////////////////////////////////////////////////////////////////// 
        if(driver.getLeftBumper()){ //reverse rollers
            
        }
        //////////////////////////////////////////////////////
        if(driver.getBackButton()){  // stop all 
          
        }
        ////////////////////////////////////////////////////////
        if(driver.getStartButton()){
        	
        }

        robot.dt.sendInput(driver.getLeftStickX(), driver.getLeftStickY(), driver.getRightStickX());
    }
    public void update(){
    	coDriver();
    	driver();
    }
    
}
