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
    private RoboSystem robot;
    private FSM fsm;
    private static TeleController instance = null;
    public TeleController(){

        driver = new Xbox(0);
        codriver  = new Xbox(1);
        robot = RoboSystem.getInstance();
    }
    public static TeleController getInstance(){
        if(instance == null){
            instance = new TeleController();
        }
        return instance;
    }
    public void loadSubsystems(){
        robot = RoboSystem.getInstance();
        fsm = FSM.getInstance();
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

       
        
        if(driver.getRawButton(5)){
        	
        }
        if(driver.getRawButton(10)){
            
        }
        
        if(driver.getRawButton(11)){
            
        }
        if(driver.getRawButton(8)){ //left 7
         
        }
        if(driver.getRawButton(6)){
            
        }
        if (driver.getRawButton(1)){
        	
        }
        if(driver.getRawButton(3)){
        	
        }
        robot.dt.sendInput(driver.getLeftStickX(), driver.getLeftStickY(), driver.getRightStickX());
    }
    public void update(){
    	coDriver();
    	driver();
    }
    
}
