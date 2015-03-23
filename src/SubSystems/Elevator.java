package SubSystems;

import Sensors.IRProximity;
import Sensors.SuperEncoder;
import Utilities.Constants;
import Utilities.Ports;
import Utilities.Util;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Elevator extends SynchronousPID implements Controller
{
    private VictorSP drive;
    private SuperEncoder eleEnc;
    private DigitalInput lowerLimitSwitch;
    private DigitalInput upperLimitSwitch;
    private IRProximity lineBreak;
    private DigitalInput toteBumperSwitch;
    private double goalPosition;
    private boolean isOnTarget = false;
    private int onTargetThresh = 50;
    private int onTargetCounter = onTargetThresh;
    public static double kOnTargetToleranceInches = Constants.ELEVATOR_TOLERANCE;
    private static Elevator instance = null;
    private int toteCounter = 0;
    private int manualToteCount = 0;
    private Solenoid topStackHooks;
    public static Elevator getInstance()
    {
        if( instance == null )
            instance = new Elevator();
        return instance;
    }

    private Elevator()
    {
        loadProperties();
        drive = new VictorSP(Ports.ELEVATOR);
        eleEnc = new SuperEncoder(Ports.ELEVATOR_ENC,Ports.ELEVATOR_ENC+1,false,SuperEncoder.HIGH_RESOLUTION);//comp false
        eleEnc.setPIDReturn(SuperEncoder.PID_DISTANCE);
        eleEnc.setDistancePerPulse(Constants.ELEVATOR_DISTANCE_PER_PULSE);
        eleEnc.start();
        lineBreak = new IRProximity(Ports.REAR_LINE_BREAK);
        toteBumperSwitch = new DigitalInput(Ports.TOTE_BUMPER);
        lowerLimitSwitch = new DigitalInput(Ports.ELEVATOR_BOTTOM_LIMIT);
        upperLimitSwitch = new DigitalInput(Ports.ELEVATOR_TOP_LIMIT);
        goalPosition = eleEnc.getDistance();
        this.setInputRange(Constants.ELEVATOR_MIN_HEIGHT, Constants.ELEVATOR_MAX_HEIGHT);
        this.setOutputRange(-Math.abs(Constants.ELEVATOR_MAX_POWER), Math.abs(Constants.ELEVATOR_MAX_POWER));
        topStackHooks = new Solenoid(Ports.REVERSE_CARRIAGE_TOTE_HOOK);
    }

    public synchronized void setGoal(double goalDistance)
    {
    	double goal = goalDistance;
    	if(goal > Constants.ELEVATOR_MAX_HEIGHT){
    		goal = Constants.ELEVATOR_MAX_HEIGHT;
    	}else if(goal < Constants.ELEVATOR_MIN_HEIGHT){
    		goal = Constants.ELEVATOR_MIN_HEIGHT;
    	}
        reset();
        if(goal < getHeight()){
        	this.setPID(Constants.ELEVATOR_DOWN_P, Constants.ELEVATOR_DOWN_I, Constants.ELEVATOR_DOWN_D);
        }else{
        	this.setPID(Constants.ELEVATOR_P, Constants.ELEVATOR_I, Constants.ELEVATOR_D);
        }
        this.setSetpoint(goal);
        goalPosition = goal;
    }
    public void stop(){ this.setGoal(this.getHeight());}
    public double getHeight(){
        return eleEnc.getDistance();
    }
    public int getManualToteCount(){
    	return manualToteCount;
    }
    public void resetManualToteCount(){
    	manualToteCount = 0;
    }
    public void increaseManualToteCount(){
    	manualToteCount = (int)Util.limit(manualToteCount + 1, 0, 3);
    }
    public void decreaseManualToteCount(){
    	manualToteCount = (int)Util.limit(manualToteCount - 1, 0, 3);
    }
    public synchronized void reset()
    {
        super.reset();
        isOnTarget = false;
        onTargetCounter = onTargetThresh;
    }
    public boolean checkLowerLimit(){
        if(!lowerLimitSwitch.get()){
               eleEnc.reset();
               isOnTarget = true;
               return true;
        }else{
            return false;
        }
    }
    public boolean checkUpperLimit(){
        if(!upperLimitSwitch.get()){
               isOnTarget = true;
               return true;
        }else{
            return false;
        }
    }
    public boolean toteOnBumper(){
    	return !toteBumperSwitch.get();
    }
    public boolean lineBreakTrigger(){
    	return lineBreak.getDistance() > 2.2;
    }
    public void closeTopStackHook(){
    	topStackHooks.set(true);
    }
    public void openTopStackHook(){
    	topStackHooks.set(false);
    }
    public void lowerP(){
        double p = this.getP();
        double d = this.getD();
        double i = this.getI();
        
        p -= 0.01;
        this.setPID(p, i, d);
    }
    public void upP(){
        double p = this.getP();
        double d = this.getD();
        double i = this.getI();
        
        p += 0.01;
        this.setPID(p, i, d);
    }
    public void lowerD(){
        double p = this.getP();
        double d = this.getD();
        double i = this.getI();
        
        d -= 0.01;
        this.setPID(p, i, d);
    }
    public void upD(){
        double p = this.getP();
        double d = this.getD();
        double i = this.getI();
        
        d += 0.01;
        this.setPID(p, i, d);
    }
    public void trim(double inches){
        reset();
        if(inches > 0){
            this.setGoal(Util.limit(this.goalPosition + inches, 0, Constants.ELEVATOR_MAX_HEIGHT));
        }else{
            this.setGoal(Util.limit(this.goalPosition + inches, Constants.ELEVATOR_MIN_HEIGHT, Constants.ELEVATOR_MAX_HEIGHT));
        }        
    }
    public void manualUp(double height){
    	this.setGoal(this.getSetpoint() + (height * 2.0));
    }
    public void manualDown(double height){
    	this.setGoal(this.getSetpoint() - (height*2.0));
    }
    public void manualStop(){
        drive.set(0);
    }
    public void downGain(){
        this.setPID(Constants.ELEVATOR_DOWN_P, Constants.ELEVATOR_DOWN_I, Constants.ELEVATOR_DOWN_D);
    }
    public void upGain(){
        this.setPID(Constants.ELEVATOR_P, Constants.ELEVATOR_I, Constants.ELEVATOR_D);
    }
    public void increaseToteCount(){
    	toteCounter +=1;
    	System.out.println("TOTEINCREASE" + toteCounter);
    }
    public void resetToteCounter(){
    	toteCounter = 0;
    }
    public int getToteCount(){
    	return toteCounter;
    }
    public synchronized void run()
    {
    	double goal = this.getSetpoint();
    	double current = eleEnc.getDistance();        
        double power = -this.calculate(current);
        boolean lowerLimit = checkLowerLimit();
        boolean upperLimit = checkUpperLimit();
        
        if(Util.onTarget(goalPosition,current,kOnTargetToleranceInches) || isOnTarget )
        {
        	if(onTarget())
                isOnTarget = true;
        	else
        		onTargetCounter--;
        	
            if(goal == 0 && lowerLimit){
                power = 0;
            }else{
	            if(goal <= 0 && lowerLimit){ //Elevator is at the bottom but goal is below current position. Reset to 0
	                this.setGoal(0);
	                power = 0;
	            }else if(upperLimit){ //Elevator encoder is off and is trying to go beyond limit. Set goal to current height
	            	this.setGoal(current);
	            }else if(goal <= 0 && !lowerLimit){
//	            	this.setGoal(current - 1.0);
	            }
            }
        }
        else
        {
        	onTargetCounter = onTargetThresh;
            isOnTarget = false;
        }
        power = Util.deadBand(power, 0.1);
//    	drive.set(-power);
        SmartDashboard.putNumber("ELE_HEIGHT", current);
        SmartDashboard.putNumber("ELE_GOAL", goalPosition);
        SmartDashboard.putNumber("ELE_POWER", power);
//        SmartDashboard.putNumber("ELE_RAW", eleEnc.getRaw());
        SmartDashboard.putNumber("ELE_P", this.getP());
        SmartDashboard.putNumber("ELE_I", this.getI());
        SmartDashboard.putNumber("ELE_D", this.getD());
        SmartDashboard.putNumber("LINE_BREAK", this.lineBreak.getDistance());
        SmartDashboard.putBoolean("TOTE_BUMPER", toteOnBumper());
        SmartDashboard.putBoolean("bottomHall", lowerLimitSwitch.get());
        SmartDashboard.putBoolean("TopSwitch", upperLimitSwitch.get());
        SmartDashboard.putNumber("TOTE_COUNT", getToteCount());
        SmartDashboard.putNumber("MAN_TOTE", getManualToteCount());
    }

    public synchronized boolean onTarget(){return onTargetCounter <= 0;}
    public boolean withinRange(double current){
        return ((Math.abs(goalPosition) - Math.abs(current)) < kOnTargetToleranceInches + 0.5) &&  ((Math.abs(goalPosition) - Math.abs(current)) > -(kOnTargetToleranceInches + 0.5) );
    }
    public synchronized boolean onTargetNow()
    {
        return Util.onTarget(this.getSetpoint(), eleEnc.getDistance(), Constants.ELEVATOR_TOLERANCE);
    }
    public double error(){
        return getHeight() - this.getSetpoint();
    }
    public final void loadProperties()
    {
        double kp = Constants.ELEVATOR_P;
        double ki = Constants.ELEVATOR_I;
        double kd = Constants.ELEVATOR_D;
        this.setPID(kp, ki, kd);
        this.setInputRange(Constants.ELEVATOR_MIN_HEIGHT, Constants.ELEVATOR_MAX_HEIGHT);
        kOnTargetToleranceInches = Constants.ELEVATOR_TOLERANCE;
    }
    public void setOnTargetThreshHold(int threshHold){
    	onTargetThresh = threshHold;
    }
    
    public synchronized void update(){
    	double goal = this.getSetpoint();
    	double current = eleEnc.getDistance();        
        double power = this.calculate(current);
        boolean lowerLimit = checkLowerLimit();
        boolean upperLimit = checkUpperLimit();
        
        if(Util.onTarget(goalPosition,current,kOnTargetToleranceInches) || isOnTarget )
        {
        	if(onTarget())
                isOnTarget = true;
        	else
        		onTargetCounter--;
        	
            if(goal == 0){
                power = 0;
            }else{
	            if(goal < 0 && lowerLimit){ //Elevator is at the bottom but goal is below current position. Reset to 0
	                this.setGoal(0);
	            }else if(upperLimit){
	            	this.setGoal(current);
	            }
            }
        }
        else
        {
        	onTargetCounter = onTargetThresh;
            isOnTarget = false;
        }
        
    	drive.set(-power);
    }
}