package SubSystems;

import Sensors.MA3;
import Utilities.Constants;
import Utilities.Ports;
import Utilities.Util;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class CanLift extends SynchronousPID implements Controller
{
    private VictorSP drive;
    private MA3 enc;
    private double goalPosition;
    private boolean isOnTarget = false;
    private int onTargetThresh = 10;
    private int onTargetCounter = onTargetThresh;
    public static double kOnTargetToleranceInches = Constants.CAN_TOLERANCE;
    private static CanLift instance = null;
    private DigitalInput toteBumper;
    private Solenoid clapper;
    private boolean clapperState = false;
    private PowerDistributionPanel pdp;
    private DigitalInput topLift;
    public static CanLift getInstance()
    {
        if( instance == null )
            instance = new CanLift();
        return instance;
    }

    private CanLift()
    {
        loadProperties();
        drive = new VictorSP(Ports.CAN_LIFT);
        enc = new MA3(Ports.CAN_LIFT_ENC);
        goalPosition = getAngle();
        this.setGoal(goalPosition);
        toteBumper = new DigitalInput(Ports.TOTE_BUMPER);
        clapper = new Solenoid(1,Ports.CAN_CLAPPER);
        this.setInputRange(Constants.CAN_MIN_HEIGHT, Constants.CAN_MAX_HEIGHT);
        this.setOutputRange(-Math.abs(Constants.CAN_MAX_POWER), Math.abs(Constants.CAN_MAX_POWER));
        pdp = new PowerDistributionPanel();
        topLift = new DigitalInput(Ports.ELEVATOR_TOP_LIMIT);
    }
    public void openClapper(){
    	clapper.set(true);
    	clapperState = true;
    }
    public boolean topLimit(){
    	return !topLift.get();
    }
    public void closeClapper(){
    	clapper.set(false);
    	clapperState = false;
    }
    public void cycleClapper(){
    	clapper.set(!clapperState);
    	clapperState = !clapperState;
    }
    public boolean toteOnBumper(){
    	return !toteBumper.get();
    }
    public synchronized void setGoal(double goalDistance)
    {
    	double goal = goalDistance;
    	if(goal > Constants.CAN_MAX_HEIGHT){
    		goal = Constants.CAN_MAX_HEIGHT;
    	}else if(goal < Constants.CAN_MIN_HEIGHT){
    		goal = Constants.CAN_MIN_HEIGHT;
    	}
        reset();
        if(goal < getAngle()){
        	this.setPID(Constants.CAN_DOWN_P, Constants.CAN_DOWN_I, Constants.CAN_DOWN_D);
        }else{
        	this.setPID(Constants.CAN_P, Constants.CAN_I, Constants.CAN_D);
        }
        this.setSetpoint(goal);
        goalPosition = goal;
    }
    public void stop(){ this.setGoal(this.getAngle());}
    public double getAngle(){
        return -enc.getAngle();
    }
    
    public synchronized void reset()
    {
        super.reset();
        isOnTarget = false;
        onTargetCounter = onTargetThresh;
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
            this.setGoal(Util.limit(this.goalPosition + inches, 0, Constants.CAN_MAX_HEIGHT));
        }else{
            this.setGoal(Util.limit(this.goalPosition + inches, Constants.CAN_MIN_HEIGHT, Constants.CAN_MAX_HEIGHT));
        }        
    }
    public void rawPower(double power){
    	if(power > 0)
    		drive.set(power);
    	else if(power < 0){
    		drive.set(power);
    	}else{
    		drive.set(0);
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
        this.setPID(Constants.CAN_DOWN_P, Constants.CAN_DOWN_I, Constants.CAN_DOWN_D);
    }
    public void upGain(){
        this.setPID(Constants.CAN_P, Constants.CAN_I, Constants.CAN_D);
    }
    public synchronized void run()
    {
    	double current = getAngle();        
        double power = this.calculate(current);
        double amps = pdp.getCurrent(7);
        if(Util.onTarget(goalPosition,current,kOnTargetToleranceInches) || isOnTarget )
        {
        	if(onTarget()){
                isOnTarget = true;
        	}
        	else
        		onTargetCounter--;        	
        }
        else
        {
        	onTargetCounter = onTargetThresh;
            isOnTarget = false;
        }
        if(amps > 12.0)
        	drive.set(0);
        else{
        	if((current <= Constants.CAN_MIN_HEIGHT+5.0 || current >= Constants.CAN_MAX_HEIGHT-5.0) && amps > 5.0)
        		drive.set(0);
        	else{
        		drive.set(power);
        	}
        		
        }        	
        SmartDashboard.putNumber("CAN_ANGLE", current);
        SmartDashboard.putNumber("CAN_GOAL", goalPosition);
        SmartDashboard.putNumber("CAN_POWER", power);
        SmartDashboard.putNumber("CAN_P", this.getP());
        SmartDashboard.putNumber("CAN_I", this.getI());
        SmartDashboard.putNumber("CAN_D", this.getD());
        SmartDashboard.putBoolean("TOTE_BUMPER", toteOnBumper());
        SmartDashboard.putNumber("CAN_LIFT_AMPS", amps);
        SmartDashboard.putBoolean("CLAPPER_OPEN",clapperState );
    }

    public synchronized boolean onTarget(){return onTargetCounter <= 0;}
    public boolean withinRange(double current){
        return ((Math.abs(goalPosition) - Math.abs(current)) < kOnTargetToleranceInches + 0.5) &&  ((Math.abs(goalPosition) - Math.abs(current)) > -(kOnTargetToleranceInches + 0.5) );
    }
    public synchronized boolean onTargetNow()
    {
        return Util.onTarget(this.getSetpoint(), -enc.getAngle(), Constants.CAN_TOLERANCE);
    }
    public double error(){
        return getAngle() - this.getSetpoint();
    }
    public final void loadProperties()
    {
        double kp = Constants.CAN_P;
        double ki = Constants.CAN_I;
        double kd = Constants.CAN_D;
        this.setPID(kp, ki, kd);
        this.setInputRange(Constants.CAN_MIN_HEIGHT, Constants.CAN_MAX_HEIGHT);
        kOnTargetToleranceInches = Constants.CAN_TOLERANCE;
    }
    public void setOnTargetThreshHold(int threshHold){
    	onTargetThresh = threshHold;
    }
    
    public synchronized void update(){
    	double goal = this.getSetpoint();
    	double current = -enc.getAngle();        
        double power = this.calculate(current);
        
        if(Util.onTarget(goalPosition,current,kOnTargetToleranceInches) || isOnTarget )
        {
        	if(onTarget())
                isOnTarget = true;
        	else
        		onTargetCounter--;
        	
            if(goal == 0){
                power = 0;
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