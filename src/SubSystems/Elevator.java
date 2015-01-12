package SubSystems;

import Sensors.SuperEncoder;
import Utilities.Constants;
import Utilities.Ports;
import Utilities.Util;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Elevator extends SynchronousPID implements Controller
{
    private Victor drive;
    private SuperEncoder eleEnc;
    private DigitalInput limitSwitch;
    
    private double goalPosition;
    private boolean isOnTarget = false;
    private static final int onTargetThresh = 25;
    private int onTargetCounter = onTargetThresh;
    public static double kOnTargetToleranceInches = Constants.DISTANCE_TOLERANCE;
    public static final double kLoopRate = 200.0;

    private static Elevator instance = null;
    public static Elevator getInstance()
    {
        if( instance == null )
            instance = new Elevator();
        return instance;
    }

    private Elevator()
    {
        loadProperties();
        drive = new Victor(Ports.ELEVATOR);
        eleEnc = new SuperEncoder(Ports.ELEVATOR_ENC,Ports.ELEVATOR_ENC+1,false,2);//comp false
        eleEnc.setPIDReturn(1);
        eleEnc.setDistancePerPulse(Constants.ELEVATOR_DISTANCE_PER_PULSE);
        eleEnc.start();
        limitSwitch = new DigitalInput(Ports.ELEVATOR_BOTTOM_LIMIT);
        goalPosition = eleEnc.getDistance();
        this.setInputRange(Constants.ELEVATOR_MIN_HEIGHT, Constants.ELEVATOR_MAX_HEIGHT);
        this.setOutputRange(-Math.abs(Constants.ELEVATOR_MAX_POWER), Math.abs(Constants.ELEVATOR_MAX_POWER));
    }

    public synchronized void setGoal(double goalDistance)
    {
        reset();
        this.setSetpoint(goalDistance);
        goalPosition = goalDistance;
    }
    public void stop(){ this.setGoal(this.getHeight());}
    public double getHeight(){
        return eleEnc.getDistance();
    }
    public synchronized void reset()
    {
        super.reset();
        isOnTarget = false;
        onTargetCounter = onTargetThresh;
    }
    public boolean checkLimit(){
        if(!limitSwitch.get()){
               eleEnc.reset();
               return true;
        }else{
            return false;
        }
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
    public void manualUp(){
        drive.set(0.4);
    }
    public void manualDown(){
        drive.set(-0.4);
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
    public synchronized void run()
    {
        checkLimit();
        double current = eleEnc.getDistance();
        double difference = this.getSetpoint() - current;
        
        double calPower = this.calculate(current);
        double power = Util.pidPower(-calPower, -Math.abs(Constants.ELEVATOR_MIN_POWER), -Math.abs(Constants.ELEVATOR_MAX_POWER),Math.abs(Constants.ELEVATOR_MIN_POWER), Math.abs(Constants.ELEVATOR_MAX_POWER));
        
        if(Util.onTarget(goalPosition,current,kOnTargetToleranceInches) || isOnTarget )
        {
            if(onTarget())
                isOnTarget = true;
                onTargetCounter--;
                if(this.getSetpoint() == 0){
                    power = 0;
                }
                if(this.goalPosition < 0){ //Elevator is at the bottom but goal is below current position. Reset to 0
                    eleEnc.reset();
                    this.setGoal(0);
                }
        }
        else
        {
           if(!checkLimit()){
               if(current >= Constants.ELEVATOR_MAX_HEIGHT){
                   if(power < 0) // only allow down movement
                     power = 0;
                }
           }else{
               if(power > 0) //only move up
                  power = 0;
           }
            onTargetCounter = onTargetThresh;
            isOnTarget = false;
        }
        drive.set(power);
        SmartDashboard.putNumber("ELE_HEIGHT", current);
        SmartDashboard.putNumber("ELE_GOAL", goalPosition);
        SmartDashboard.putNumber("ELE_POWER", power);
        SmartDashboard.putNumber("ELE_P", this.getP());
        SmartDashboard.putNumber("ELE_I", this.getI());
        SmartDashboard.putNumber("ELE_D", this.getD());
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
}