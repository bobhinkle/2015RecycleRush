package SubSystems;

import Sensors.MA3;
import Utilities.Constants;
import Utilities.Ports;
import Utilities.TrajectorySmoother;
import Utilities.Util;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveTrain{
	private static DriveTrain instance = null;
	
	private SwerveDriveModule frontLeft;
	private SwerveDriveModule frontRight;
	private SwerveDriveModule rearLeft;
	private SwerveDriveModule rearRight;
//	private static double R = Math.sqrt(Math.pow(Constants.WHEELBASE_LENGTH,2.0) + Math.pow(Constants.WHEELBASE_WIDTH,2.0))/2.0;
	private static double R = 39.52216348;
	private double xInput,yInput,rotateInput = 0;
	private double inputTimeStamp = 0.0;
	private double pointOfRotationX = 0.0;
	private double pointOfRotationY = 0.0;
	private Navigation nav;
	public HeadingController heading;
	public DistanceController distance;
	private boolean useHeadingController = false;
	private boolean useDistanceController = false;
	private boolean useTurnController = false;
	private boolean setHeading = false;
	private boolean turn45 = false;
	private double headingControllerInput = 0;
	
	public DriveTrain(){
		frontLeft  = new SwerveDriveModule(Ports.FRONT_LEFT_MA3,Ports.FRONT_LEFT_ROTATION,Ports.FRONT_LEFT_DRIVE,2);
		frontRight = new SwerveDriveModule(Ports.FRONT_RIGHT_MA3,Ports.FRONT_RIGHT_ROTATION,Ports.FRONT_RIGHT_DRIVE,1);
		rearLeft   = new SwerveDriveModule(Ports.REAR_LEFT_MA3,Ports.REAR_LEFT_ROTATION,Ports.REAR_LEFT_DRIVE,3);
		rearRight  = new SwerveDriveModule(Ports.REAR_RIGHT_MA3,Ports.REAR_RIGHT_ROTATION,Ports.REAR_RIGHT_DRIVE,4);
		nav = Navigation.getInstance();
		heading = new HeadingController();
		distance = new DistanceController();
	}
	public static DriveTrain getInstance()
    {
        if( instance == null )
            instance = new DriveTrain();
        return instance;
    }
	public void sendInputHeadingHold(double x, double y, double rotatex,boolean halfPower){
		double angle = nav.getRawHeading()/180.0*Math.PI;		
		if(!halfPower){
			y = y * 0.6;
			x = x * 0.6;
		}
		if(rotatex < -0.2 || rotatex > 0.2)
			heading.setGoal(nav.getRawHeading() + (rotatex * 5.0));
    	heading.run();
    	rotateInput = Util.turnControlSmoother(headingControllerInput);
	    xInput = (y * Math.sin(angle)) + (x * Math.cos(angle));
		yInput = (-y * Math.cos(angle)) + (x * Math.sin(angle));
	    turn45 = false;
	    SmartDashboard.putNumber("X Input", xInput);
		SmartDashboard.putNumber("Y Input", yInput);
		SmartDashboard.putNumber("rotate", rotateInput);
		SmartDashboard.putNumber("X InputC", x);
		SmartDashboard.putNumber("Y InputC", y);
		update();
	}
	public void sendInput(double x, double y, double rotate,boolean halfPower,boolean robotCentric,boolean bypassHeadingController){
		double angle = nav.getRawHeading()/180.0*Math.PI;
		if(!halfPower){
			y = y * 0.5;
			x = x * 0.5;			
		}
		if(robotCentric){
			xInput = x;
			yInput = -y;
			rotateInput = rotate;
			useHeadingController = false;
			turn45 = false;
			heading.setGoal(heading.getSetpoint() + (Constants.MAX_ROTATION_ANGLE_PER_SEC * rotate));
		}else{
			xInput = (y * Math.sin(angle)) + (x * Math.cos(angle));
			yInput = (-y * Math.cos(angle)) + (x * Math.sin(angle));
			turn45 = true;
			if(rotate < -0.2 || rotate > 0.2){
				rotateInput = rotate;
				setHeading = true;
			}else{
				if(setHeading){
					setHeading = false;
					heading.setGoal(nav.getRawHeading());
				}
				if(bypassHeadingController){
					rotateInput = rotate;
				}else{
					if(heading.onTarget()){
						heading.run();
						rotateInput = Util.turnControlSmoother(headingControllerInput);	
					}
				}
			}			
		}
		SmartDashboard.putNumber("X Input", xInput);
		SmartDashboard.putNumber("Y Input", yInput);
		SmartDashboard.putNumber("rotate", rotateInput);
		SmartDashboard.putNumber("X InputC", x);
		SmartDashboard.putNumber("Y InputC", y);
		SmartDashboard.putNumber("rotateC", rotate);
		SmartDashboard.putBoolean("RobotCentric", robotCentric);
		SmartDashboard.putBoolean("useHeadingController", useHeadingController);
		update();
	}
	public void setPointOfRotation(double radius){
		double heading = nav.getHeadingInDegrees();
		double x,y = 0;
		if(0 <= heading && heading <= 180){
			x = Math.sin(heading) * radius;
			y = Math.cos(heading) * radius;
		}else{
			x = Math.cos(heading) * radius;
			y = Math.sin(heading) * radius;
		}
		pointOfRotationX = nav.getX() - x;
		pointOfRotationY = nav.getY() - y;
	}
	public void dosado(double power, double radius ){
		double rads = Util.degreesToRadians(nav.getHeadingInDegrees());
		double fwd = -rads * Math.cos(rads) * radius;
		double str = rads * Math.sin(rads) * radius;
	}
	private class SwerveDriveModule extends SynchronousPID implements Controller{
		private MA3 rotationMA3;
		private VictorSP rotationMotor;
		private VictorSP driveMotor;
		private int moduleID;
		private String dashboardNameAngle;
		private String dashboardNamePower;
		private String dashboardNameGoal;
		private double goalPosition;
		private boolean reversePower = false;
		public SwerveDriveModule(int ma3,int rotationMotorPort, int driveMotorPort,int moduleNum){
			loadProperties();
			rotationMA3 = new MA3(ma3);
			rotationMotor = new VictorSP(rotationMotorPort);
			driveMotor = new VictorSP(driveMotorPort);
			moduleID = moduleNum;
			dashboardNameAngle = "WheelAngle" + Integer.toString(moduleID);
			dashboardNamePower = "WheelRotationPower" + Integer.toString(moduleID);
			dashboardNameGoal = "WheelRotationGoal" + Integer.toString(moduleID);
            goalPosition = Util.boundAngleNeg180to180Degrees(rotationMA3.getAngle());            
		}
		private void reverseDirection(){
			reversePower = true;
		}
		private void forwardDirection(){
			reversePower = false;
		}
		public synchronized void setGoal(double goalAngle)
	    {
			if(shortestPath(getCurrentAngle(),goalAngle)){
				forwardDirection();
				this.setSetpoint(goalAngle);
		        goalPosition = goalAngle;
			}else{
				reverseDirection();
				goalPosition = Util.boundAngleNeg180to180Degrees(goalAngle + 180.0);
//				System.out.println("Setting " + goalPosition);
				this.setSetpoint(goalPosition);		        
			}	        
	    }
	    public final void loadProperties()
	    {
	        double kp = Constants.STEERING_P;
	        double ki = Constants.STEERING_I;
	        double kd = Constants.STEERING_D;
	        this.setPID(kp, ki, kd);
	        this.setInputRange(-180.0,180.0);
            this.setOutputRange(-1.0,1.0);
            this.setContinuous(true);
	    }public void setDriveSpeed(double power){
	    	if(reversePower)
	    		driveMotor.set(-power);
	    	else
	    		driveMotor.set(power);
		}
		public double getCurrentAngle(){
			return rotationMA3.getAngle();
		}
		@Override
		public void run() {   
			double angle = getCurrentAngle();
	        double calPower = this.calculate(angle);
	        SmartDashboard.putNumber(dashboardNamePower, calPower);
	        SmartDashboard.putNumber(dashboardNameAngle,Util.boundAngleNeg180to180Degrees(angle));
	        SmartDashboard.putNumber(dashboardNameGoal,this.getSetpoint());
			rotationMotor.set(-calPower);	
		}
		@Override
		public boolean onTarget() {
			// TODO Auto-generated  method stub
			return false;
		}
		//find shortest path. reverse motor power if shortest distance is 180 degress from goal
		private boolean shortestPath(double current, double goal){
			double goal2 = Util.boundAngleNeg180to180Degrees(goal+180);
			double G1 = Math.abs(goal - current);
			double G2 = Math.abs(goal2 - current);
//			System.out.println(current + " " + goal + " " + goal2 + " " + G1 + " " + G2);
//			return G1 < G2;
			return true;
		}
	}
	public void run(){
		frontLeft.run();
		frontRight.run();
		rearLeft.run();
		rearRight.run();
	}
	private void update(){
		if(xInput == 0 && yInput == 0 && rotateInput == 0){
			if(turn45){
/*				frontLeft.setGoal(135.0);
				frontRight.setGoal(45.0);
				rearLeft.setGoal(45.0);
				rearRight.setGoal(-45.0);*/
			}
			frontLeft.setDriveSpeed(0.0);
			frontRight.setDriveSpeed(0.0);
			rearLeft.setDriveSpeed(0.0);
			rearRight.setDriveSpeed(0.0);
		}else{
			//do pid calculation and apply power to each module
			double A = xInput - rotateInput * (Constants.WHEELBASE_LENGTH / R);
	        double B = xInput + rotateInput * (Constants.WHEELBASE_LENGTH / R);
	        double C = yInput - rotateInput * (Constants.WHEELBASE_WIDTH / R);
	        double D = yInput + rotateInput * (Constants.WHEELBASE_WIDTH / R);
	        
	        //find wheel speeds
	        double frontRightWheelSpeed = Math.sqrt((B * B) + (C * C));
	        double frontLeftWheelSpeed  = Math.sqrt((B * B) + (D * D));
	        double rearLeftWheelSpeed   = Math.sqrt((A * A) + (D * D));
	        double rearRightWheelSpeed  = Math.sqrt((A * A) + (C * C));
	        //normalize wheel speeds
	        double max = frontRightWheelSpeed;
	        max = Util.normalize(max, frontLeftWheelSpeed);
	        max = Util.normalize(max, rearLeftWheelSpeed);
	        max = Util.normalize(max, rearRightWheelSpeed);
	        if(max > 1.0){
	        	frontRightWheelSpeed /= max;
	            frontLeftWheelSpeed /= max;
	            rearLeftWheelSpeed /= max;
	            rearRightWheelSpeed /= max;
	        }        
	        //find steering angles
	        double frontRightSteeringAngle = Math.atan2(B, C)*180/Math.PI; 
	        double frontLeftSteeringAngle = Math.atan2(B, D)*180/Math.PI;
	        double rearLeftSteeringAngle = Math.atan2(A, D)*180/Math.PI;
	        double rearRightSteeringAngle = Math.atan2(A, C)*180/Math.PI;
	        frontLeft.setGoal(frontLeftSteeringAngle);
			frontRight.setGoal(frontRightSteeringAngle);
			rearLeft.setGoal(rearLeftSteeringAngle);
			rearRight.setGoal(rearRightSteeringAngle);
			frontLeft.setDriveSpeed(-frontLeftWheelSpeed);
			frontRight.setDriveSpeed(frontRightWheelSpeed);
			rearLeft.setDriveSpeed(-rearLeftWheelSpeed);
			rearRight.setDriveSpeed(rearRightWheelSpeed);
		}
	}
	public void setHeading(double angle){
		heading.setGoal(angle);
	}
	
	public enum Axis{
    	X, Y, BOTH, BOTH_MINUS_X
    }
	public void killDistanceController(){
		useDistanceController = false;
	}
	public class DistanceController extends FeedforwardPIV implements Controller
	{
	    public static final double kLoopRate = 200.0;
	    private double lastDistance = 0;
	    private TrajectorySmoother trajectory;
	    private static final int onTargetThresh = 2;
	    private int onTargetCounter = onTargetThresh;
	    private static final double kOnTargetToleranceDegrees = Constants.DISTANCE_TOLERANCE;
	    private Axis motion = Axis.Y;
	    private double maxVelocity = 0.0;
	    private double maxAccel	   = 0.0;
	    public DistanceController()
	    {
	    	loadProperties();
	    }
	    public synchronized void setGoal(double goalDistance)
	    {
	    	reset();
	    	useDistanceController = true;
	    	useHeadingController = true;
	    	this.setSetpoint(goalDistance);
	    }
	    
	    public synchronized void reset()
	    {
	        lastDistance = nav.getY();
	        this.setSetpoint(lastDistance);
	        onTargetCounter = onTargetThresh;
	    }
	    public void setAxis(Axis axis){
	    	motion = axis;
	    }
	    public synchronized void run()
	    {
	    	double x = 0,y = 0;
	    	double angle = nav.getRawHeading()/180.0*Math.PI;
	    	double current;
	    	if(motion == Axis.X){
	    		current = nav.getX();
	    	}else{
	    		current = nav.getY();
	    	}
	        double position = this.getSetpoint() - current;
	        double velocity = (current- lastDistance) * kLoopRate;
	        trajectory.update(position, velocity, 0.0, 1.0/kLoopRate);
	        double output = this.calculate(this.getSetpoint(), this.maxVelocity, this.maxAccel, current, velocity, 1.0/kLoopRate);
	        if(current > this.getSetpoint() && motion == Axis.X){
	        	output = -output;
	        }
	        if(useDistanceController){
		        if(!Util.onTarget(this.getSetpoint(),current,kOnTargetToleranceDegrees))
		        {
		        	switch(motion){
		        	case X:
		        		x = output;
		        		y = 0;
		        		break;
		        	case Y:
		        		x = 0;
		        		y = -output;
		        		break;
		        	case BOTH:
		        		x = output;
		        		y = -output;
		        		break;
		        	case BOTH_MINUS_X:
		        		x = output;
		        		y = output;
		        		break;
		        	}
		        	
		            onTargetCounter = onTargetThresh;
		        }
		        else
		        {
		            onTargetCounter--;
		            x = 0;
	        		y = 0;
		            if(onTarget()){
		            	useDistanceController = false;
		            }
		        }
		        xInput = (y * Math.sin(angle)) + (x * Math.cos(angle));
				yInput = (-y * Math.cos(angle)) + (x * Math.sin(angle));
				rotateInput = headingControllerInput;
				turn45 = true;
		        SmartDashboard.putNumber("X InputD", xInput);
				SmartDashboard.putNumber("Y InputD", yInput);
				SmartDashboard.putNumber("Rotate InputD", rotateInput);
				update();
	        }	        
	        lastDistance = current;
	        SmartDashboard.putNumber("D_VELOCIY", velocity);
	    }
	    public void setMaxVelAcell(double vel, double acel){
	    	this.maxVelocity = vel;
	    	this.maxAccel = acel;
	    }
	    public final void loadProperties()
	    {
	        double kp = Constants.DIST_KP;
	        double ki = Constants.DIST_KI;
	        double kd = Constants.DIST_KD;
	        double kfa = Constants.DIST_KFA;
	        double kfv = Constants.DIST_KFV;
	        setParams(kp, ki, kd, kfv, kfa);
	        trajectory = new TrajectorySmoother(Constants.DIST_MAX_ACCEL,Constants.DIST_MAX_VEL);
	        this.maxVelocity = Constants.DIST_MAX_VEL;
	        this.maxAccel = Constants.DIST_MAX_ACCEL;
	        this.setOutputRange(-1.0, 1.0);
	    }
		@Override
		public boolean onTarget() {
			// TODO Auto-generated method stub
			return onTargetCounter <= 0;
		}
	}
	public class HeadingController extends SynchronousPID implements Controller
	{
	    public static final double kLoopRate = 200.0;
	    private double lastHeading = 0;
	    private TrajectorySmoother trajectory;
	    private static final int onTargetThresh = 10;
	    private int onTargetCounter = onTargetThresh;
	    private static final double kOnTargetToleranceDegrees = Constants.TURN_ON_TARGET_DEG;
	    private boolean isOnTarget = false;
	    
	    public HeadingController()
	    {
	    	loadProperties();
	    }
	    public synchronized void setGoal(double goalAngle)
	    {
	    	reset();
	    	this.setSetpoint(goalAngle);
	    	useHeadingController = false;
	    }
	    
	    public synchronized void reset()
	    {
	        lastHeading = nav.getRawHeading();
	        this.setSetpoint(lastHeading);
	        onTargetCounter = onTargetThresh;
	        isOnTarget = false;
	    }
	    public synchronized double getError(){
	    	return nav.getRawHeading() - this.getSetpoint();
	    }
	    public synchronized void run()
	    {
	    	double current = nav.getRawHeading();
	        double position = Util.getDifferenceInAngleDegrees(this.getSetpoint(), current);
	        double velocity = Util.getDifferenceInAngleDegrees(current, lastHeading) * kLoopRate;
	        trajectory.update(position, velocity, 0.0, 1.0/kLoopRate);
	        double output = this.calculate(current);
	        if(Util.onTarget(this.getSetpoint(),current,kOnTargetToleranceDegrees) || isOnTarget)
	        {
	        	if(onTarget())
	                isOnTarget = true;
	        	else
	        		onTargetCounter--;
	            headingControllerInput = 0;
	        }
	        else
	        {
	        	headingControllerInput = output;
	            onTargetCounter = onTargetThresh;
	        }
	        lastHeading = current;
	        SmartDashboard.putNumber("T_DIFF", position);
	        SmartDashboard.putNumber("T_VELOCIY", velocity);
	        SmartDashboard.putNumber("T_GOAL", this.getSetpoint());
	        SmartDashboard.putNumber("MOD", current%360.0);
	    }
	    public final void loadProperties()
	    {
	        double kp = Constants.TURN_KP;
	        double ki = Constants.TURN_KI;
	        double kd = Constants.TURN_KD;
	        double kfa = Constants.TURN_KFA;
	        double kfv = Constants.TURN_KFV;
	        this.setPID(kp, ki, kd);
	        trajectory = new TrajectorySmoother(Constants.TURN_MAX_ACCEL,Constants.TURN_MAX_VEL);
	        this.setOutputRange(-0.5, 0.5);
	    }
		@Override
		public boolean onTarget() {
			// TODO Auto-generated method stub
			return onTargetCounter <= 0;
		}
	}
	double timeout = 0;
	public void setTimeOut(double timer){
		timeout = System.currentTimeMillis() + timer;
	}
	public void driveDistanceHoldingHeading(double distance, Axis axis, double heading, boolean gyroReset,double timer,boolean zeroPosition){
		setTimeOut(timer);
		if(zeroPosition){
			nav.resetRobotPosition(0, 0, 0, gyroReset);
		}
		this.heading.setGoal(heading);
		this.distance.setAxis(axis);
		this.distance.setGoal(distance);
	}
	
	public boolean runDC(){
		if(this.distance.onTarget() || (timeout < System.currentTimeMillis())){
			killDistanceController();
			return false;
		}else{
			return true;
		}
	}
}