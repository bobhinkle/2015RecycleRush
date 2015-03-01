package SubSystems;

import Sensors.MA3;
import Utilities.Constants;
import Utilities.Ports;
import Utilities.TrajectorySmoother;
import Utilities.Util;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveTrain{
	private static DriveTrain instance = null;
	
	private SwerveDriveModule frontLeft;
	private SwerveDriveModule frontRight;
	private SwerveDriveModule rearLeft;
	private SwerveDriveModule rearRight;
//	private static double R = Math.sqrt(Math.pow(Constants.WHEELBASE_LENGTH,2.0) + Math.pow(Constants.WHEELBASE_WIDTH,2.0))/2.0;
	private static double R = 39.52216348;
	private double DEAD_BAND = 0.0;
	private double xInput,yInput,rotateInput;
	private double inputTimeStamp = 0.0;
	private double pointOfRotationX = 0.0;
	private double pointOfRotationY = 0.0;
	private Navigation nav;
	private HeadingController heading;
	private boolean useHeadingController = false;
	private boolean setHeading = true;
	private boolean no45 = false;
	public DriveTrain(){
		frontLeft  = new SwerveDriveModule(Ports.FRONT_LEFT_MA3,Ports.FRONT_LEFT_ROTATION,Ports.FRONT_LEFT_DRIVE,2);
		frontRight = new SwerveDriveModule(Ports.FRONT_RIGHT_MA3,Ports.FRONT_RIGHT_ROTATION,Ports.FRONT_RIGHT_DRIVE,1);
		rearLeft   = new SwerveDriveModule(Ports.REAR_LEFT_MA3,Ports.REAR_LEFT_ROTATION,Ports.REAR_LEFT_DRIVE,3);
		rearRight  = new SwerveDriveModule(Ports.REAR_RIGHT_MA3,Ports.REAR_RIGHT_ROTATION,Ports.REAR_RIGHT_DRIVE,4);
		nav = Navigation.getInstance();
		heading = new HeadingController();
	}
	public static DriveTrain getInstance()
    {
        if( instance == null )
            instance = new DriveTrain();
        return instance;
    }
	public void sendInput(double x, double y, double rotate,boolean halfPower){
		double angle = nav.getRawHeading()/180.0*Math.PI;
		if(halfPower){
			y = y/2.0;
			x = x/2.0;
		}
		xInput = (y * Math.sin(angle)) + (x * Math.cos(angle));
		yInput = (-y * Math.cos(angle)) + (x * Math.sin(angle));
		rotateInput = rotate;
		no45 = halfPower;
		if(rotate == 0){
			if(!setHeading){
				useHeadingController = true;
//				heading.run();
			}else{
				heading.reset();
				setHeading = false;
			}
		}else{
			useHeadingController = false;
			setHeading = true;
			rotateInput = rotate;
		}
		SmartDashboard.putNumber("X Input", xInput);
		SmartDashboard.putNumber("Y Input", yInput);
		SmartDashboard.putNumber("Rotate Input", rotateInput);
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
		private Victor rotationMotor;
		private Victor driveMotor;
		private int moduleID;
		private String dashboardNameAngle;
		private String dashboardNamePower;
		private String dashboardNameGoal;
		private double goalPosition;
		private boolean reversePower = false;
		public SwerveDriveModule(int ma3,int rotationMotorPort, int driveMotorPort,int moduleNum){
			loadProperties();
			rotationMA3 = new MA3(ma3);
			rotationMotor = new Victor(rotationMotorPort);
			driveMotor = new Victor(driveMotorPort);
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
			// TODO Auto-generated method stub
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
			if(!no45){
				frontLeft.setGoal(135.0);
				frontRight.setGoal(45.0);
				rearLeft.setGoal(45.0);
				rearRight.setGoal(-45.0);
			}
			frontLeft.setDriveSpeed(0);
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
	private class HeadingController extends FeedforwardPIV implements Controller
	{
	    private double goalPosition;
	    public static final double kLoopRate = 200.0;
	    private double lastHeading = 0;
	    private TrajectorySmoother trajectory;
	    public HeadingController()
	    {
	    	loadProperties();
	    }
	    public synchronized void setGoal(double goalAngle)
	    {
	        this.setSetpoint(goalAngle);
	        goalPosition = goalAngle;
	    }
	    
	    public synchronized void reset()
	    {
	        lastHeading = nav.getRawHeading();
	        setGoal(lastHeading);
	    }

	    public synchronized void run()
	    {
	        double current = nav.getRawHeading();
	        double position = Util.getDifferenceInAngleDegrees(this.getSetpoint(), current);
	        double velocity = Util.getDifferenceInAngleDegrees(current, lastHeading) * kLoopRate;
	        trajectory.update(position, velocity, 0.0, 1.0/kLoopRate);
	        double output = this.calculate(this.getSetpoint(), 90, 90, current, velocity, 1.0/kLoopRate);
	        if(useHeadingController){
	        	rotateInput = Util.deadBand(output, 0.1);
	        }
	        lastHeading = current;
	    }
	    public final void loadProperties()
	    {
	        double kp = Constants.TURN_KP;
	        double ki = Constants.TURN_KI;
	        double kd = Constants.TURN_KD;
	        double kfa = Constants.TURN_KFA;
	        double kfv = Constants.TURN_KFV;
	        setParams(kp, ki, kd, kfv, kfa);
	        trajectory = new TrajectorySmoother(Constants.TURN_MAX_ACCEL,Constants.TURN_MAX_VEL);
	        this.setOutputRange(-1.0, 1.0);
	    }
		@Override
		public boolean onTarget() {
			// TODO Auto-generated method stub
			return false;
		}
	}
}