package SubSystems;

import Sensors.MA3;
import Utilities.Constants;
import Utilities.Ports;
import Utilities.Util;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveTrain{
	private static DriveTrain instance = null;
	
	private SwerveDriveModule frontLeft;
	private SwerveDriveModule frontRight;
	private SwerveDriveModule rearLeft;
	private SwerveDriveModule rearRight;
	private static double R = Math.sqrt(Math.pow(Constants.WHEELBASE_LENGTH,2.0) + Math.pow(Constants.WHEELBASE_WIDTH,2.0))/2.0;
	private double DEAD_BAND = 0.2;
	private double xInput,yInput,rotateInput;
	private double inputTimeStamp = 0.0;
	
	public DriveTrain(){
		frontLeft  = new SwerveDriveModule(Ports.FRONT_LEFT_MA3,Ports.FRONT_LEFT_ROTATION,Ports.FRONT_LEFT_DRIVE,2);
		frontRight = new SwerveDriveModule(Ports.FRONT_RIGHT_MA3,Ports.FRONT_RIGHT_ROTATION,Ports.FRONT_RIGHT_DRIVE,1);
		rearLeft   = new SwerveDriveModule(Ports.REAR_LEFT_MA3,Ports.REAR_LEFT_ROTATION,Ports.REAR_LEFT_DRIVE,3);
		rearRight  = new SwerveDriveModule(Ports.REAR_RIGHT_MA3,Ports.REAR_RIGHT_ROTATION,Ports.REAR_RIGHT_DRIVE,4);
	}
	public static DriveTrain getInstance()
    {
        if( instance == null )
            instance = new DriveTrain();
        return instance;
    }
	public void sendInput(double x, double y, double rotate){
		
		if((DEAD_BAND < x) && (x < -DEAD_BAND) || (DEAD_BAND < y) && (y < -DEAD_BAND) || (DEAD_BAND < rotate) && (rotate < -DEAD_BAND)){
			xInput = Util.deadBand(x, DEAD_BAND);
			yInput = Util.deadBand(y, DEAD_BAND);
			rotateInput = Util.deadBand(rotate, DEAD_BAND);
			inputTimeStamp = System.currentTimeMillis();
		}else{
			if(System.currentTimeMillis() > inputTimeStamp + Constants.INPUT_DELAY){
				xInput = Util.deadBand(x, DEAD_BAND);
				yInput = Util.deadBand(y, DEAD_BAND);
				rotateInput = Util.deadBand(rotate, DEAD_BAND);
			}
		}
		
		SmartDashboard.putNumber("X Input", xInput);
		SmartDashboard.putNumber("Y Input", yInput);
		SmartDashboard.putNumber("Rotate Input", rotateInput);
		update();
		
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
		public synchronized void setGoal(double goalAngle)
	    {
			if(shortestPath(getCurrentAngle(),goalAngle)){
				reversePower = false;
				this.setSetpoint(goalAngle);
		        goalPosition = goalAngle;
			}else{
				reversePower = true;
				goalPosition = Util.boundAngleNeg180to180Degrees(goalAngle + 180.0);
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
            this.setOutputRange(-0.5,0.5);
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
	        SmartDashboard.putNumber(dashboardNameGoal,goalPosition);
			rotationMotor.set(calPower);	
		}
		@Override
		public boolean onTarget() {
			// TODO Auto-generated method stub
			return false;
		}
		//find shortest path. reverse motor power if shortest distance is 180 degress from goal
		private boolean shortestPath(double current, double goal){
			double C1 = current + 180.0;
			double G1, G2;
			G1 = goal + 180;
			if(goal >= 0)
				G2 = goal;
			else
				G2 = G1 + 180;
			return Math.abs(C1 - G1) < Math.abs(C1 - G2);
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
			frontLeft.setGoal(45.0);
			frontLeft.setDriveSpeed(0);
			frontRight.setGoal(45.0);
			frontRight.setDriveSpeed(0.0);
			rearLeft.setGoal(45.0);
			rearLeft.setDriveSpeed(0.0);
			rearRight.setGoal(45.0);
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
	        //set angles and power
	        frontLeft.setGoal(frontLeftSteeringAngle);
			frontLeft.setDriveSpeed(frontLeftWheelSpeed);
			frontRight.setGoal(frontRightSteeringAngle);
			frontRight.setDriveSpeed(frontRightWheelSpeed);
			rearLeft.setGoal(rearLeftSteeringAngle);
			rearLeft.setDriveSpeed(rearLeftWheelSpeed);
			rearRight.setGoal(rearRightSteeringAngle);
			rearRight.setDriveSpeed(rearRightWheelSpeed);
		}
	}
	
	
}