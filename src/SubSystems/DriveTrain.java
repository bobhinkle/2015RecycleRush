package SubSystems;

import Sensors.MA3;
import Utilities.Constants;
import Utilities.Ports;
import Utilities.Util;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveTrain{
	private static DriveTrain instance = null;
	
	private SwerveDriveModule frontLeft;
	private SwerveDriveModule frontRight;
	private SwerveDriveModule rearLeft;
	private SwerveDriveModule rearRight;
	private static double R = Math.sqrt(Math.pow(Constants.WHEELBASE_LENGTH,2.0) + Math.pow(Constants.WHEELBASE_WIDTH,2.0))/2.0;
	
	private double xInput,yInput,rotateInput;
	
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
		xInput = x;
		yInput = y;
		rotateInput = rotate;
		update();
	}
	private class SwerveDriveModule implements PIDOutput, PIDSource{
		private MA3 rotationMA3;
		private Victor rotationMotor;
		private Victor driveMotor;
		public PIDController pid;
		private int moduleID;
		private String dashboardNameAngle;
		private String dashboardNamePower;
		private String dashboardNameGoal;
		public SwerveDriveModule(int ma3,int rotationMotorPort, int driveMotorPort,int moduleNum){
			rotationMA3 = new MA3(ma3);
			rotationMotor = new Victor(rotationMotorPort);
			driveMotor = new Victor(driveMotorPort);
			moduleID = moduleNum;
			dashboardNameAngle = "WheelAngle" + Integer.toString(moduleID);
			dashboardNamePower = "WheelRotationPower" + Integer.toString(moduleID);
			dashboardNameGoal = "WheelRotationGoal" + Integer.toString(moduleID);
			pid = new PIDController(Constants.STEERING_P,
                    Constants.STEERING_I,
                    Constants.STEERING_D, this, this);
            pid.setInputRange(-180, 180);
            pid.setOutputRange(-0.5, 0.5);
            pid.setContinuous(true);
            pid.enable();
            pid.setSetpoint(Util.boundAngleNeg180to180Degrees(rotationMA3.getAngle()));
		}
		@Override
		public double pidGet() {
			double angle = rotationMA3.getAngle();
			SmartDashboard.putNumber(dashboardNameAngle,Util.boundAngleNeg180to180Degrees(angle));
			SmartDashboard.putNumber(dashboardNameGoal, pid.getSetpoint());
			return Util.boundAngleNeg180to180Degrees(angle);
		}

		@Override
		public void pidWrite(double output) {
			SmartDashboard.putNumber(dashboardNamePower, output);
			rotationMotor.set(output);			
		}
		
		public void setDriveSpeed(double power){
			driveMotor.set(power);
		}
		public double getCurrentAngle(){
			return rotationMA3.getAngle();
		}
	}
	private void update(){
		//do pid calculation and apply power to each module
		double A = xInput - rotateInput * (Constants.WHEELBASE_LENGTH / R);
        double B = xInput + rotateInput * (Constants.WHEELBASE_LENGTH / R);
        double C = yInput - rotateInput * (Constants.WHEELBASE_WIDTH / R);
        double D = yInput + rotateInput * (Constants.WHEELBASE_WIDTH / R);
        //find wheel speeds
        double frontRightWheelSpeed = Math.sqrt((B * B) + (C * C));
        double frontLeftWheelSpeed = Math.sqrt((B * B) + (D * D));
        double rearLeftWheelSpeed = Math.sqrt((A * A) + (D * D));
        double rearRightWheelSpeed = Math.sqrt((A * A) + (C * C));
        //normalize wheel speeds
        double max = frontRightWheelSpeed;
        if (frontLeftWheelSpeed > max) {
            max = frontLeftWheelSpeed;
        }
        if (rearLeftWheelSpeed > max) {
            max = rearLeftWheelSpeed;
        }
        if (rearRightWheelSpeed > max) {
            max = rearRightWheelSpeed;
        }
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
        
        if(shortestPath(frontLeft.getCurrentAngle(),frontLeftSteeringAngle)){
        	frontLeft.pid.setSetpoint(frontLeftSteeringAngle);
    		frontLeft.setDriveSpeed(frontLeftWheelSpeed);
        }else{
        	frontLeft.pid.setSetpoint(Util.boundAngleNeg180to180Degrees(frontLeftSteeringAngle + 180.0));
    		frontLeft.setDriveSpeed(-frontLeftWheelSpeed);
        }
        if(shortestPath(frontRight.getCurrentAngle(),frontRightSteeringAngle)){
        	frontRight.pid.setSetpoint(frontRightSteeringAngle);
    		frontRight.setDriveSpeed(frontRightWheelSpeed);
        }else{
        	frontRight.pid.setSetpoint(Util.boundAngleNeg180to180Degrees(frontRightSteeringAngle + 180.0));
    		frontRight.setDriveSpeed(-frontRightWheelSpeed);
        }
        if(shortestPath(rearLeft.getCurrentAngle(),rearLeftSteeringAngle)){
        	rearLeft.pid.setSetpoint(rearLeftSteeringAngle);
    		rearLeft.setDriveSpeed(rearLeftWheelSpeed);
        }else{
        	rearLeft.pid.setSetpoint(Util.boundAngleNeg180to180Degrees(rearLeftSteeringAngle + 180.0));
    		rearLeft.setDriveSpeed(-rearLeftWheelSpeed);
        }
        if(shortestPath(rearRight.getCurrentAngle(),rearRightSteeringAngle)){
        	rearRight.pid.setSetpoint(rearRightSteeringAngle);
    		rearRight.setDriveSpeed(rearRightWheelSpeed);
        }else{
        	rearRight.pid.setSetpoint(Util.boundAngleNeg180to180Degrees(rearRightSteeringAngle + 180.0));
        	rearRight.setDriveSpeed(-rearRightWheelSpeed);
        }
	}
	
	public boolean shortestPath(double current, double goal){
		double C1 = current + 180.0;
		double G1, G2;
		G1 = goal + 180;
		if(goal >= 0){
			G2 = goal;
		}else{
			G2 = G1 + 180;
		}
		if(Math.abs(C1 - G1) < Math.abs(C1 - G2)){
			return true;
		}else{
			return false;
		}
	}
}