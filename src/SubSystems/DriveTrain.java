package SubSystems;

import Sensors.MA3;
import Utilities.Constants;
import Utilities.Ports;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.Victor;

public class DriveTrain{
	private static DriveTrain instance = null;
	
	private SwerveDriveModule frontLeft;
	private SwerveDriveModule frontRight;
	private SwerveDriveModule rearLeft;
	private SwerveDriveModule rearRight;
	private static double R = Math.sqrt(Math.pow(Constants.WHEELBASE_LENGTH,2.0) + Math.pow(Constants.WHEELBASE_WIDTH,2.0))/2.0;
	private int DRIVE_STYLE = DriveTrain.SIMPLE_DRIVE;
	public static final int SIMPLE_DRIVE = 0;
	public static final int STOPPED = 1;
	
	private double xInput,yInput,rotateInput;
	public DriveTrain(){
		frontLeft = new SwerveDriveModule(Ports.FRONT_LEFT_MA3,Ports.FRONT_LEFT_ROTATION,Ports.FRONT_LEFT_DRIVE);
		frontRight = new SwerveDriveModule(Ports.FRONT_RIGHT_MA3,Ports.FRONT_RIGHT_ROTATION,Ports.FRONT_RIGHT_DRIVE);
		rearLeft = new SwerveDriveModule(Ports.REAR_LEFT_MA3,Ports.REAR_LEFT_ROTATION,Ports.REAR_LEFT_DRIVE);
		rearRight = new SwerveDriveModule(Ports.REAR_RIGHT_MA3,Ports.REAR_RIGHT_ROTATION,Ports.REAR_RIGHT_DRIVE);
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
	}
	private class SwerveDriveModule implements PIDOutput, PIDSource{
		private MA3 rotationMA3;
		private Victor rotationMotor;
		private Victor driveMotor;
		private PIDController pid;
		public SwerveDriveModule(int ma3,int rotationMotorPort, int driveMotorPort){
			rotationMA3 = new MA3(ma3);
			rotationMotor = new Victor(rotationMotorPort);
			driveMotor = new Victor(driveMotorPort);
			pid = new PIDController(Constants.STEERING_P,
                    Constants.STEERING_I,
                    Constants.STEERING_D, this, this);
            pid.setInputRange(-180, 180);
            pid.setContinuous(true);
            pid.enable();
		}
		@Override
		public double pidGet() {
			return rotationMA3.getAngle();
		}

		@Override
		public void pidWrite(double output) {
			rotationMotor.set(output);			
		}
		
		public void setDriveSpeed(double power){
			driveMotor.set(power);
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
        if(max>1){
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
		switch(DRIVE_STYLE){
			case STOPPED:
			
				break;
			case SIMPLE_DRIVE:
				
				break;
		}
		frontLeft.pidWrite(frontLeftSteeringAngle);
		frontLeft.setDriveSpeed(frontLeftWheelSpeed);
		frontRight.pidWrite(frontRightSteeringAngle);
		frontRight.setDriveSpeed(frontRightWheelSpeed);
		rearLeft.pidWrite(rearLeftSteeringAngle);
		rearLeft.setDriveSpeed(rearLeftWheelSpeed);
		rearRight.pidWrite(rearRightSteeringAngle);
		rearRight.setDriveSpeed(rearRightWheelSpeed);
		
	}
}