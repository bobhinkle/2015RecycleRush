package SubSystems;

import Sensors.MA3;
import Utilities.Ports;
import edu.wpi.first.wpilibj.Victor;

public class DriveTrain{
	private static DriveTrain instance = null;
	
	private SwerveDriveModule frontLeft;
	private SwerveDriveModule frontRight;
	private SwerveDriveModule rearLeft;
	private SwerveDriveModule rearRight;
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
	
	public void driveVertical(double power){
				
	}
	public void driveHorizontal(double power){
		
	}
	
	private class SwerveDriveModule{
		private MA3 rotationMA3;
		private Victor rotationMotor;
		private Victor driveMotor;
		
		public SwerveDriveModule(int ma3,int rotationMotorPort, int driveMotorPort){
			rotationMA3 = new MA3(ma3);
			rotationMotor = new Victor(rotationMotorPort);
			driveMotor = new Victor(driveMotorPort);
		}
		
		public void rotateModule(double power){
			rotationMotor.set(power);
		}
		public void moveModule(double power){
			driveMotor.set(power);
		}
		public double getAngle(){
			return rotationMA3.getAngle();
		}
	}
	private void run(){
		//do pid calculation and apply power to each module
		frontLeft.rotateModule(0);
		frontLeft.moveModule(0);
		frontRight.rotateModule(0);
		frontRight.moveModule(0);
		rearLeft.rotateModule(0);
		rearLeft.moveModule(0);
		rearRight.rotateModule(0);
		rearRight.moveModule(0);
		
	}
}