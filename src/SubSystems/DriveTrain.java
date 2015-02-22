package SubSystems;

import Sensors.MA3;
import Utilities.Calculate;
import Utilities.Constants;
import Utilities.Ports;
import Utilities.Util;
import Utilities.Vector2;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveTrain{
	private static DriveTrain instance = null;
	
	private SwerveDriveModule frontLeft;
	private SwerveDriveModule frontRight;
	private SwerveDriveModule rearLeft;
	private SwerveDriveModule rearRight;
//	private static double R = Math.sqrt(Constants.WHEELBASE_LENGTH * Constants.WHEELBASE_LENGTH) + Math.pow(Constants.WHEELBASE_WIDTH,2.0))/2.0;
	private static double R = 39.52216348;
	private double DEAD_BAND = 0.0;
	private double xInput,yInput,rotateInput;
	private double inputTimeStamp = 0.0;
	private double pointOfRotationX = 0.0;
	private double pointOfRotationY = 0.0;
	private Navigation nav;
	public DriveTrain(){
		frontLeft  = new SwerveDriveModule(Ports.FRONT_LEFT_MA3,Ports.FRONT_LEFT_ROTATION,Ports.FRONT_LEFT_DRIVE,2);
		frontRight = new SwerveDriveModule(Ports.FRONT_RIGHT_MA3,Ports.FRONT_RIGHT_ROTATION,Ports.FRONT_RIGHT_DRIVE,1);
		rearLeft   = new SwerveDriveModule(Ports.REAR_LEFT_MA3,Ports.REAR_LEFT_ROTATION,Ports.REAR_LEFT_DRIVE,3);
		rearRight  = new SwerveDriveModule(Ports.REAR_RIGHT_MA3,Ports.REAR_RIGHT_ROTATION,Ports.REAR_RIGHT_DRIVE,4);
		nav = Navigation.getInstance();
	}
	public static DriveTrain getInstance()
    {
        if( instance == null )
            instance = new DriveTrain();
        return instance;
    }
	public void sendInput(double x, double y, double rotate){
		double angle = nav.getRawHeading()/180.0*Math.PI;
		xInput = (y * Math.sin(angle)) + (x * Math.cos(angle));
		yInput = (-y * Math.cos(angle)) + (x * Math.sin(angle));
		
		if(x==0 && y ==0){
			rotateInput = rotate;
		}else{
			rotateInput = rotate * 1.0;
		}
//		System.out.println("X1" + x + " Y1" + y + " X2" + xInput + " Y2" + yInput + " H" + nav.getHeadingInDegrees() + " cos" + Math.cos(nav.getHeadingInDegrees()));
		SmartDashboard.putNumber("X1", x);
		SmartDashboard.putNumber("Y1", y);
		SmartDashboard.putNumber("X Input", xInput);
		SmartDashboard.putNumber("Y Input", yInput);
		SmartDashboard.putNumber("Rotate Input", rotateInput);
//		reactToJoysticksWithSwerve(true);
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
			frontLeft.setGoal(135.0);
			frontLeft.setDriveSpeed(0);
			frontRight.setGoal(45.0);
			frontRight.setDriveSpeed(0.0);
			rearLeft.setGoal(45.0);
			rearLeft.setDriveSpeed(0.0);
			rearRight.setGoal(-45.0);
			rearRight.setDriveSpeed(0.0);
		}else{
			//do pid calculation and apply power to each module
			double A = xInput - rotateInput * (Constants.WHEELBASE_LENGTH / R);
	        double B = xInput + rotateInput * (Constants.WHEELBASE_LENGTH / R);
	        double C = yInput - rotateInput * (Constants.WHEELBASE_WIDTH / R);
	        double D = yInput + rotateInput * (Constants.WHEELBASE_WIDTH / R);
//			double A = 0.080456925;
//			double B = 0.919543075;
//			double C = 0.22800072;
//			double D = 0.77199928;
	        SmartDashboard.putNumber("A", A);
	        SmartDashboard.putNumber("B", B);
	        SmartDashboard.putNumber("C", C);
	        SmartDashboard.putNumber("D", D);
	        SmartDashboard.putNumber("R", R);
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
/*	        if(rotateInput == 0){
	        	double newAngle = Util.boundAngleNeg180to180Degrees(frontLeftSteeringAngle - nav.getHeadingInDegrees());
	        	frontLeft.setGoal(newAngle);
				frontRight.setGoal(newAngle);
				rearLeft.setGoal(newAngle);
				rearRight.setGoal(newAngle);
	        }else{
	        	frontLeft.setGoal(frontLeftSteeringAngle);
				frontRight.setGoal(frontRightSteeringAngle);
				rearLeft.setGoal(rearLeftSteeringAngle);
				rearRight.setGoal(rearRightSteeringAngle);
	        }*/
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
	
	
	
public void reactToJoysticksWithSwerve(boolean withGyro) {
        
        Vector2 translationVec;
        Vector2 currVec;
        double rotationVal;
        double rotationRate;
        double maxWheelVel;
        boolean goingClockwise;
        double angle;
        Vector2 zeroVec = new Vector2(0.0, 0.0);

        //translation
        currVec = new Vector2(xInput, yInput);
        if(currVec.getMagnitude()>0.2){
            translationVec = currVec;
            //previousTransVec=currVec;
        }
        else
            translationVec = zeroVec;
        //translationVec = new Vector2(rightJ.getX(), rightJ.getY());
        double speed = translationVec.getMagnitude();
        speed = Calculate.saturate(speed, 1, -1);
        //System.out.println("speed is "+speed);
        if(withGyro)
            angle = nav.getHeadingInDegrees()-translationVec.getWheelAngle();
        else
            angle = translationVec.getWheelAngle();
        //System.out.println("angle is "+angle);
        //System.out.println("(" + translationVec.getX() + "," + translationVec.getY() + ")");


        //rotation
        rotationVal = rotateInput;
        if(Math.abs(rotationVal)>0.2){
            rotationRate = rotationVal;
        }
        else{
            rotationRate = 0.0;
        }
        if (rotationRate > 0) {
            goingClockwise = true;
        } else {
            goingClockwise = false;
        }

        double vtx = speed * Math.cos(Math.toRadians(angle + 90));
        double vty = speed * Math.sin(Math.toRadians(angle + 90));

        //trans and rot combined
        double flvx = vtx - rotationRate * (-Constants.WHEELBASE_LENGTH / 2);//LEFT FRONT=FRONT LEFT
        double flvy = vty + rotationRate * (Constants.WHEELBASE_WIDTH / 2);
        double flTotalVel = Calculate.hypot(flvx, flvy);
        maxWheelVel = flTotalVel;
        //System.out.println("init lfTotalVel is "+lfTotalVel);
        //lfTotalVel = Calculate.saturate(lfTotalVel, 1, -1);
        double flTargetAngle = (flvy != 0) ? Math.toDegrees(Math.atan2(flvy, flvx)) : 0;
        

        double blvx = vtx - rotationRate * (-Constants.WHEELBASE_LENGTH / 2);//LEFT REAR = BACK LEFT
        double blvy = vty + rotationRate * (-Constants.WHEELBASE_WIDTH / 2);
        double blTotalVel = Calculate.hypot(blvx, blvy);
        if(blTotalVel>maxWheelVel)
            maxWheelVel = blTotalVel;
        //System.out.println("init lrTotalVel is "+lrTotalVel);
        //lrTotalVel = Calculate.saturate(lrTotalVel, 1, -1);
        double blTargetAngle = (blvy != 0) ? Math.toDegrees(Math.atan2(blvy, blvx)) : 0;
        

        double frvx = vtx - rotationRate * (Constants.WHEELBASE_LENGTH / 2);//RIGHT FRONT = FRONT RIGHT
        double frvy = vty + rotationRate * (Constants.WHEELBASE_WIDTH / 2);
        double frTotalVel = Calculate.hypot(frvx, frvy);
        if(frTotalVel>maxWheelVel)
            maxWheelVel = frTotalVel;
        //System.out.println("init rfTotalVel is "+rfTotalVel);
        //rfTotalVel = Calculate.saturate(rfTotalVel, 1, -1);
        double frTargetAngle = (frvy != 0) ? Math.toDegrees(Math.atan2(frvy, frvx)) : 0;
        

        double brvx = vtx - rotationRate * (Constants.WHEELBASE_LENGTH / 2);//RIGHT REAR = BACK RIGHT
        double brvy = vty + rotationRate * (-Constants.WHEELBASE_WIDTH / 2);
        double brTotalVel = Calculate.hypot(brvx, brvy);
        if(brTotalVel>maxWheelVel)
            maxWheelVel = brTotalVel;
        //System.out.println("init rrTotalVel is "+rrTotalVel);
        //rrTotalVel = Calculate.saturate(rrTotalVel, 1, -1);
        double brTargetAngle = (brvy != 0) ? Math.toDegrees(Math.atan2(brvy, brvx)) : 0;
        
        
        //scaling wheel velocities
        if(maxWheelVel >= 1)
        {
            flTotalVel = flTotalVel/maxWheelVel;
            blTotalVel = blTotalVel/maxWheelVel;
            frTotalVel = frTotalVel/maxWheelVel;
            brTotalVel = brTotalVel/maxWheelVel;
        }
        
        frontLeft.setGoal(flTargetAngle);
		frontRight.setGoal(frTargetAngle);
		rearLeft.setGoal(blTargetAngle);
		rearRight.setGoal(brTargetAngle);
		frontLeft.setDriveSpeed(-flTotalVel);
		frontRight.setDriveSpeed(frTotalVel);
		rearLeft.setDriveSpeed(-blTotalVel);
		rearRight.setDriveSpeed(brTotalVel);
    }
}