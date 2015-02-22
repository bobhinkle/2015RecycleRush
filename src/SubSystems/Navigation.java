package SubSystems;

import Sensors.GyroThread;
import Sensors.SuperEncoder;
import Utilities.Constants;
import Utilities.Ports;
import Utilities.Util;
import edu.wpi.first.wpilibj.Gyro;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Navigation implements PIDSource{
	
	   // Sensors
    protected SuperEncoder followerWheel;
    protected GyroThread gyro;
//    Gyro gyro;
    Gyro gyro2;
    // Navigational state
    private double x = 0.0; // positive from driver facing center of the field
    private double y = 0.0; // positive from driver looking left
    private static Navigation instance;
    private double basicDistance = 0;
    private double topGoalPosition = 360;
    private boolean topGoalFound = false;
    private boolean twoGyro = false;
    private double angle = 0;
    private Navigation()
    {
        followerWheel = new SuperEncoder(Ports.NAV_X_ENC,Ports.NAV_X_ENC+1,false,1);
        followerWheel.setDistancePerPulse(Constants.DRIVE_DISTANCE_PER_PULSE);
        followerWheel.start();
//        lights = Lights.getInstance();
//        lights.setState(Lights.GYRO_INIT);
        gyro = new GyroThread();
        gyro.start();
//        lights.setState(Lights.GYRO_COMP);
//        gyro = new Gyro(Ports.GYRO);
//        gyro.initGyro();
        if(twoGyro){
	        gyro2 = new Gyro(Ports.GYRO2);
	        gyro2.initGyro();
        }
    }
    public static Navigation getInstance()
    {
        if( instance == null )
        {
            instance = new Navigation();
        }
        return instance;
    }
    public void initGyro(){
        System.out.println("init");
        SmartDashboard.putString("GYRO_STATUS", "INITIALIZING");
        System.out.println("init done");
        SmartDashboard.putString("GYRO_STATUS", "READY");
    }
    public synchronized void resetRobotPosition(double x, double y, double theta,boolean gyroReset)
    {
        this.x = x;
        this.y = y;
        followerWheel.reset();
        if(gyroReset){
            gyro.rezero();
//        	gyro.reset();
        }
        basicDistance = 0;
    }
    
    public synchronized double getX()
    {
        return x;
    }

    public synchronized double getY()
    {
        return y;
    }

    public double getHeadingInDegrees()
    {
        return Util.boundAngle0to360Degrees(gyro.getAngleInDegrees());
    }
    public double getRawHeading(){
//        return gyro.getAngle();
        return angle;
    }

    public double getPitchInDegrees()
    {
        //return gyro.getAngle();
        return 0;
    }

    public void resetPitch()
    {
        gyro.rezero();
//    	gyro.reset();
    }

    public synchronized void run()
    {
        updatePosition();
        SmartDashboard.putNumber("Distance",getDistance());
        SmartDashboard.putNumber("RawDistance",followerWheel.getRaw());
        SmartDashboard.putNumber("Heading",getHeadingInDegrees());
        SmartDashboard.putNumber("RawHeading",getRawHeading());
        if(twoGyro){
        	
        }
    }

    public double getFollowerWheelDistance()
    {
        return followerWheel.getDistance();
    }

    public double getDistance(){
        return basicDistance;
    }
    public void updatePosition()
    {
//        basicDistance = (followerWheel.getDistance() + rightDriveEncoder.getDistance())/2.0;
        basicDistance = followerWheel.getDistance();
        if(twoGyro){
            angle = (gyro.getAngle() + gyro2.getAngle())/1.0;
            SmartDashboard.putNumber("GYRO_HEADING2", gyro2.getAngle());
        }else{
            angle = gyro.getAngleInDegrees();
        }
        SmartDashboard.putNumber("GYRO_HEADING", gyro.getAngleInDegrees());
        /*
        double distanceTravelled = ((followerWheel.getDistance() + rightDriveEncoder.getDistance())/2.0) - distanceLast;
        double timePassed = System.currentTimeMillis() - timeLast;
        speedX = distanceTravelled/timePassed;
        x += distanceTravelled * Math.cos(Math.toRadians(getHeadingInDegrees()));
        y += distanceTravelled * Math.sin(Math.toRadians(getHeadingInDegrees()));

        distanceLast = (followerWheel.getDistance() + rightDriveEncoder.getDistance())/2.0;
        timeLast = System.currentTimeMillis();*/
    }
    public double pidGet() {
        return basicDistance;
    }
    
    public class Distance implements PIDSource {
    
        public double pidGet(){
            return basicDistance = followerWheel.getDistance();
        }
    }
	
	private class L3GD20H{
		private I2C i2c;
		private final int address  = 0xD6;
		private final int L3GD20_POLL_TIMEOUT = 100;         // Maximum number of read attempts
	    private final int L3GD20_ID           = 0xD4;
	    private final int L3GD20H_ID          = 0xD7;
	    private final double GYRO_SENSITIVITY_250DPS  = 0.00875;    // Roughly 22/256 for fixed point match
	    private final double GYRO_SENSITIVITY_500DPS  = 0.0175;     // Roughly 45/256
	    private final double GYRO_SENSITIVITY_2000DPS = 0.070;      // Roughly 18/256
	    
	    private final int CTRL_REG1 = 0x20;
	    private final int CTRL_REG2 = 0x21;
	    private final int CTRL_REG  = 0x22;
	    private final int CTRL_REG4 = 0x23;
	    private final int CTRL_REG5 = 0x24;
	    private final int OUT_X_L   = 0x28;
	    private final int OUT_X_H   = 0x29;
	    private final int OUT_Y_L   = 0x2A;
	    private final int OUT_Y_H   = 0x2B;
	    private final int OUT_Z_L   = 0x2C;
	    private final int OUT_Z_H   = 0x2D;
	    private byte[] x,y,z;
	    
		public L3GD20H(){
			i2c = new I2C(Port.kOnboard,address);
			x = new byte[2];
			y = new byte[2];
			z = new byte[2];
			i2c.write(CTRL_REG1, 0x0F);
			Timer.delay(0.1);
			i2c.write(CTRL_REG4, 0x20);
			Timer.delay(0.1);
		}
		public void getXValue(){
			byte[] value = new byte[1];
			i2c.write(address,OUT_X_L);
			Timer.delay(0.04);
			i2c.read(OUT_X_L, 1, value);
			Timer.delay(0.04);
			x[1] = value[0];
			i2c.write(address,OUT_X_H);
			Timer.delay(0.04);
			i2c.read(OUT_X_L, 1, value);
			x[0] = value[0];
		}
		byte[] ReadRegister(int Register){
			byte[] result = new byte[2];
			
			return(result);  
			}
	}
	
	
	
}