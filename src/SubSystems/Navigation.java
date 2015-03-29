package SubSystems;

import Sensors.GyroThread;
import Sensors.SuperEncoder;
import Utilities.Constants;
import Utilities.Ports;
import Utilities.Util;
import edu.wpi.first.wpilibj.Gyro;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Navigation implements PIDSource{
	
	// Sensors
    protected SuperEncoder followerWheelX;
    protected SuperEncoder followerWheelY;
    protected GyroThread gyro;
    Gyro gyro2;
    // Navigational state
    private double x = 0.0; // positive from driver facing center of the field
    private double y = 0.0; // positive from driver looking left
    private static Navigation instance;
    private double basicDistance = 0;
    private boolean twoGyro = false;
    private double angle = 0;
    private double STARTING_ANGLE_OFFSET = 0.0;
    private Navigation()
    {
        followerWheelX = new SuperEncoder(Ports.NAV_X_ENC,Ports.NAV_X_ENC+1,true,1);
        followerWheelX.setDistancePerPulse(Constants.DRIVE_DISTANCE_PER_PULSE);
        followerWheelX.start();
        followerWheelY = new SuperEncoder(Ports.NAV_Y_ENC,Ports.NAV_Y_ENC+1,true,1);
        followerWheelY.setDistancePerPulse(Constants.DRIVE_DISTANCE_PER_PULSE);
        followerWheelY.start();
        gyro = new GyroThread();
        gyro.start();
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
        followerWheelX.reset();
        followerWheelY.reset();
        if(gyroReset){
            gyro.reset();
        }
        basicDistance = 0;
        STARTING_ANGLE_OFFSET = 0;
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
    public double getRawHeadingInDegrees(){
    	return Util.radsToDegrees(angle);
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
        SmartDashboard.putNumber("X",getX());
        SmartDashboard.putNumber("Y",getY());
        SmartDashboard.putNumber("RawDistanceX",followerWheelX.getRaw());
        SmartDashboard.putNumber("RawDistanceY",followerWheelY.getRaw());
        SmartDashboard.putNumber("Heading",getHeadingInDegrees());
        SmartDashboard.putNumber("RawHeading",getRawHeading());        
    }

    public double getFollowerWheelDistance()
    {
        return followerWheelX.getDistance();
    }

    public double getDistance(){
        return basicDistance;
    }
    public void updatePosition()
    {
    	y = followerWheelY.getDistance();
        x = followerWheelX.getDistance();
        if(twoGyro){
            angle = (gyro.getAngle() + gyro2.getAngle())/1.0;
            SmartDashboard.putNumber("GYRO_HEADING2", gyro2.getAngle());
        }else{
            angle = gyro.getAngleInDegrees() + STARTING_ANGLE_OFFSET;
        }
        SmartDashboard.putNumber("GYRO_HEADING", gyro.getAngleInDegrees());
    }
    public double pidGet() {
        return getY();
    }
    
    public class Distance implements PIDSource {
        public double pidGet(){
            return basicDistance = followerWheelY.getDistance();
        }
    }
}