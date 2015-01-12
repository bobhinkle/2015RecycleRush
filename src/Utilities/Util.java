package Utilities;


public class Util {

    /**
     * carful with negative
     * 
     * @param val 
     * @param min
     * @param max
     * @return 
     */
    public static double speedLimit(double speed, double position, double min, double max) {
        if((position < min && speed < 0) || (position > max && speed > 0))
            return 0.0;
        else
            return speed;
    } 
    public static double getDifferenceInAngleDegrees(double from, double to)
    {
        return boundAngleNeg180to180Degrees(to-from);
    }
    public static double boundAngleNeg180to180Degrees(double angle)
    {
        // Naive algorithm
        while(angle >= 180.0)
        {
            angle -= 360.0;
        }
        while(angle < -180.0)
        {
            angle += 360.0;
        }
        return angle;
    }
    public static double pidPower(double power,double minReverse, double maxReverse, double minForward, double maxForward){
        if(maxReverse > minReverse){
            maxReverse = minReverse;
        }
        if(maxForward < minForward){
            maxForward = minForward;
        }
        if(power < 0){
            if(power > minReverse){
                return minReverse;
            }else if(power < maxReverse){
                return maxReverse;
            }else{
                return power;
            }
        }else{
            if(power < minForward){
                return minForward;
            }else if(power > maxForward){
                return maxForward;
            }else{
                return power;
            }
        }
    }

    /**
     * 
     * @param val
     * @param min
     * @param max
     * @return 
     */
    public static double limit(double val, double min, double max) {
        if (min > max) return 0.0;
        if (val > max) return max;
        if (val < min) return min;
        return val;
    } 
    
    /**
     * 
     * @param val
     * @param abs
     * @return 
     */
    public static double limit(double val, double abs){
        if (val > abs) 
            return abs;
        else if (val < -abs) 
            return -abs;
        else
            return val;
    }  
    
    public static double limit(double val){
        if (val > 1) 
            return 1;
        else if (val < -1) 
            return -1;
        else
            return val;
    }  
    
    public static double buffer(double goalValue, double storedValue, int strength) { 
        return ((storedValue * strength) + (goalValue * (100 - strength))) / 100;
    }

    public static double deadBand(double val, double deadband){
        if (val < deadband && val > -deadband) 
            return 0.0;
        else 
            return val;
    }
    public static double deadBandBump(double val, double deadband){
        if (val < deadband && val > 0){ 
            return deadband;
        }else if(val > -deadband && val < 0){
            return -deadband;
        }else{
            return val;
        }
    }
    public static boolean onTarget(double target, double current, double error){
        return ((Math.abs(current) < (Math.abs(target)+ Math.abs(error))) && (Math.abs(current) > (Math.abs(target)- Math.abs(error))));
    }
    public static boolean inRange(double val, double maxAbsError) {
        return (Math.abs(val) < maxAbsError);
    }

    public static boolean inRange(double val, double minError, double maxError) {
        return (val > minError && val < maxError);
    }
    
    public static double aTan(double opp, double adj) {
        return Math.toDegrees(Math.atan2(opp, adj)); 
    }
    
    public static double aSin(double opp, double hyp) {
        return Math.toDegrees(Math.asin(opp / hyp)); 
    }
   
    public static double boundAngle0to360Degrees(double angle)
    {
        // Naive algorithm
        while(angle >= 360.0)
        {
            angle -= 360.0;
        }
        while(angle < 0.0)
        {
            angle += 360.0;
        }
        return angle;
    }
    
    public static double scale(double x, double from_min, double from_max, double to_min, double to_max)
    {
        if(x < from_min)
            x = from_min;
        else if(x > from_max)
            x = from_max;
        return ((x-from_min)*(to_max-to_min)/(from_max-from_min)) + to_min;
    }
}