/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package Sensors;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDSource;
/**
 *
 * @author xpsl05x
 */
public class SuperEncoder implements PIDSource {

    private Encoder enc;
    private double lastTime = 0.0;
    private double lastRead = 0.0;
    public double rpm = 0.0;      
    private int pidReturn = 0;
    private double checkDiff = 0;
    public SuperEncoder(int aChannel, int bChannel,boolean reverseDirection,int type) {
            // TODO Auto-generated constructor stub
        switch(type){
            case 1:
                pidReturn = 1;
                enc = new Encoder(aChannel,bChannel,reverseDirection,EncodingType.k2X);
                enc.setSamplesToAverage((byte)100);
                break;
            case 2:
                pidReturn = 0;
                enc = new Encoder(aChannel,bChannel,reverseDirection,EncodingType.k4X);
                break;
        }
            
    }
//Initialize the encoder
	//Return the distance as pidGet
    public double pidGet() {
        update();
        switch(pidReturn){
            case 0:
                return getRPM();
            case 1:
                return getDistance();
            default:
                return getDistance();
         
        
        } 
    }
    public void setPIDReturn(int pidSet){
        pidReturn = pidSet;
    }
    
    public void reset(){
        enc.reset();
    }
    public double getRate(){
        return enc.getRate();
    }
    public double getRaw(){
        return enc.getRaw();
    }
    public double getDistance(){
        return enc.getDistance();
    }
    public void start(){
    	
    }
    public void setDistancePerPulse(double dpp){
        enc.setDistancePerPulse(dpp);
    }
    public void update(){
        rpm = getRate()/5.208333333333333;
    }
    public double getRPM()
    {        
       return rpm;
    }
}