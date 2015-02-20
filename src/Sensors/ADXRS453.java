package Sensors;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.Timer;

public class ADXRS453{
	private double WARM_UP_PERIOD = 5.0;
	private double CALIBRATE_PERIOD = 15.0;
	private static final int DATA_SIZE = 4;
	private static final int PARITY_BIT = 1;
	private static final int FIRST_BYTE_DATA = 0x3;
	private static final int THIRD_BYTE_DATA = 0xFC;
	private static final int READ_COMMAND = 0x20;
	private double accumulated_angle;
	private Timer update_timer;
	private Timer calibration_timer;
	private double current_rate;
	private double accumulated_offset;
	private double rate_offset;
	private byte command[];
	private byte data[];
	private short combinedData;
	SPI spi;
	private int sensor_output_1[];
	private int sensor_output_2[];
	private int sensor_output_3[];
	private int sensor_output_4[];
	private double lastTime;
	private double thisTime;
	private int iLoop;
	
	public ADXRS453(){
		spi = new SPI(Port.kOnboardCS0);
		spi.setClockRate(4000000);
		spi.setClockActiveHigh();
		spi.setChipSelectActiveLow();
		spi.setMSBFirst();
		command = new byte[4];
		command[0] = READ_COMMAND;
		command[1] = 0;
		command[2] = 0;
		command[3] = 0;
		data = new byte[4];
		data[0] = 0;
		data[1] = 0;
		data[2] = 0;
		data[3] = 0;
		iLoop = 0;
		
		accumulated_angle = 0;
		current_rate = 0.0;
		accumulated_offset = 0.0;
		rate_offset = 0.0;
		update_timer = new Timer();
		update_timer.start();
		calibration_timer = new Timer();
		calibration_timer.start();
		
		
	}
	
	public double getRate(){
		return current_rate;
	}
	public double getAngle(){
		return accumulated_angle;
	}
	public void reset(){
		data[0] = 0;
		data[1] = 0;
		data[2] = 0;
		data[3] = 0;
		current_rate = 0.0;
		accumulated_angle = 0.0;
		rate_offset = 0.0;
		accumulated_offset = 0.0;

		//calibration_timer->Stop();
		calibration_timer.reset();
		//update_timer->Stop();
		update_timer.reset();
	}
	public void update(){
		spi.transaction(command, data, DATA_SIZE);
		double cal_time = calibration_timer.get();
		if(cal_time < WARM_UP_PERIOD){
			lastTime = thisTime = update_timer.get();
			return;
		}else if(cal_time < CALIBRATE_PERIOD){
			calibrate();
		}else{
			updateData();
		}
	}
	public void calculateData(){
		combinedData = (short) (((short)(data[0] & (short)FIRST_BYTE_DATA)) << 14 | ((short) data[1]) << 6 | ((short) (data[2] & (short)THIRD_BYTE_DATA)) >> 2);
	}
	public double offset(){
		return rate_offset;
	}
	public void start(){
		
	}
	public void stop(){
		
	}
	private void updateData(){
		
	}
	private void calibrate(){
		
	}
}