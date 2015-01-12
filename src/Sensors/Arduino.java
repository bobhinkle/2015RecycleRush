package Sensors;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.SerialPort.Port;

public class Arduino {
    private static Arduino instance = null;
    private SerialPort serial;

    public Arduino() {
        try {
            serial = new SerialPort(115200,Port.kUSB);
            this.serial.disableTermination();
        }
        catch (Exception e) {
            System.out.println("something went wrong, " + e.getMessage());
        }
    }
    public static Arduino getInstance()
    {
        if( instance == null )
            instance = new Arduino();
        return instance;
    }
    public String getData() {
        try {
            return this.serial.readString();
        } catch (Exception e) {
            System.out.println("something went wrong, " + e.getMessage());
            return null;
        }
    }

    public boolean sendData(byte[] buffer) throws Exception {
        try {
            int count = buffer.length;
            this.serial.write(buffer, count);
            return true;
        } catch (Exception e) {
            System.out.println("something went wrong, " + e.getMessage());
            return false;
        }
    }
    
    public boolean printf(String data) {
        try {
            this.serial.writeString(data);
            return true;
        } catch (Exception e) {
            System.out.println("something went wrong, " + e.getMessage());
            return false;
        }
    }

    public String requestData() {
        try {
            this.serial.writeString("r");
            return this.serial.readString();
        } catch (Exception e) {
            System.out.println("something went wrong, " + e.getMessage());
            return null;
        }
    }

    public int requestData(String request) {
        try {
            this.serial.writeString(request);
            return Integer.parseInt(this.getData());
        } catch (Exception e) {
            System.out.println("something went wrong, " + e.getMessage());
            return 0;
        }
    }
}