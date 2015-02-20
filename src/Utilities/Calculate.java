/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
/*
   The Calculate class is a static class that defines multiple utility calculation methods
*/
package Utilities;

public class Calculate {
    public static double wrapAngle(double angle) {//wraps a given angle between -180 to 180 degrees
        return angle - 360 * Math.floor((angle + 180)/ 360);
    }
    public static double saturate(double input, double max, double min){//limit the minimum and maximum values of an input
        if(input>max)
            input = max;
        if(input<min)
            input = min;
        return input;
    }

    public static double wrapAngle360(double angle){//wraps a given angle between 0 and 360 degrees
        return angle%360;
    }
    
    public static double hypot(double x, double y){//returns the hypotenuse a triangle with given side lengths
        return Math.sqrt(Math.pow(x, 2)+Math.pow(y, 2));
    }
    
    public static Vector2 vectorAdd(Vector2 v1, Vector2 v2){//returns resultant vector when adding two given vectors 
        Vector2 v3 = new Vector2(v1.getX()+v2.getX(), v1.getY()+v2.getY());
        return v3;
    } 
    
    public static double[] addAndShift(double[] arr,double a){//add most current value to an array and shift to the right
        for(int i=0; i<arr.length-1; i++)
        {
            arr[i]=arr[i+1];
        }
        arr[arr.length-1]=a;
        return arr;
    }
    
    public static boolean checkTolerance(double[] error, double tolerance)//check if a tolerance is being met
    {
        int counter=0;
        for (int i =0; i<error.length; i++)
        {
            if (Math.abs(error[i])<tolerance)
                counter++;
        }
        if(counter==error.length)
            return true;
        else
            return false;
    }
}
