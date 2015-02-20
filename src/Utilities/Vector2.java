/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

package Utilities;


/**
 *
 * @author Riley
 */
public class Vector2 {
    private double x, y;
    
    public Vector2(double horiz, double vert) {//create a 2 dimensional vector with given x and y
        x = horiz;
        y = vert;
    }
    
    public double getX() {//return the x value of a given 2D vector
        return x;
    }
    
    public double getY() {//return the y value of a given 2D vector
        return y;
    }
    
    public double getMagnitude() {//return the magnitude of a given 2D vector
        return Math.sqrt(x*x + y*y);
    }
    
    public double getAngle() {//return the angle of a given 2D vector
        return Math.toDegrees(Math.atan2(y, x));
    }
    
    public double getWheelAngle() {//return the angle of a given 2D vector, adjusted for the discrepancy between 
    return Calculate.wrapAngle(90 - Math.toDegrees(Math.atan2(y, x)));//"mathematical zero" and "robot zero"
    }
    
    public void addVector(Vector2 v) {//add another 2D vector to a given vector
        x += v.getX();
        y += v.getY();
    }
    
    
}