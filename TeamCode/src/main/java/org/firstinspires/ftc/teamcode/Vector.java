package org.firstinspires.ftc.teamcode;

/**
 * Created by Cole Savage on 7/11/2017.
 */
public class Vector {
    public double x,y,theta;
    public Vector(double inx, double iny) {
        this.x = inx;
        this.y = iny;
        if(inx > 0) {
            this.theta = Math.atan(iny/inx);
        }
        else if(inx < 0) {
            this.theta = Math.atan(iny/inx) + Math.PI;
        }
        else if(iny > 0) {
            this.theta = Math.PI/2;
        }
        else if(iny < 0) {
            this.theta = -Math.PI/2;
        }
        else {
            this.theta = 0;
        }
        this.theta = this.theta > 0 ? this.theta : this.theta + 2*Math.PI; //To make everything positive, because I don't like negative angles as much
    }
    //Counterclockwise is positive, clockwise is negative
    //must be in radians
    public Vector rotate(double theta) {
        return new Vector((this.x*Math.cos(theta))-(this.y*Math.sin(theta)), (this.x*Math.sin(theta))+(this.y*Math.cos(theta)));
    }
    public boolean isZeroVector() {
        return (this.x == 0.0) && (this.y == 0.0);
    }
    //maybe used for Andrew's standarization idea later
    public Vector normalize(double length) {
        return new Vector(length*Math.cos(this.theta), length*Math.sin(this.theta));
    }
    public Vector normalize() {
        return normalize(1.0);
    }
}