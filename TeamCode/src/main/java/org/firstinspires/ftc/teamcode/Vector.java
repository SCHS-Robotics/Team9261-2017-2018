package org.firstinspires.ftc.teamcode;

/**
 * Created by Sage Creek Level Up on 11/1/2017.
 */

public class Vector {
    public double x,y,r;
    public Vector(double inx, double iny){
        this.x = inx;
        this.y = iny;
        this.r = Math.sqrt(this.x*this.x + this.y*this.y);
    }
    //Counterclockwise is positive, clockwise is negative
    public Vector rotate(double θ){
        return new Vector((this.r*this.x*Math.cos(θ))-(this.r*this.y*Math.cos(θ)),(this.r*this.x*Math.sin(θ))+(this.r*this.y*Math.cos(θ)));
}
    public boolean isZeroVector(){
        return (this.x == 0.0) && (this.y == 0.0);
    }
}


/*
    new Vector(x,y);




 */