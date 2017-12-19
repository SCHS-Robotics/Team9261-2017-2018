package org.firstinspires.ftc.teamcode.season.relicrecov18.polaris.v1.program.programs;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

/**
 * Created by Sage Creek Level Up on 12/16/2017.
 */

//It's a circle, what else is there to say?
public class Circle {
    public Point center;
    public double radius;

    public Circle(Point center, double radius) {
        this.center = center;
        this.radius = radius;
    }

    //Draws the circle onto a Mat
    public Mat draw(Mat input) {
        return draw(input, new Scalar(255,255,255), 1);
    }

    public Mat draw(Mat input, Scalar drawColor) {
        return draw(input, drawColor, 1);
    }

    public Mat draw(Mat input, Scalar drawColor, int lineWidth) {
        Imgproc.circle(input, this.center, (int) Math.round(this.radius),drawColor,lineWidth,8,0);
        Imgproc.circle(input, this.center, 3, new Scalar(0,255,0), -1, 8, 0);
        return input;
    }
}
