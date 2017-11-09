package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.Range;

import static java.lang.Math.cos;
import static java.lang.Math.pow;
import static java.lang.Math.sin;
import static java.lang.Math.sqrt;

/**
 * Created by Sage Creek Level Up on 10/22/2017.
 */

public class Direction_Vector {
    double magnitude;
    double angle;
    double xmagnitude;
    double ymagnitude;
    public Direction_Vector(double magnitudex, double magnitudey, double angle) {
        this.magnitude = sqrt(pow(magnitudex,2)+pow(magnitudey,2));
        this.angle = angle;
        this.xmagnitude = magnitudex;
        this.ymagnitude = magnitudey;
    }
}
