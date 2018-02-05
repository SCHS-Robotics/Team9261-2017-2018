package org.firstinspires.ftc.teamcode.season.relicrecov18.polaris.v1.robot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.robot.Robot;

/**
 * Created by Andrew on 12/6/2017.
 */

public class Polaris extends Robot {
    public Polaris(OpMode opMode){
        super(opMode);
        putSubSystem("Drive", new Drive(this));
        putSubSystem("Intake", new Intake(this));
        putSubSystem("Jewel", new Jewel(this));
        putSubSystem("Navi",new Navi(this));
        putSubSystem("JewelDetector", new JewelDetector(this));
        putSubSystem("CryptoboxRange", new CryptoboxRange(this));
        putSubSystem("FlippyBoii", new FlippyBoii(this));
    }
}
