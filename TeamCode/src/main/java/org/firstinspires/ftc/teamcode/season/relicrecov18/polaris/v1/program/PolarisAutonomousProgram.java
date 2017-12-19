package org.firstinspires.ftc.teamcode.season.relicrecov18.polaris.v1.program;

import org.firstinspires.ftc.teamcode.opMode.AutonomousProgram;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.season.relicrecov18.polaris.v1.robot.Belt;
import org.firstinspires.ftc.teamcode.season.relicrecov18.polaris.v1.robot.Drive;
import org.firstinspires.ftc.teamcode.season.relicrecov18.polaris.v1.robot.Jewel;
import org.firstinspires.ftc.teamcode.season.relicrecov18.polaris.v1.robot.Polaris;
import org.firstinspires.ftc.teamcode.season.relicrecov18.polaris.v1.robot.Navi;

/**
 * Created by Andrew on 12/7/2017.
 */

public abstract class PolarisAutonomousProgram extends AutonomousProgram {
    public Drive drive;
    public Belt belt;
    public Jewel jewel;
    public Navi navi;

    @Override
    protected Robot buildRobot() {
        Polaris polaris = new Polaris(this);

        drive = (Drive) polaris.getSubSystem("Drive");
        belt = (Belt) polaris.getSubSystem("Belt");
        jewel = (Jewel) polaris.getSubSystem("Jewel");
        navi = (Navi) polaris.getSubSystem("Navi");

        return polaris;
    }
}
