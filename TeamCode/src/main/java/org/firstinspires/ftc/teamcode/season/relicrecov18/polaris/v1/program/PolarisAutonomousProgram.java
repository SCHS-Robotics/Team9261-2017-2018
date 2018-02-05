package org.firstinspires.ftc.teamcode.season.relicrecov18.polaris.v1.program;

import org.firstinspires.ftc.teamcode.opMode.AutonomousProgram;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.season.relicrecov18.polaris.v1.robot.FlippyBoii;
import org.firstinspires.ftc.teamcode.season.relicrecov18.polaris.v1.robot.Intake;
import org.firstinspires.ftc.teamcode.season.relicrecov18.polaris.v1.robot.CryptoboxRange;
import org.firstinspires.ftc.teamcode.season.relicrecov18.polaris.v1.robot.Drive;
import org.firstinspires.ftc.teamcode.season.relicrecov18.polaris.v1.robot.Jewel;
import org.firstinspires.ftc.teamcode.season.relicrecov18.polaris.v1.robot.JewelDetector;
import org.firstinspires.ftc.teamcode.season.relicrecov18.polaris.v1.robot.Polaris;
import org.firstinspires.ftc.teamcode.season.relicrecov18.polaris.v1.robot.Navi;

/**
 * Created by Andrew on 12/7/2017.
 */

public abstract class PolarisAutonomousProgram extends AutonomousProgram {
    public Drive drive;
    public Intake belt;
    public Jewel jewel;
    public Intake intake;
    public Navi navi;
    public JewelDetector jewelDetector;
    public CryptoboxRange ranger;
    public FlippyBoii flippyBoii;

    @Override
    protected Robot buildRobot() {
        Polaris polaris = new Polaris(this);

        drive = (Drive) polaris.getSubSystem("Drive");
        belt = (Intake) polaris.getSubSystem("Intake");
        jewel = (Jewel) polaris.getSubSystem("Jewel");
        navi = (Navi) polaris.getSubSystem("Navi");
        intake = (Intake) polaris.getSubSystem("Intake");
        jewelDetector = (JewelDetector) polaris.getSubSystem("JewelDetector");
        ranger = (CryptoboxRange) polaris.getSubSystem("CryptoboxRange");
        flippyBoii = (FlippyBoii) polaris.getSubSystem("FlippyBoii");

        return polaris;
    }
}
