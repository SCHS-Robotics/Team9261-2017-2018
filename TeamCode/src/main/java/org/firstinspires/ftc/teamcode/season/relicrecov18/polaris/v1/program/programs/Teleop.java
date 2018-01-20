package org.firstinspires.ftc.teamcode.season.relicrecov18.polaris.v1.program.programs;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.opMode.DriverControlledProgram;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.season.relicrecov18.polaris.v1.robot.Polaris;

/**
 * Created by Andrew on 12/31/2017.
 */
@TeleOp(name = "Competition TeleOp", group = "Teleop")
public class Teleop extends DriverControlledProgram {
    private Polaris polaris;
    @Override
    protected Robot buildRobot() {
        polaris = new Polaris(this);
        return polaris;
    }
}