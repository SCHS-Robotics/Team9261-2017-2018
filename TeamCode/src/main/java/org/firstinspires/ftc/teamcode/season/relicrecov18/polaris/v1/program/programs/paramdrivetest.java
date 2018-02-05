package org.firstinspires.ftc.teamcode.season.relicrecov18.polaris.v1.program.programs;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.season.relicrecov18.polaris.v1.program.PolarisAutonomousProgram;

/**
 * Created by andre_000 on 12/30/2017.
 */
@Autonomous(name = "Parametric", group = "Autonomous")
public class paramdrivetest extends PolarisAutonomousProgram {
    @Override
    protected Robot buildRobot() {
        return super.buildRobot();
    }

    @Override
    public void main() throws InterruptedException {
        waitForStart();
        drive.paramDrive(15, 15, -90);
    }
}
