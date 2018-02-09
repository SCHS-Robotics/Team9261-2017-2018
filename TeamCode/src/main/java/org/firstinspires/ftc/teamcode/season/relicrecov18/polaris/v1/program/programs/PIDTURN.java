package org.firstinspires.ftc.teamcode.season.relicrecov18.polaris.v1.program.programs;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.season.relicrecov18.polaris.v1.program.PolarisAutonomousProgram;

/**
 * Created by andre_000 on 02/06/2018.
 */
@Autonomous(name = "PID", group = "A")
public class PIDTURN extends PolarisAutonomousProgram {
    @Override
    protected Robot buildRobot() {
        Robot robot =  super.buildRobot();
        return robot;
    }

    @Override
    public void main() throws InterruptedException {
        waitForStart();
        drive.turnPID(0.6, 45, 1);
        drive.turnPID(0.8, 0, 1);
        drive.turnPID(0.15, 90, 1);
        drive.turnPID(0.4, 0, 1);
        drive.turnPID(0.2, 135, 1);
        drive.turnPID(0.2, 0, 1);
        drive.turnPID(0.2, 15, 1);
        sleep(3000);

    }
}
