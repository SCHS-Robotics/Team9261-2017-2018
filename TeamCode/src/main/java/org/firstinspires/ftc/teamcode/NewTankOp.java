package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by Andrew on 3/4/2017.
 */
@TeleOp (name = "New Tank Op", group = "Main")
public class NewTankOp extends LinearOpMode
{
    RobotTankDrive drive = new RobotTankDrive();

    @Override
    public void runOpMode() throws InterruptedException
    {
        drive.initDrive(this);

        while (!isStarted())
        {
            telemetry.addData(">", "Press Start");

            telemetry.update();
        }

        while (opModeIsActive())
        {
            drive.manualDrive();
            drive.moveRobot();
            telemetry.update();
        }
        telemetry.addData(">", "Shutting Down. Bye!");
        telemetry.update();
    }
}
