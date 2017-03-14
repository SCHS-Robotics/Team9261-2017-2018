package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Created by Andrew on 3/6/2017.
 */
public abstract class BaseAutonomous extends LinearOpMode
{
    RobotTankDrive drive = new RobotTankDrive();

    @Override
    public void runOpMode() throws InterruptedException
    {
        drive.initDrive(this);
        waitForStart();
        drive.drivewithPID(0.7, 30);
    }
}


