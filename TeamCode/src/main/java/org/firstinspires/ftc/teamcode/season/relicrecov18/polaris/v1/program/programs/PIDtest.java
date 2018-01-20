package org.firstinspires.ftc.teamcode.season.relicrecov18.polaris.v1.program.programs;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.season.relicrecov18.polaris.v1.program.PolarisAutonomousProgram;

import java.lang.reflect.Array;

/**
 * Created by andre_000 on 12/31/2017.
 */
@Autonomous(name = "PIDtest", group = "Autonomous")
public class PIDtest extends PolarisAutonomousProgram {
    @Override
    public void main() throws InterruptedException {
        while(!drive.imu.isGyroCalibrated() && !isStarted() && !isStopRequested()){
            sleep(50);
        }
        Array values[];
        drive.notifyGyroReady();
        waitForStart();
        //drive.driveMaintainYaw(25, 0.4, 0, 1);
        //sleep(5000);
        drive.strafeMaintainYaw(15, 0.4, 0, 1);
        sleep(5000);
        //drive.turnExact(0.2, 45, 1);
        //sleep(5000);

    }
}