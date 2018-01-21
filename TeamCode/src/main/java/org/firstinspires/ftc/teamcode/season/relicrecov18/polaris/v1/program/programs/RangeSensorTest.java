package org.firstinspires.ftc.teamcode.season.relicrecov18.polaris.v1.program.programs;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.AutoTransitioner;
import org.firstinspires.ftc.teamcode.season.relicrecov18.polaris.v1.program.PolarisAutonomousProgram;

/**
 * Created by andre_000 on 01/07/2018.
 */
@Autonomous(name = "RangerDanger", group = "meme")
public class RangeSensorTest extends PolarisAutonomousProgram {
    @Override
    public void main() throws InterruptedException {
        drive.initGyro();
        while(!drive.imu.isGyroCalibrated() && !isStarted() && !isStopRequested()){
            sleep(50);
        }
        drive.notifyGyroReady();
        drive.readSensorsSetUp();
        int i = 0;
        waitForStart();
        while(opModeIsActive()){
            drive.strafeMaintainYawUntilDistance(132, -0.4, 0);
        }
    }
}
