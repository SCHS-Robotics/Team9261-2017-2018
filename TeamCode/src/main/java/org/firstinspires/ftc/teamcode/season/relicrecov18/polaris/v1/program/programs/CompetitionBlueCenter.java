package org.firstinspires.ftc.teamcode.season.relicrecov18.polaris.v1.program.programs;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.AutoTransitioner;
import org.firstinspires.ftc.teamcode.season.relicrecov18.polaris.v1.program.PolarisAutonomousProgram;

/**
 * Created by andre_000 on 01/06/2018.
 */
@Autonomous(name = "Blue Center", group = "Autonomous")
public class CompetitionBlueCenter extends StateMachine {
    @Override
    public void main() throws InterruptedException {
        AutoTransitioner.transitionOnStop(this, "Competition TeleOp");
        while(!drive.imu.isGyroCalibrated() && !isStarted() && !isStopRequested()){
            sleep(50);
        }
        drive.notifyGyroReady();
        jewelDetector.startOpenCV(jewelDetector);
        waitForStart();
        setCurrentState(currentState.JewelBlue);
        setCurrentState(currentState.DrivingoffStone);
        setCurrentState(currentState.AligningCryptobox);
        setCurrentState(currentState.DispensingBlocks);
        setCurrentState(currentState.CollectingBlocks);
        setCurrentState(currentState.DispensingBlocks);
    }
}
