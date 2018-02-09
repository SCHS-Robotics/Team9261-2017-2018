package org.firstinspires.ftc.teamcode.season.relicrecov18.polaris.v1.program.programs;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.teamcode.AutoTransitioner;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.season.relicrecov18.polaris.v1.program.PolarisAutonomousProgram;
import org.firstinspires.ftc.teamcode.season.relicrecov18.polaris.v1.robot.Jewel;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

/**
 * Created by andre_000 on 01/12/2018.
 */
@Autonomous(name = "Blue Far Auto", group = "Autonomous")
public class AutonomousBlueFar extends PolarisAutonomousProgram {
    private String direction;

    private final ExecutorService service = Executors.newFixedThreadPool(2); //The executor service that will run the two processing threads simultaneously

    //The circles previously returned by the two threads
    private Circle prevRedCircle;
    private Circle prevBlueCircle;
    @Override
    protected Robot buildRobot() {
        Robot robot = super.buildRobot();
        return robot;
    }
    @Override
    public void main() throws InterruptedException {
        /*AutoTransitioner.transitionOnStop(this, "Competition TeleOp");
        while(!drive.imu.isGyroCalibrated() && !isStarted() && !isStopRequested()){
            sleep(50);
        }
        drive.notifyGyroReady();
        navi.setupVuforia();
        while(!navi.vuforiaSetup && !isStarted() && !isStopRequested()){}
        navi.startVuforia();
        while (!isStarted() && !isStopRequested()) {
            navi.updateVuforia();
            telemetry.addData("key", navi.key);
            telemetry.update();
        }
        navi.stopVuforia();
        waitForStart();

        while(navi.vuforiaSetup){}
        jewelDetector.startOpenCV(jewelDetector);
        sleep(3000);
        jewelDetector.stopOpenCV();
        belt.electionIsRigged();
        sleep(1000);

        belt.deploy();
        sleep(2000);
        belt.stopDeploy();
        jewel.moveAxe(Jewel.axePos.UPPERMID);
        sleep(500);
        jewel.moveAxe(Jewel.axePos.LOWERMID);
        sleep(500);
        jewel.moveAxe(Jewel.axePos.DOWN);
        sleep(250);
        if(jewelDetector.direction == "left") {
            jewel.moveArm(Jewel.armPos.LEFT);
        }
        else if(jewelDetector.direction == "right") {
            jewel.moveArm(Jewel.armPos.RIGHT);
        }
        sleep(1000);
        belt.stopDeploy();
        jewel.moveArm(Jewel.armPos.MIDDLE);
        sleep(1000);
        jewel.moveAxe(Jewel.axePos.UP);
        sleep(1000);
        belt.rigThatElection();
        jewel.moveAxe(Jewel.axePos.UP);
        //belt.intakeBlock();
        //belt.spitBlock();
        //sleep(1000);
        //belt.rigThatElection();
        //belt.stopIntake();
        //belt.stopSpitting();
        //belt.clipBlock();
        //drive.driveoffBalancingStone();
        drive.driveMaintainYaw(30, 0.6, 0, 1);
        drive.turnExact(0.2, -45, 1);
        drive.driveMaintainYaw(20, -0.8, -45, 1 );
        drive.turnExact(0.2, -90, 1);
        if (navi.key == RelicRecoveryVuMark.LEFT){      //this check assumes that robot arrives at center. generally, check for left/center/right but the arrived column
            drive.strafeMaintainYaw(6, -.5, -90, 1);      //strafe left
        }else if(navi.key == RelicRecoveryVuMark.RIGHT){
            drive.strafeMaintainYaw(6, .5, -90, 1);       //strafe right
        }   //don't need else, since you'll be at desired column
        sleep(500);
        belt.unclipBlock();
        belt.gateGranted();
        drive.driveMaintainYaw(4, -0.4, -90, 1);
        sleep(500);
        belt.spitBlock();
        sleep(3000);
        belt.stopSpitting();
        drive.driveMaintainYaw(2, 0.4, -90, 1);
        */
    }
}
