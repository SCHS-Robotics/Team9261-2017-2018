package org.firstinspires.ftc.teamcode.season.relicrecov18.polaris.v1.program.programs;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.teamcode.AutoTransitioner;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.season.relicrecov18.polaris.v1.program.PolarisAutonomousProgram;
import org.firstinspires.ftc.teamcode.season.relicrecov18.polaris.v1.robot.Jewel;

/**
 * Created by andre_000 on 01/12/2018.
 */
@Autonomous (name = "Red Far Auto", group = "Autonomous")
public class AutonomousRedFar extends PolarisAutonomousProgram {
        String direction;
        String key;
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
            key = navi.key.toString();
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
                jewel.moveArm(Jewel.armPos.RIGHT);
            }
            else if(jewelDetector.direction == "right") {
                jewel.moveArm(Jewel.armPos.LEFT);
            }
            sleep(1000);
            belt.stopDeploy();
            jewel.moveArm(Jewel.armPos.MIDDLE);
            sleep(1500);
            jewel.moveAxe(Jewel.axePos.UP);
            sleep(2000);
            belt.rigThatElection();
            jewel.moveAxe(Jewel.axePos.UP);
            /*belt.intakeBlock();
            belt.spitBlock();
            sleep(1000);
            belt.rigThatElection();
            belt.stopIntake();
            belt.stopSpitting();
            belt.clipBlock();
            //drive.driveoffBalancingStone();*/
            /*drive.driveMaintainYaw(28, 0.6, 0, 1);
            drive.turnExact(0.2, 45, 1);
            drive.driveMaintainYaw(24, -0.8, 45, 1 );
            drive.turnExact(0.2, 90, 1);
            if (key == RelicRecoveryVuMark.LEFT.toString()){      //this check assumes that robot arrives at center. generally, check for left/center/right but the arrived column
                drive.strafeMaintainYaw(6, -.5, 90, 1);      //strafe left
            }else if(key == RelicRecoveryVuMark.RIGHT.toString()){
                drive.strafeMaintainYaw(6, .5, 90, 1);       //strafe right
            }   //don't need else, since you'll be at desired column
            sleep(500);
            belt.unclipBlock();
            belt.gateGranted();
            sleep(500);
            belt.spitBlock();
            sleep(3000);
            belt.stopSpitting();
            drive.driveMaintainYaw(2, 0.4, 90, 1);
            */
    }
}
