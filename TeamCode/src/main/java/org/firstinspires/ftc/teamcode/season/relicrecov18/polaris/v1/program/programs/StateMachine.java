package org.firstinspires.ftc.teamcode.season.relicrecov18.polaris.v1.program.programs;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.season.relicrecov18.polaris.v1.program.PolarisAutonomousProgram;
import org.firstinspires.ftc.teamcode.season.relicrecov18.polaris.v1.robot.Jewel;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

/**
 * Created by andre_000 on 01/06/2018.
 */

public abstract class StateMachine extends PolarisAutonomousProgram {
    @Override
    protected Robot buildRobot() {
        Robot robot = super.buildRobot();
        return robot;
    }
    public abstract void main() throws InterruptedException;
    public enum currentState{
        JewelBlue, JewelRed, DrivingoffStone, AligningCryptobox, DispensingBlocks, CollectingBlocks;
    }
    public void setCurrentState(currentState state) throws InterruptedException{
        switch(state){
            case JewelBlue:
                jewelDetector.stopOpenCV();
                belt.electionIsRigged();
                sleep(1000);
                jewel.moveAxe(Jewel.axePos.UPPERMID);
                belt.deploy();
                sleep(500);
                jewel.moveAxe(Jewel.axePos.LOWERMID);
                sleep(500);
                jewel.moveAxe(Jewel.axePos.DOWN);
                sleep(250);
                if(jewelDetector.direction.equals("left")) {
                    jewel.moveArm(Jewel.armPos.LEFT);
                }
                else if(jewelDetector.direction.equals("right")) {
                    jewel.moveArm(Jewel.armPos.RIGHT);
                }
                sleep(1000);
                belt.stopDeploy();
                jewel.moveArm(Jewel.armPos.MIDDLE);
                jewel.moveAxe(Jewel.axePos.UP);
                break;
            case JewelRed:
                jewelDetector.stopOpenCV();
                belt.electionIsRigged();
                sleep(1000);
                jewel.moveAxe(Jewel.axePos.UPPERMID);
                belt.deploy();
                sleep(500);
                jewel.moveAxe(Jewel.axePos.LOWERMID);
                sleep(500);
                jewel.moveAxe(Jewel.axePos.DOWN);
                sleep(250);
                if(jewelDetector.direction.equals("left")) {
                    jewel.moveArm(Jewel.armPos.RIGHT);
                }
                else if(jewelDetector.direction.equals("right")) {
                    jewel.moveArm(Jewel.armPos.LEFT);
                }
                sleep(1000);
                belt.stopDeploy();
                jewel.moveArm(Jewel.armPos.MIDDLE);
                jewel.moveAxe(Jewel.axePos.UP);
                break;

            case DrivingoffStone:
                belt.putThatWallUp();
                belt.intakeBlock();
                belt.spitBlock();
                sleep(1000);
                belt.rigThatElection();
                belt.stopIntake();
                belt.stopSpitting();
                belt.clipBlock();
                drive.driveoffBalancingStone();
                drive.driveMaintainYaw(15, 0.6, 0, 1);
                break;
            case DispensingBlocks:
                belt.spitBlock();
                sleep(2000);
                belt.stopSpitting();
                break;
            case AligningCryptobox:
                drive.turnExact(0.2, -45, 1);
                drive.driveMaintainYaw(38, -0.8, -45, 1 );
                drive.turnExact(0.2, 0, 1);
                /*if (navi.key == RelicRecoveryVuMark.LEFT){
                    drive.strafeMaintainYaw(6, -.5, 0, 1);
                }else if(navi.key == RelicRecoveryVuMark.RIGHT){
                    drive.strafeMaintainYaw(6, .5, 0, 1);
                }*/
                belt.unclipBlock();
                drive.driveMaintainYaw(5, -0.2, 0, 1);
                sleep(500);
                break;
            case CollectingBlocks:
                drive.driveMaintainYaw(2, 0.6, 0, 1);
                sleep(500);
                drive.driveMaintainYaw(12, 0.8, 0, 1);
                belt.intakeBlock();
                belt.spitBlock();
                drive.driveMaintainYaw(10, 0.4, 0, 1);
                belt.spitintake();
                sleep(500);
                belt.stopIntake();
                drive.driveMaintainYaw(11, 0.2, 0,1 );
                belt.stopSpitting();
                belt.clipBlock();
                break;

        }
    }
}
