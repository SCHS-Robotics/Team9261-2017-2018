package org.firstinspires.ftc.teamcode.season.relicrecov18.polaris.v1.robot;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.SubSystem;

/**
 * Created by andre_000 on 01/07/2018.
 */

public class CryptoboxRange extends SubSystem {
    public CryptoboxRange(Robot robot) {super(robot);}
    public ModernRoboticsI2cRangeSensor rangerLeft, rangerRight, rangerFront;
    @Override
    public void init() throws InterruptedException {
        rangerLeft = robot.hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "rangerLeft");
        rangerRight = robot.hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "rangerRight");
        rangerFront = robot.hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "rangerFront");
        rangerLeft.setI2cAddress(I2cAddr.create8bit(0x28));
        rangerFront.setI2cAddress(I2cAddr.create8bit(0x28));
        rangerRight.setI2cAddress(I2cAddr.create8bit(0x38));
    }

    @Override
    public void handle() {

    }

    @Override
    public void stop() {

    }
}
