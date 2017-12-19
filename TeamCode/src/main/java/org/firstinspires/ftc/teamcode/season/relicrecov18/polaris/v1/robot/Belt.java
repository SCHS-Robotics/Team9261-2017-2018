package org.firstinspires.ftc.teamcode.season.relicrecov18.polaris.v1.robot;

import com.qualcomm.robotcore.hardware.DcMotor;


import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.SubSystem;

/**
 * Created by Andrew on 12/6/2017.
 */

public class Belt extends SubSystem {
    private DcMotor motor5, motor6;

    public Belt(Robot robot){
        super(robot);
    }

    @Override
    public void init() {
        motor5 = robot.hardwareMap.dcMotor.get("motor5");
        motor6 = robot.hardwareMap.dcMotor.get("motor6");

        motor5.setDirection(DcMotor.Direction.FORWARD);
        motor6.setDirection(DcMotor.Direction.REVERSE);
    }

    @Override
    public void handle() {

    }
    public void spitBlock(){
        motor5.setPower(0.7);
        motor6.setPower(0.7);
    }
    public void stopSpitting(){
        motor5.setPower(0);
        motor6.setPower(0);
    }
    @Override
    public void stop() {

    }
}
