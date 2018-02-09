package org.firstinspires.ftc.teamcode.season.relicrecov18.polaris.v1.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;


import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.SubSystem;

/**
 * Created by Andrew on 12/6/2017.
 */

public class Intake extends SubSystem {
    private DcMotor leftintake, rightintake;
    private Servo jade,  pence, putin, hammer, gate;
    public Intake(Robot robot){
        super(robot);
    }
    boolean yPrevState, xPrevState;
    boolean shouldWall = false;
    boolean x2Prev;
    @Override
    public void init() {
        leftintake = robot.hardwareMap.dcMotor.get("leftintake");     //leftintake and rightintake are flipped
        rightintake = robot.hardwareMap.dcMotor.get("rightintake");

        leftintake.setDirection(DcMotor.Direction.FORWARD);
        rightintake.setDirection(DcMotor.Direction.REVERSE);

        jade = robot.hardwareMap.servo.get("jade");
    }

    @Override
    public void handle() {
        //leftintake.setPower(Math.abs(robot.gamepad2.left_trigger-robot.gamepad2.right_trigger)*(robot.gamepad2.left_trigger-robot.gamepad2.right_trigger));
        //rightintake.setPower(Math.abs(robot.gamepad2.left_trigger-robot.gamepad2.right_trigger)*(robot.gamepad2.left_trigger-robot.gamepad2.right_trigger));
        //leftintake.setPower((robot.gamepad2.left_stick_y-robot.gamepad2.left_stick_x));

        if(robot.gamepad2.left_bumper) {
            leftintake.setPower(1);
            rightintake.setPower(1);
        }
        else if(robot.gamepad2.right_bumper) {
            leftintake.setPower(-1);
            rightintake.setPower(-1);
        }
        else{
            leftintake.setPower(0);
            rightintake.setPower(0);
        }

        if(robot.gamepad2.x != xPrevState && robot.gamepad2.x){
            shouldWall = !shouldWall;
            jade.setPosition(jadepos());
        }

        xPrevState = robot.gamepad2.x;
    }

    public void spitintake(){
        leftintake.setPower(1);
        rightintake.setPower(1);
    }
    public void stopIntake(){
        leftintake.setPower(0);
        rightintake.setPower(0);
    }
    public void reverseintake(){
        leftintake.setPower(-1);
        rightintake.setPower(-1);
    }
    public double jadepos(){
        double position = shouldWall ? 1 : 0.5;
        return position;
    }

    @Override
    public void stop() {

    }
}
