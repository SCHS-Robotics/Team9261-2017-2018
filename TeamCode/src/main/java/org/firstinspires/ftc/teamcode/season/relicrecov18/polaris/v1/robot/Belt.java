package org.firstinspires.ftc.teamcode.season.relicrecov18.polaris.v1.robot;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.SubSystem;

/**
 * Created by Andrew on 12/6/2017.
 */

public class Belt extends SubSystem {
    private DcMotor motor5, motor6, motor7, motor8;
    private CRServo leftcr, rightcr;
    private Servo trump, pence, putin, hammer, gate;
    public Belt(Robot robot){
        super(robot);
    }
    boolean yPrevState;
    boolean xPrevState;
    boolean x2Prev;
    boolean gateCheck;
    boolean clipCheck;
    boolean beltCheck;
    @Override
    public void init() {
        motor5 = robot.hardwareMap.dcMotor.get("motor5");
        motor6 = robot.hardwareMap.dcMotor.get("motor6");
        motor7 = robot.hardwareMap.dcMotor.get("motor7");
        motor8 = robot.hardwareMap.dcMotor.get("motor8");

        motor5.setDirection(DcMotor.Direction.FORWARD);
        motor6.setDirection(DcMotor.Direction.REVERSE);
        motor7.setDirection(DcMotor.Direction.FORWARD);
        motor8.setDirection(DcMotor.Direction.FORWARD);
        leftcr = robot.hardwareMap.crservo.get("leftcr");     //leftcr and rightcr are flipped
        rightcr = robot.hardwareMap.crservo.get("rightcr");
        gate = robot.hardwareMap.servo.get("greenCard");
        leftcr.setDirection(CRServo.Direction.REVERSE);
        rightcr.setDirection(CRServo.Direction.FORWARD);
        trump = robot.hardwareMap.servo.get("trump");
        pence = robot.hardwareMap.servo.get("pence");
        putin = robot.hardwareMap.servo.get("putin");
        hammer = robot.hardwareMap.servo.get("hammer");
        rigThatElection();
        clipBlock();
        gate.setPosition(1);
    }

    @Override
    public void handle() {
        if(robot.gamepad2.y != yPrevState && robot.gamepad2.y) {
            gateCheck = !gateCheck;
            gate.setPosition(gatePos());
        }
        if (robot.gamepad1.x != xPrevState && robot.gamepad1.x){
            clipCheck = !clipCheck;
            hammer.setPosition(clipPos());
        }
        if (robot.gamepad2.x != x2Prev && robot.gamepad2.x){
            beltCheck = !beltCheck;
            trump.setPosition(beltPull());
        }
        motor7.setPower(wheelPow());
        motor8.setPower(robot.gamepad1.right_stick_x);
        motor5.setPower(Math.abs(robot.gamepad2.left_trigger-robot.gamepad2.right_trigger)*(robot.gamepad2.left_trigger-robot.gamepad2.right_trigger));
        motor6.setPower(Math.abs(robot.gamepad2.left_trigger-robot.gamepad2.right_trigger)*(robot.gamepad2.left_trigger-robot.gamepad2.right_trigger));
        leftcr.setPower((robot.gamepad2.left_stick_y-robot.gamepad2.left_stick_x)*0.5);
        rightcr.setPower((robot.gamepad2.left_stick_y+robot.gamepad2.left_stick_x)*0.5);
        yPrevState = robot.gamepad2.y;
        xPrevState = robot.gamepad1.x;
        x2Prev = robot.gamepad2.x;
    }
    public double gatePos(){
        double position = gateCheck ? 1 : 0.4;
        return position;
    }
    public double clipPos(){
        double position = clipCheck ? 0.4 : 0.1;
        return position;
    }
    public double wheelPow(){
        double position = robot.gamepad2.a ? 1 : 0;
        return position;
    }
    public double beltPull(){
        double position = beltCheck ? 1 : 0;
        return position;
    }

    public void requestGate(){
        gate.setPosition(1);
    }
    public void gateGranted(){
        gate.setPosition(0);
    }
    public void intakeBlock(){
        leftcr.setPower(-.5);
        rightcr.setPower(-.5);
    }
    public void spitintake(){
        leftcr.setPower(.5);
        rightcr.setPower(0.5);
    }
    public void clipBlock(){
        hammer.setPosition(0.5);
    }
    public void unclipBlock(){
        hammer.setPosition(0.2);
    }
    public void stopIntake(){
        leftcr.setPower(0);
        rightcr.setPower(0);
    }

    public void spitBlock(){
        motor5.setPower(0.5);
        motor6.setPower(0.5);
    }
    public void stopSpitting(){
        motor5.setPower(0);
        motor6.setPower(0);
    }
    public void putThatWallUp(){
        trump.setPosition(0);
        pence.setPosition(1);
    }
    public void getThatWallBuilt(){
        trump.setPosition(0.5);
        pence.setPosition(0.5);
    }
    public void rigThatElection(){
        putin.setPosition(0);
    }
    public void electionIsRigged(){
        putin.setPosition(1);
    }
    public void deploy() {
        motor7.setPower(1);
    }
    public void stopDeploy(){
        motor7.setPower(0);
    }
    @Override
    public void stop() {

    }
}
