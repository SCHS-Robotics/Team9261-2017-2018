package org.firstinspires.ftc.teamcode;



import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Vector;

/**

 * Created by Sage Creek Level Up on 11/8/2017.

 */

@TeleOp(name = "Omni Op", group = "TeleOp")

public class OmniOp extends OpMode {
    private DcMotor motora;
    private DcMotor motorb;
    private DcMotor motorc;
    private DcMotor motord;

    private DcMotor motor5;
    private DcMotor motor6;
    private DcMotor motor7;

    //remember to have servos inside
    private CRServo leftcr;
    private CRServo rightcr;
    private Servo jewelCR;
    private Servo axe;

    @Override
    public void init() {

        motora = hardwareMap.dcMotor.get("motor1");
        motorb = hardwareMap.dcMotor.get("motor2");
        motorc = hardwareMap.dcMotor.get("motor4");
        motord = hardwareMap.dcMotor.get("motor3");

        motor5 = hardwareMap.dcMotor.get("motor5");
        motor6 = hardwareMap.dcMotor.get("motor6");
        motor7 = hardwareMap.dcMotor.get("motor7");

        leftcr = hardwareMap.crservo.get("leftcr");     //leftcr and rightcr are flipped
        rightcr = hardwareMap.crservo.get("rightcr");

        jewelCR = hardwareMap.servo.get("jewelcr");
        axe = hardwareMap.servo.get("axe");

        leftcr.setDirection(CRServo.Direction.REVERSE);
        rightcr.setDirection(CRServo.Direction.FORWARD);

        motora.setDirection(DcMotor.Direction.REVERSE);
        motorb.setDirection(DcMotor.Direction.FORWARD);
        motorc.setDirection(DcMotor.Direction.FORWARD);
        motord.setDirection(DcMotor.Direction.REVERSE);

        motor5.setDirection(DcMotor.Direction.FORWARD);
        motor6.setDirection(DcMotor.Direction.REVERSE);
        motor7.setDirection(DcMotor.Direction.FORWARD);

    }
    @Override
    public void init_loop() {

    }

    @Override
    public void start() {
        axe.setPosition(0.6);
        jewelCR.setPosition(0.35);
    }

    @Override
    public void loop() {

        Vector inputVector = new Vector(gamepad1.left_stick_x, -gamepad1.left_stick_y); //gamepads are weird, -1 is at the top of the y axis
        Vector motionVector = inputVector.rotate(-Math.PI / 4);

        if (gamepad2.left_bumper && gamepad2.x) {
            motor5.setPower(1);
            motor6.setPower(1);
        }
        else if (gamepad2.left_bumper) {
            motor5.setPower(0.7);
            motor6.setPower(0.65);
        }
        else if (gamepad2.right_bumper) {
            motor5.setPower(-0.7);
            motor6.setPower(-0.7);
        }
        else{
            motor5.setPower(0);
            motor6.setPower(0);
        }

        if(gamepad2.a){
            motor7.setPower(1);
        }else{
            motor7.setPower(0);
        }

        if (gamepad1.left_bumper && motionVector.isZeroVector()) {
            motora.setPower(-1);
            motorb.setPower(1);
            motorc.setPower(1);
            motord.setPower(-1);
        }
        else if (gamepad1.right_bumper && motionVector.isZeroVector()) {
            motora.setPower(1);
            motorb.setPower(-1);
            motorc.setPower(-1);
            motord.setPower(1);
        }
        else if(gamepad1.dpad_left) {
            motora.setPower(-1);
            motorb.setPower(1);
            motorc.setPower(-1);
            motord.setPower(1);
        }
        else if(gamepad1.dpad_right) {
            motora.setPower(1);
            motorb.setPower(-1);
            motorc.setPower(1);
            motord.setPower(-1);
        }
        else {
            double offset = (gamepad1.left_trigger - gamepad1.right_trigger) / 2;
            motora.setPower(Range.clip(motionVector.x - offset, -1, 1));
            motorb.setPower(Range.clip(motionVector.y + offset, -1, 1));
            motorc.setPower(Range.clip(motionVector.x + offset, -1, 1));
            motord.setPower(Range.clip(motionVector.y - offset, -1, 1));
        }
        leftcr.setPower(Range.clip(gamepad2.right_trigger-gamepad2.left_trigger, -.87, .87));
        rightcr.setPower(Range.clip(gamepad2.right_trigger-gamepad2.left_trigger, -.87, .87));
    }
}