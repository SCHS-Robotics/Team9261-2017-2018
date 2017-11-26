package org.firstinspires.ftc.teamcode;



import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
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

    @Override
    public void init() {

        motora = hardwareMap.dcMotor.get("motor1");
        motorb = hardwareMap.dcMotor.get("motor2");
        motorc = hardwareMap.dcMotor.get("motor4");
        motord = hardwareMap.dcMotor.get("motor3");

        motora.setDirection(DcMotorSimple.Direction.REVERSE);
        motorb.setDirection(DcMotorSimple.Direction.FORWARD);
        motorc.setDirection(DcMotorSimple.Direction.FORWARD);
        motord.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    @Override
    public void init_loop() {

    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {
        Vector inputVector = new Vector(gamepad1.left_stick_x,-gamepad1.left_stick_y); //gamepads are weird, -1 is at the top of the y axis
        Vector motionVector = inputVector.rotate(-Math.PI/4);

        if(gamepad1.left_bumper && motionVector.isZeroVector()) {
            motora.setPower(-1);
            motorb.setPower(1);
            motorc.setPower(1);
            motord.setPower(-1);
        }

        else if(gamepad1.right_bumper && motionVector.isZeroVector()) {
            motora.setPower(1);
            motorb.setPower(-1);
            motorc.setPower(-1);
            motord.setPower(1);
        }

        else {

            double offset = (gamepad1.left_trigger-gamepad1.right_trigger)/4;

            motora.setPower(Range.clip(motionVector.x - offset,-1,1));
            motorb.setPower(Range.clip(motionVector.y + offset,-1,1));
            motorc.setPower(Range.clip(motionVector.x + offset,-1,1));
            motord.setPower(Range.clip(motionVector.y - offset,-1,1));
        }
    }
}
