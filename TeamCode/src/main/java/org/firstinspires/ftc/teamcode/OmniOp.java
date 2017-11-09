package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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
        motord = hardwareMap.dcMotor.get("motor3");
        motorc = hardwareMap.dcMotor.get("motor4");

        motora.setDirection(DcMotorSimple.Direction.FORWARD);
        motorb.setDirection(DcMotorSimple.Direction.FORWARD);
        motorc.setDirection(DcMotorSimple.Direction.REVERSE);
        motord.setDirection(DcMotorSimple.Direction.REVERSE);

    }
    @Override
    public void init_loop(){

    }
    @Override
    public void start(){

    }
    @Override
    public void loop(){
        Vector inputVector = new Vector(gamepad1.left_stick_x,-gamepad1.left_stick_y);
        Vector motionVector = inputVector.rotate(-Math.PI/4);
        if(gamepad1.left_bumper && motionVector.isZeroVector()) {
            motora.setPower(1);
            motorb.setPower(-1);
            motorc.setPower(1);
            motord.setPower(-1);
        }
        else if(gamepad1.right_bumper && motionVector.isZeroVector()) {
            motora.setPower(-1);
            motorb.setPower(1);
            motorc.setPower(-1);
            motord.setPower(1);
        }
        else {
            telemetry.addData("inX:", inputVector.x);
            telemetry.addData("inY:", inputVector.y);
            telemetry.addData("X:", motionVector.x);
            telemetry.addData("Y:", motionVector.y);
            telemetry.addData("blah", inputVector.y*Math.cos(45));
            telemetry.update();
            motora.setPower(motionVector.x);
            motorb.setPower(motionVector.y);
            motorc.setPower(motionVector.x);
            motord.setPower(motionVector.y);
        }
    }
}
