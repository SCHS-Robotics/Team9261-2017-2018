package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Created by Sage Creek Level Up on 10/4/2017.
 */


@TeleOp(name = "Op", group = "Linear Opmode")
public class Op extends LinearOpMode {
    DcMotor motor1;
    DcMotor motor2;

    public void runOpMode(){
        waitForStart();
        motor1 = hardwareMap.dcMotor.get("motor1");
        motor2 = hardwareMap.dcMotor.get("motor2");

        motor1.setDirection(DcMotorSimple.Direction.FORWARD);
        motor2.setDirection(DcMotorSimple.Direction.FORWARD);
        while(opModeIsActive()) {
            if (gamepad1.a){
                motor1.setPower(1);
            }
            if (gamepad1.b){
                motor2.setPower(1);
            }
            if (gamepad1.x) {
                motor1.setPower(0);
                motor2.setPower(0);

            }
        }
    }
}

