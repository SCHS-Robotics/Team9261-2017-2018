package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Created by Sage Creek Level Up on 9/27/2017.
 */
@TeleOp(name = "DaBest", group = "Linear Opmode")
public class TestOp extends LinearOpMode {
    DcMotor motor1;
    DcMotor motor2;

    public void runOpMode(){
        waitForStart();
        motor1 = hardwareMap.dcMotor.get("motor1");
        motor2 = hardwareMap.dcMotor.get("motor2");

        motor1.setDirection(DcMotorSimple.Direction.FORWARD);
        motor2.setDirection(DcMotorSimple.Direction.FORWARD);

        motor1.setPower(gamepad1.right_stick_y);
        motor2.setPower(gamepad1.right_stick_y);
    }
}
