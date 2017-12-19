package org.firstinspires.ftc.teamcode;

import android.media.MediaPlayer;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.text.SimpleDateFormat;
import java.util.Date;

import static android.os.SystemClock.sleep;

/**
 * Created by Sage Creek Level Up on 10/22/2017.
 */
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Mecanum Drive", group = "OpMode")
public class MecanumDrive extends OpMode{
    private DcMotor motor1; //motors 1 & 3 are left motors
    private DcMotor motor2; //motors 2 & 4 are right motors

    private DcMotor motor3;
    private DcMotor motor4;

    private DcMotor motor5; //glyph motors
    private DcMotor motor6;

    private Servo leftcr;
    private Servo rightcr;

    //private Servo glyphservo;

    @Override
    public void init(){
        motor1 = hardwareMap.dcMotor.get("motor1");
        motor1.setDirection(DcMotor.Direction.REVERSE);
        motor2 = hardwareMap.dcMotor.get("motor2");
        motor2.setDirection(DcMotor.Direction.FORWARD);

        motor3 = hardwareMap.dcMotor.get("motor3");
        motor3.setDirection(DcMotor.Direction.REVERSE);
        motor4 = hardwareMap.dcMotor.get("motor4");
        motor4.setDirection(DcMotor.Direction.FORWARD);

        motor5 = hardwareMap.dcMotor.get("motor5");
        motor5.setDirection(DcMotor.Direction.REVERSE);
        motor6 = hardwareMap.dcMotor.get("motor6");
        motor6.setDirection(DcMotor.Direction.FORWARD);

        leftcr = hardwareMap.servo.get("leftcr");
        rightcr = hardwareMap.servo.get("rightcr");

        leftcr.setDirection(Servo.Direction.REVERSE);
        rightcr.setDirection(Servo.Direction.FORWARD);
    }

    @Override
    public void init_loop(){

    }

    @Override
    public void start(){

    }

    @Override
    public void loop(){
        double x = gamepad1.left_stick_x;
        double y = gamepad1.left_stick_y;
        double c = gamepad1.left_trigger-gamepad1.right_trigger;
        motor1.setPower(Range.clip(y-x+c, -1, 1));
        motor2.setPower(Range.clip(y+x-c, -1, 1));
        motor3.setPower(Range.clip(y+x+c, -1, 1));
        motor4.setPower(Range.clip(y-x-c, -1, 1));

        if(gamepad1.right_bumper){
            if(gamepad1.x){
                motor5.setPower(1);
                motor6.setPower(1);
            }else {
                motor5.setPower(0.7);
                motor6.setPower(0.65);
            }
            leftcr.setPosition(1);
            rightcr.setPosition(1);

        }else if(gamepad1.left_bumper) {
            motor5.setPower(-1);
            motor6.setPower(-1);
            leftcr.setPosition(-1);
            rightcr.setPosition(-1);
        }else{
            motor5.setPower(0);
            motor6.setPower(0);
            leftcr.setPosition(0);
            rightcr.setPosition(0);
        }
    }

    @Override
    public void stop(){

    }
}
