package org.firstinspires.ftc.teamcode.season.relicrecov18.polaris.v1.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.SubSystem;

/**
 * Created by andre_000 on 01/22/2018.
 */

public class FlippyBoii extends SubSystem {
    private DcMotor flippyboiAlpha, flippyboiBeta;
    private Servo dumpTruck1, dumpTruck2, dumpTruck3;
    public FlippyBoii(Robot robot) {
        super(robot);
    }
    @Override
    public void init() throws InterruptedException {
        flippyboiAlpha = robot.hardwareMap.dcMotor.get("flippyboialpha");
        flippyboiBeta = robot.hardwareMap.dcMotor.get("flippyboibeta");
        flippyboiAlpha.setDirection(DcMotor.Direction.FORWARD);
        flippyboiBeta.setDirection(DcMotor.Direction.REVERSE);

        dumpTruck1 = robot.hardwareMap.servo.get("dumptruck1");
        dumpTruck2 = robot.hardwareMap.servo.get("dumptruck2");
        dumpTruck3 = robot.hardwareMap.servo.get("dumptruck3");
        dumpTruck1.setPosition(0.6);
        dumpTruck2.setPosition(1);
        dumpTruck3.setPosition(0.4);
    }

    @Override
    public void handle() {
        flippyboiAlpha.setPower(robot.gamepad2.left_stick_y * Math.abs(robot.gamepad2.left_stick_y));
        flippyboiBeta.setPower(robot.gamepad2.left_stick_y * Math.abs(robot.gamepad2.left_stick_y));
        if(robot.gamepad2.a){   //original had a check of 1-trigger>.6
            dumpTruck1.setPosition(0.48);
        }else {
            Thread dump1 = new Thread() {
                @Override
                public void run() {
                    dumpTruck1.setPosition(Range.clip(1-robot.gamepad2.left_trigger, 0.2, 0.9));//original was 0-.6 and 1-trigger
                }
            };
            Thread dump2 = new Thread() {
                @Override
                public void run() {
                    dumpTruck3.setPosition(Range.clip(robot.gamepad2.left_trigger, 0.1, 0.8));
                }
            };
            dump1.start();
            dump2.start();
        }
        dumpTruck2.setPosition(Range.clip(1 - robot.gamepad2.right_trigger, 0, 1));
    }
    public void dispense(){
        dumpTruck1.setPosition(1);
    }
    public void stopDispense(){
        dumpTruck1.setPosition(0.2);
    }
    public void alignblock(){
        dumpTruck2.setPosition(0);
    }
    public void doneAligning(){
        dumpTruck2.setPosition(1);
    }

    @Override
    public void stop() {

    }
}
