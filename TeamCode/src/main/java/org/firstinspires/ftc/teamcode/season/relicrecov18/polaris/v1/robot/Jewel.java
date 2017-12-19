package org.firstinspires.ftc.teamcode.season.relicrecov18.polaris.v1.robot;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.SubSystem;

/**
 * Created by Andrew on 12/7/2017.
 */

public class Jewel extends SubSystem {
    private Servo jewelCR;
    private Servo axe;
    // private ModernRoboticsI2cColorSensor color1;
    //public ColorSensor color1;
    public Jewel(Robot robot){super(robot);};
    @Override
    public void init() throws InterruptedException{
        jewelCR = robot.hardwareMap.servo.get("jewelcr");
        axe = robot.hardwareMap.servo.get("axe");
        //color1 = robot.hardwareMap.get(ModernRoboticsI2cColorSensor.class, "color1");
        /*color1 = robot.hardwareMap.colorSensor.get("color1");
        color1.setI2cAddress(I2cAddr.create7bit(0x39));
        color1.enableLed(true);*/
        moveArm(armPos.UP);
        moveAxe(axePos.BACK );
    }

    public enum armPos{
        DOWN(.7), UP(0);
        public double pos;
        armPos(double pos){
            this.pos = pos;
        }
    }
    public enum axePos{
        BACK(0), MIDDLE(0.6),FRONT(1);
        public double ap;
        axePos(double ap){ this.ap = ap;}
    }
    @Override
    public void handle() {

    }
    public void moveArm(armPos position){
        jewelCR.setPosition(position.pos);
    }
    public void moveAxe(axePos position){ axe.setPosition(position.ap);}

    /*public boolean isRed(){
        return color1.red()>color1.blue();
    }
*/


    @Override
    public void stop() {

    }
}
