package org.firstinspires.ftc.teamcode.season.relicrecov18.polaris.v1.robot;

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
        moveArm(armPos.MIDDLE);
        moveAxe(axePos.UP);
    }

    public enum armPos{
        LEFT(0.5), MIDDLE(0.2), RIGHT(0), RETURN(0.2);
        public double pos;
        armPos(double pos){
            this.pos = pos;
        }
    }
    public enum axePos{
        DOWN(0.05), UP(0.7), UPPERMID(0.3), LOWERMID(0.25);
        public double ap;
        axePos(double ap){ this.ap = ap;}
    }
    @Override
    public void handle() {
        if(robot.gamepad1.x){
            moveArm(armPos.MIDDLE);
            moveAxe(axePos.UP);
        }
    }
    public void moveArm(armPos position){jewelCR.setPosition(position.pos);}
    public void moveAxe(axePos position){ axe.setPosition(position.ap);}

    /*public boolean isRed(){
        return color1.red()>color1.blue();
    }
*/

    public void slowMoveAxe(axePos finalPos) {
        while(axe.getPosition() < finalPos.ap) {
            axe.setPosition(axe.getPosition()+0.01);
        }
    }

    @Override
    public void stop() {

    }
}
