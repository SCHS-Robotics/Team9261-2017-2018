package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;

/**
 * Created by Sage Creek Level Up on 11/19/2017.
 */

@TeleOp(name = "Pixy", group = "TeleOp")
public class PixyCam extends OpMode {
    I2cDeviceSynch pixy;
    @Override
    public void init() {
        pixy = hardwareMap.i2cDeviceSynch.get("pixy");
        pixy.setI2cAddress(I2cAddr.create7bit(0x54));
        I2cDeviceSynch.ReadWindow readWindow = new I2cDeviceSynch.ReadWindow(1, 26, I2cDeviceSynch.ReadMode.ONLY_ONCE);
        pixy.setReadWindow(readWindow);
        pixy.setLogging(true);
    }

    @Override
    public void init_loop() {

    }

    @Override
    public void start() {
        pixy.engage();
        telemetry.addData("Byte 0", pixy.read8(0));
        telemetry.addData("Byte 1", pixy.read8(1));
        telemetry.addData("Byte 2", pixy.read8(2));
        telemetry.addData("Byte 3", pixy.read8(3));
        telemetry.addData("Byte 4", pixy.read8(4));
        telemetry.addData("Byte 5", pixy.read8(5));
        telemetry.addData("Byte 6", pixy.read8(6));
        telemetry.addData("Byte 7", pixy.read8(7));
        telemetry.addData("Byte 8", pixy.read8(8));
        telemetry.addData("Byte 9", pixy.read8(9));
        telemetry.addData("Byte 10", pixy.read8(10));
        telemetry.addData("Byte 11", pixy.read8(11));
        telemetry.addData("Byte 12", pixy.read8(12));
        telemetry.addData("Byte 13", pixy.read8(13));
    }

    @Override
    public void loop(){

        telemetry.update();
    }
    @Override
    public void stop() {

    }
}
