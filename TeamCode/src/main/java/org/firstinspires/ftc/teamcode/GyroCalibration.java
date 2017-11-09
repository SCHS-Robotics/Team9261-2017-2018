package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;

/**
 * Created by Sage Creek Level Up on 10/22/2017.
 */@TeleOp(name = "GyroCalibration", group = "idk")
public class GyroCalibration extends LinearOpMode {
    BNO055IMU gyro;
    @Override
    public void runOpMode() throws InterruptedException {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.loggingEnabled = true;
        parameters.loggingTag = "Gyro";
        gyro = hardwareMap.get(BNO055IMU.class,"gyro");
        gyro.initialize(parameters); //all code before this is setting up the gyro
        if(gamepad1.a) {
            BNO055IMU.CalibrationData calibrationData = gyro.readCalibrationData();
            String filename = "WhatIsThisFile?WhoKnows!.json";
            File file = AppUtil.getInstance().getSettingsFile(filename);
            ReadWriteFile.writeFile(file, calibrationData.serialize());
        }
    }
}