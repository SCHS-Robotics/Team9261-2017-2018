package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;

/**
 * Created by Sage Creek Level Up on 10/22/2017.
 */
//This was a failure
@Disabled
@TeleOp(name = "Test", group = "idk")
public class TestTeleop extends LinearOpMode {
    BNO055IMU gyro; //integrated gyro
    DcMotor motor1; //front right motor
    DcMotor motor2; //front left motor
    DcMotor motor3; //back right motor
    DcMotor motor4; //back left motor
    @Override
    public void runOpMode() throws InterruptedException{
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        CalibrateGyro(gyro);
        parameters.calibrationDataFile = "WhatIsThisFile?WhoKnows!.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "gyro";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        gyro = hardwareMap.get(BNO055IMU.class,"gyro");
        gyro.initialize(parameters);
        waitForStart();
        double magnitudex = gamepad1.right_stick_x;
        double magnitudey = gamepad1.right_stick_y;
        double angle = Math.atan(magnitudey/magnitudex);
        Orientation orientation = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
        while((gamepad1.right_stick_y != 0 || gamepad1.right_stick_x != 0) && opModeIsActive()){
            magnitudex = gamepad1.right_stick_x;
            magnitudey = gamepad1.right_stick_y;
            angle = Math.atan(magnitudey/magnitudex);
            orientation = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
            if(angle-orientation.firstAngle < 0) {
                //magically strafe
            }
            //get vector
            //update vector as you move
            //
        }
    }
    public void CalibrateGyro(BNO055IMU g){
        BNO055IMU.CalibrationData calibrationData = g.readCalibrationData();
        String filename = "WhatIsThisFile?WhoKnows!.json";
        File file = AppUtil.getInstance().getSettingsFile(filename);
        ReadWriteFile.writeFile(file, calibrationData.serialize());
    }
}
