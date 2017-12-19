package org.firstinspires.ftc.teamcode;

import android.graphics.Rect;
import android.hardware.Camera;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;

import java.util.HashMap;
import org.firstinspires.ftc.robotcontroller.external.samples.SensorMRRangeSensor;
import org.firstinspires.ftc.robotcontroller.external.samples.SensorREVColorDistance;
import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.firstinspires.ftc.robotcore.external.Telemetry;
/*import org.opencv.android.CameraBridgeViewBase;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;*/

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.math.*;
/*
  Created by rileyhull on 10/1/17.
 */
//1680 ticks per revolution for 60:1 ratio, 1120 for 40:1, and 560 for 20:1
public abstract class BaseOpModeR extends LinearOpMode
{
    private DcMotor motor2;
    private DcMotor motor1;
    private DcMotor motor4;
    private DcMotor motor3;
    private DcMotor motorC1 = hardwareMap.dcMotor.get("motorC1");
    private DcMotor motorC2 = hardwareMap.dcMotor.get("motorC2");
    private DcMotor motorFW1 = hardwareMap.dcMotor.get("motorFW1");
    private DcMotor motorFW2 = hardwareMap.dcMotor.get("motorFW2");
    //ColorSensor colorSense;
    //OpticalDistanceSensor rangeSense;
    Servo arm;
    //Servo servo1;
    //Servo servo2;
    //Servo servo3;
    //int currentPos2 = motor2.getCurrentPosition();
    //int currentPos3 = motor3.getCurrentPosition();
    //int currentPos1 = motor1.getCurrentPosition();
    //int currentPos4 = motor4.getCurrentPosition();

    public void Foreward(double power) throws InterruptedException
    {
        motor1.setPower(power);
        motor2.setPower(power);
        motor3.setPower(power);
        motor4.setPower(power);
    }

    public void Backward(double power) throws InterruptedException
    {
        motor1.setPower(-power);
        motor2.setPower(-power);
        motor3.setPower(-power);
        motor4.setPower(-power);
    }

    public void StrafeRight(double power) throws InterruptedException
    {
        motor1.setPower(power);
        motor2.setPower(-power);
        motor3.setPower(-power);
        motor4.setPower(power);
    }

    public void StrafeLeft(double power) throws InterruptedException
    {
        motor1.setPower(-power);
        motor2.setPower(power);
        motor3.setPower(power);
        motor4.setPower(-power);
    }

    public void UpRight(double power) throws InterruptedException
    {
        motor1.setPower(power);
        motor2.setPower(0);
        motor3.setPower(0);
        motor4.setPower(power);
    }

    public void UpLeft(double power) throws InterruptedException
    {
        motor1.setPower(0);
        motor2.setPower(power);
        motor3.setPower(power);
        motor4.setPower(0);
    }

    public void DownLeft(double power) throws InterruptedException
    {
        motor1.setPower(-power);
        motor2.setPower(0);
        motor3.setPower(0);
        motor4.setPower(-power);
    }

    public void DownRight(double power) throws InterruptedException
    {
        motor1.setPower(0);
        motor2.setPower(-power);
        motor3.setPower(-power);
        motor4.setPower(0);
    }

    public void SCW(double power) throws InterruptedException
    {
        motor1.setPower(power);
        motor2.setPower(-power);
        motor3.setPower(power);
        motor4.setPower(-power);
    }

    public void SCCW(double power) throws InterruptedException
    {
        motor1.setPower(-power);
        motor2.setPower(power);
        motor3.setPower(-power);
        motor4.setPower(power);
    }

    public void Stop()
    {
        motor2.setPower(0);
        motor1.setPower(0);
        motor4.setPower(0);
        motor3.setPower(0);
    }

    public void ForewardETicks(double power, int encoderticks) throws InterruptedException
    {
        int currentPos2 = motor2.getCurrentPosition();
        int currentPos3 = motor3.getCurrentPosition();
        int currentPos1 = motor1.getCurrentPosition();
        int currentPos4 = motor4.getCurrentPosition();
        /*motor1.setMode(DcMotor.RunMode.RESET_ENCODERS);
        motor2.setMode(DcMotor.RunMode.RESET_ENCODERS);
        motor3.setMode(DcMotor.RunMode.RESET_ENCODERS);
        motor4.setMode(DcMotor.RunMode.RESET_ENCODERS);*/
        motor1.setTargetPosition(encoderticks + currentPos1);
        motor2.setTargetPosition(encoderticks + currentPos2);
        motor3.setTargetPosition(encoderticks + currentPos3);
        motor4.setTargetPosition(encoderticks + currentPos4);
        motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor4.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Foreward(power);
        while (motor1.isBusy() && motor2.isBusy() && motor3.isBusy() && motor4.isBusy() && opModeIsActive())
        {
            Thread.sleep(1);
        }
        Stop();
        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor4.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //int currentPos2 = motor2.getCurrentPosition();
        //int currentPos1 = motor1.getCurrentPosition();
        //int currentPos4 = motor4.getCurrentPosition();
        //int currentPos3 = motor3.getCurrentPosition();
    }

    public void BackwardETicks(double power, int encoderticks) throws InterruptedException
    {
        int currentPos2 = motor2.getCurrentPosition();
        int currentPos3 = motor3.getCurrentPosition();
        int currentPos1 = motor1.getCurrentPosition();
        int currentPos4 = motor4.getCurrentPosition();
        /*motor1.setMode(DcMotor.RunMode.RESET_ENCODERS);
        motor2.setMode(DcMotor.RunMode.RESET_ENCODERS);
        motor3.setMode(DcMotor.RunMode.RESET_ENCODERS);
        motor4.setMode(DcMotor.RunMode.RESET_ENCODERS);*/
        motor1.setTargetPosition(encoderticks + currentPos1);
        motor2.setTargetPosition(encoderticks + currentPos2);
        motor3.setTargetPosition(encoderticks + currentPos3);
        motor4.setTargetPosition(encoderticks + currentPos4);
        motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor4.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Backward(power);
        while (motor1.isBusy() && motor2.isBusy() && motor3.isBusy() && motor4.isBusy() && opModeIsActive())
        {
            Thread.sleep(1);
        }
        Stop();
        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor4.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //int currentPos2 = motor2.getCurrentPosition();
        //int currentPos1 = motor1.getCurrentPosition();
        //int currentPos4 = motor4.getCurrentPosition();
        //int currentPos3 = motor3.getCurrentPosition();
    }

    public void RightETicks(double power, int encoderticks) throws InterruptedException
    {
        int currentPos2 = motor2.getCurrentPosition();
        int currentPos3 = motor3.getCurrentPosition();
        int currentPos1 = motor1.getCurrentPosition();
        int currentPos4 = motor4.getCurrentPosition();
        /*motor1.setMode(DcMotor.RunMode.RESET_ENCODERS);
        motor2.setMode(DcMotor.RunMode.RESET_ENCODERS);
        motor3.setMode(DcMotor.RunMode.RESET_ENCODERS);
        motor4.setMode(DcMotor.RunMode.RESET_ENCODERS);*/
        motor1.setTargetPosition(encoderticks + currentPos1);
        motor2.setTargetPosition(encoderticks + currentPos2);
        motor3.setTargetPosition(encoderticks + currentPos3);
        motor4.setTargetPosition(encoderticks + currentPos4);
        motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor4.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        StrafeRight(power);
        while (motor1.isBusy() && motor2.isBusy() && motor3.isBusy() && motor4.isBusy() && opModeIsActive())
        {
            Thread.sleep(1);
        }
        Stop();
        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor4.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //int currentPos2 = motor2.getCurrentPosition();
        //int currentPos1 = motor1.getCurrentPosition();
        //int currentPos4 = motor4.getCurrentPosition();
        //int currentPos3 = motor3.getCurrentPosition();
    }

    public void LeftETicks(double power, int encoderticks) throws InterruptedException
    {
        int currentPos2 = motor2.getCurrentPosition();
        int currentPos3 = motor3.getCurrentPosition();
        int currentPos1 = motor1.getCurrentPosition();
        int currentPos4 = motor4.getCurrentPosition();
        /*motor1.setMode(DcMotor.RunMode.RESET_ENCODERS);
        motor2.setMode(DcMotor.RunMode.RESET_ENCODERS);
        motor3.setMode(DcMotor.RunMode.RESET_ENCODERS);
        motor4.setMode(DcMotor.RunMode.RESET_ENCODERS);*/
        motor1.setTargetPosition(encoderticks + currentPos1);
        motor2.setTargetPosition(encoderticks + currentPos2);
        motor3.setTargetPosition(encoderticks + currentPos3);
        motor4.setTargetPosition(encoderticks + currentPos4);
        motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor4.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        StrafeLeft(power);
        while (motor1.isBusy() && motor2.isBusy() && motor3.isBusy() && motor4.isBusy() && opModeIsActive())
        {
            Thread.sleep(1);
        }
        Stop();
        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor4.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //int currentPos2 = motor2.getCurrentPosition();
        //int currentPos1 = motor1.getCurrentPosition();
        //int currentPos4 = motor4.getCurrentPosition();
        //int currentPos3 = motor3.getCurrentPosition();
    }

    public void UpRightETicks(double power, int encoderticks) throws InterruptedException
    {
        int currentPos2 = motor2.getCurrentPosition();
        int currentPos3 = motor3.getCurrentPosition();
        int currentPos1 = motor1.getCurrentPosition();
        int currentPos4 = motor4.getCurrentPosition();
        /*motor1.setMode(DcMotor.RunMode.RESET_ENCODERS);
        motor2.setMode(DcMotor.RunMode.RESET_ENCODERS);
        motor3.setMode(DcMotor.RunMode.RESET_ENCODERS);
        motor4.setMode(DcMotor.RunMode.RESET_ENCODERS);*/
        motor1.setTargetPosition(encoderticks + currentPos1);
        motor2.setTargetPosition(encoderticks + currentPos2);
        motor3.setTargetPosition(encoderticks + currentPos3);
        motor4.setTargetPosition(encoderticks + currentPos4);
        motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor4.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        UpRight(power);
        while (motor1.isBusy() /*&& motor2.isBusy() && motor3.isBusy()*/ && motor4.isBusy() && opModeIsActive())
        {
            Thread.sleep(1);
        }
        Stop();
        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor4.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //int currentPos2 = motor2.getCurrentPosition();
        //int currentPos1 = motor1.getCurrentPosition();
        //int currentPos4 = motor4.getCurrentPosition();
        //int currentPos3 = motor3.getCurrentPosition();
    }

    public void UpLeftETicks(double power, int encoderticks) throws InterruptedException
    {
        int currentPos2 = motor2.getCurrentPosition();
        int currentPos3 = motor3.getCurrentPosition();
        int currentPos1 = motor1.getCurrentPosition();
        int currentPos4 = motor4.getCurrentPosition();
        /*motor1.setMode(DcMotor.RunMode.RESET_ENCODERS);
        motor2.setMode(DcMotor.RunMode.RESET_ENCODERS);
        motor3.setMode(DcMotor.RunMode.RESET_ENCODERS);
        motor4.setMode(DcMotor.RunMode.RESET_ENCODERS);*/
        motor1.setTargetPosition(encoderticks + currentPos1);
        motor2.setTargetPosition(encoderticks + currentPos2);
        motor3.setTargetPosition(encoderticks + currentPos3);
        motor4.setTargetPosition(encoderticks + currentPos4);
        motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor4.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        UpLeft(power);
        while (/*motor1.isBusy() &&*/ motor2.isBusy() && motor3.isBusy() && /*motor4.isBusy() &&*/ opModeIsActive())
        {
            Thread.sleep(1);
        }
        Stop();
        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor4.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //int currentPos2 = motor2.getCurrentPosition();
        //int currentPos1 = motor1.getCurrentPosition();
        //int currentPos4 = motor4.getCurrentPosition();
        //int currentPos3 = motor3.getCurrentPosition();
    }

    public void DownRightETicks(double power, int encoderticks) throws InterruptedException
    {
        int currentPos2 = motor2.getCurrentPosition();
        int currentPos3 = motor3.getCurrentPosition();
        int currentPos1 = motor1.getCurrentPosition();
        int currentPos4 = motor4.getCurrentPosition();
        /*motor1.setMode(DcMotor.RunMode.RESET_ENCODERS);
        motor2.setMode(DcMotor.RunMode.RESET_ENCODERS);
        motor3.setMode(DcMotor.RunMode.RESET_ENCODERS);
        motor4.setMode(DcMotor.RunMode.RESET_ENCODERS)*/
        motor1.setTargetPosition(encoderticks + currentPos1);
        motor2.setTargetPosition(encoderticks + currentPos2);
        motor3.setTargetPosition(encoderticks + currentPos3);
        motor4.setTargetPosition(encoderticks + currentPos4);
        motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor4.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        DownRight(power);
        while (/*motor1.isBusy() &&*/ motor2.isBusy() && motor3.isBusy() && /*motor4.isBusy() &&*/ opModeIsActive())
        {
            Thread.sleep(1);
        }
        Stop();
        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor4.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //int currentPos2 = motor2.getCurrentPosition();
        //int currentPos1 = motor1.getCurrentPosition();
        //int currentPos4 = motor4.getCurrentPosition();
        //int currentPos3 = motor3.getCurrentPosition();
    }

    public void DownLeftETicks(double power, int encoderticks) throws InterruptedException
    {
        int currentPos2 = motor2.getCurrentPosition();
        int currentPos3 = motor3.getCurrentPosition();
        int currentPos1 = motor1.getCurrentPosition();
        int currentPos4 = motor4.getCurrentPosition();
        /*motor1.setMode(DcMotor.RunMode.RESET_ENCODERS);
         motor2.setMode(DcMotor.RunMode.RESET_ENCODERS);
         motor3.setMode(DcMotor.RunMode.RESET_ENCODERS);
         motor4.setMode(DcMotor.RunMode.RESET_ENCODERS);*/
        motor1.setTargetPosition(encoderticks + currentPos1);
        motor2.setTargetPosition(encoderticks + currentPos2);
        motor3.setTargetPosition(encoderticks + currentPos3);
        motor4.setTargetPosition(encoderticks + currentPos4);
        motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor4.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        DownLeft(power);
        while (motor1.isBusy() /*&& motor2.isBusy() && motor3.isBusy()*/ && motor4.isBusy() && opModeIsActive())
        {
            Thread.sleep(1);
        }
        Stop();
        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor4.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //int currentPos2 = motor2.getCurrentPosition();
        //int currentPos1 = motor1.getCurrentPosition();
        //int currentPos4 = motor4.getCurrentPosition();
        //int currentPos3 = motor3.getCurrentPosition();
    }

    public void StrafeRightETicks(double power, int encoderticks) throws InterruptedException
    {
        int currentPos2 = motor2.getCurrentPosition();
        int currentPos3 = motor3.getCurrentPosition();
        int currentPos1 = motor1.getCurrentPosition();
        int currentPos4 = motor4.getCurrentPosition();
        /*motor1.setMode(DcMotor.RunMode.RESET_ENCODERS);
         motor2.setMode(DcMotor.RunMode.RESET_ENCODERS);
         motor3.setMode(DcMotor.RunMode.RESET_ENCODERS);
         motor4.setMode(DcMotor.RunMode.RESET_ENCODERS);*/
        motor1.setTargetPosition(encoderticks + currentPos1);
        motor2.setTargetPosition(encoderticks + currentPos2);
        motor3.setTargetPosition(encoderticks + currentPos3);
        motor4.setTargetPosition(encoderticks + currentPos4);
        motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor4.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        StrafeRight(power);
        while (motor1.isBusy() && motor2.isBusy() && motor3.isBusy() && motor4.isBusy() && opModeIsActive())
        {
            Thread.sleep(1);
        }
        Stop();
        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor4.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //int currentPos2 = motor2.getCurrentPosition();
        //int currentPos1 = motor1.getCurrentPosition();
        //int currentPos4 = motor4.getCurrentPosition();
        //int currentPos3 = motor3.getCurrentPosition();
    }

    public void StrafeLeftETicks(double power, int encoderticks) throws InterruptedException
    {
        int currentPos2 = motor2.getCurrentPosition();
        int currentPos3 = motor3.getCurrentPosition();
        int currentPos1 = motor1.getCurrentPosition();
        int currentPos4 = motor4.getCurrentPosition();
        /*motor1.setMode(DcMotor.RunMode.RESET_ENCODERS);
         motor2.setMode(DcMotor.RunMode.RESET_ENCODERS);
         motor3.setMode(DcMotor.RunMode.RESET_ENCODERS);
         motor4.setMode(DcMotor.RunMode.RESET_ENCODERS);*/
        motor1.setTargetPosition(encoderticks + currentPos1);
        motor2.setTargetPosition(encoderticks + currentPos2);
        motor3.setTargetPosition(encoderticks + currentPos3);
        motor4.setTargetPosition(encoderticks + currentPos4);
        motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor4.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        StrafeLeft(power);
        while (motor1.isBusy() && motor2.isBusy() && motor3.isBusy() && motor4.isBusy() && opModeIsActive())
        {
            Thread.sleep(1);
        }
        Stop();
        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor4.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //int currentPos2 = motor2.getCurrentPosition();
        //int currentPos1 = motor1.getCurrentPosition();
        //int currentPos4 = motor4.getCurrentPosition();
        //int currentPos3 = motor3.getCurrentPosition();
    }

    public void SCCWETicks(double power, int encoderticks) throws InterruptedException
    {
        int currentPos2 = motor2.getCurrentPosition();
        int currentPos3 = motor3.getCurrentPosition();
        int currentPos1 = motor1.getCurrentPosition();
        int currentPos4 = motor4.getCurrentPosition();
        /*motor1.setMode(DcMotor.RunMode.RESET_ENCODERS);
        motor2.setMode(DcMotor.RunMode.RESET_ENCODERS);
        motor3.setMode(DcMotor.RunMode.RESET_ENCODERS);
        motor4.setMode(DcMotor.RunMode.RESET_ENCODERS);*/
        motor1.setTargetPosition(encoderticks + currentPos1);
        motor2.setTargetPosition(encoderticks + currentPos2);
        motor3.setTargetPosition(encoderticks + currentPos3);
        motor4.setTargetPosition(encoderticks + currentPos4);
        motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor4.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        SCCW(power);
        while (motor1.isBusy() && motor2.isBusy() && motor3.isBusy() && motor4.isBusy() && opModeIsActive())
        {
            Thread.sleep(1);
        }
        Stop();
        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor4.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //int currentPos2 = motor2.getCurrentPosition();
        //int currentPos1 = motor1.getCurrentPosition();
        //int currentPos4 = motor4.getCurrentPosition();
        //int currentPos3 = motor3.getCurrentPosition();
    }

    public void SCWETicks(double power, int encoderticks) throws InterruptedException
    {
        int currentPos2 = motor2.getCurrentPosition();
        int currentPos3 = motor3.getCurrentPosition();
        int currentPos1 = motor1.getCurrentPosition();
        int currentPos4 = motor4.getCurrentPosition();
        /*motor1.setMode(DcMotor.RunMode.RESET_ENCODERS);
        motor2.setMode(DcMotor.RunMode.RESET_ENCODERS);
        motor3.setMode(DcMotor.RunMode.RESET_ENCODERS);
        motor4.setMode(DcMotor.RunMode.RESET_ENCODERS);*/
        motor1.setTargetPosition(encoderticks + currentPos1);
        motor2.setTargetPosition(encoderticks + currentPos2);
        motor3.setTargetPosition(encoderticks + currentPos3);
        motor4.setTargetPosition(encoderticks + currentPos4);
        motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor4.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        SCW(power);
        while (motor1.isBusy() && motor2.isBusy() && motor3.isBusy() && motor4.isBusy() && opModeIsActive())
        {
            Thread.sleep(1);
        }
        Stop();
        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor4.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //int currentPos2 = motor2.getCurrentPosition();
        //int currentPos1 = motor1.getCurrentPosition();
        //int currentPos4 = motor4.getCurrentPosition();
        //int currentPos3 = motor3.getCurrentPosition();
    }

    public void SCCWDegrees(double degrees) throws InterruptedException
    {
        double degreeticks = ((4.006 * degrees) / (4 * java.lang.Math.sqrt(2) * 360));
        int aticks = (int)Math.round(degreeticks);
        SCCWETicks(1, aticks);
    }

    public void SCWDegrees(double degrees) throws InterruptedException
    {
        double degreeticks = ((4.006 * degrees) / (4 * java.lang.Math.sqrt(2) * 360));
        int aticks = (int)Math.round(degreeticks);
        SCWETicks(1, aticks);
    }

    public void DtT(long distance) throws InterruptedException
    {
        double encoderticks = ((1120 * distance)/(8.012 * Math.PI));
    }

    public void ForewardDistance(double power, long distance) throws InterruptedException
    {
        double encoderticks = ((1120 * distance)/(8.012 * Math.PI));
        int dticks = (int)Math.round(encoderticks);
        ForewardETicks(power, dticks);
    }

    public void BackwardDistance(double power, long distance) throws InterruptedException
    {
        double encoderticks = ((1120 * distance)/(8.012 * Math.PI));
        int dticks = (int)Math.round(encoderticks);
        BackwardETicks(power, dticks);
    }

    public void LeftDistance(double power, long distance) throws InterruptedException
    {
        double encoderticks = ((1120 * distance)/(8.012 * Math.PI));
        int dticks = (int)Math.round(encoderticks);
        StrafeLeftETicks(power, dticks);
    }

    public void RightDistance(double power, long distance) throws InterruptedException
    {
        double encoderticks = ((1120 * distance)/(8.012 * Math.PI));
        int dticks = (int)Math.round(encoderticks);
        StrafeRightETicks(power, dticks);
    }

    public void UpRightDistance(double power, long distance) throws InterruptedException
    {
        double encoderticks = ((1120 * distance) / (8.012 * Math.PI));
        int dticks = (int) Math.round(encoderticks);
        StrafeRightETicks(power, dticks);
    }

    public void UpLeftDistance(double power, long distance) throws InterruptedException
    {
        double encoderticks = ((1120 * distance) / (8.012 * Math.PI));
        int dticks = (int) Math.round(encoderticks);
        UpLeftETicks(power, dticks);
    }

    public void DownLeftDistance(double power, long distance) throws InterruptedException
    {
        double encoderticks = ((1120 * distance) / (8.012 * Math.PI));
        int dticks = (int) Math.round(encoderticks);
        DownLeftETicks(power, dticks);
    }

    public void DownRightDistance(double power, long distance) throws InterruptedException
    {
        double encoderticks = ((1120 * distance) / (8.012 * Math.PI));
        int dticks = (int) Math.round(encoderticks);
        DownRightETicks(power, dticks);
    }

    public void JewelKnock(long armposition) throws InterruptedException //code written by Bonnie Brasher
    {
        ColorSensor colorsense;
        colorsense = hardwareMap.colorSensor.get("colorsense");

        arm.setPosition(armposition);
        if (colorsense.red() >= 5)
        {
            StrafeRightETicks(1, 200);
        }
        else
        {
            StrafeLeftETicks(1, 200);
        }
        arm.setPosition(0);
    }

    public void GlyphPickUp(long power) throws InterruptedException
    {
        //recognize it OpenCV #marcorules
        //align robit OpenCV #marcorules
        //start conveyor belt
        motorC1.setPower(power);
        motorC2.setPower(power);
        //start wheels
        motorFW1.setPower(power);
        motorFW2.setPower(power);
        //wait
        Thread.sleep(2000);
        //stop wheels
        motorFW1.setPower(0);
        motorFW2.setPower(0);
        //stop conveyor belt
        motorC1.setPower(0);
        motorC2.setPower(0);
    }

    public void initilaize()
    {
        //Assigning previously declared variables to expansion hub names
        //colorSense = hardwareMap.colorSensor.get("colorMR");
        //servo1 = hardwareMap.servo.get("servo1");
        // servo2 = hardwareMap.servo.get("servo2");
        // servo3 = hardwareMap.servo.get("servo3");
        //rangeSense = hardwareMap.opticalDistanceSensor.get("rangeREV");

        //Creating motors
        motor2 = hardwareMap.dcMotor.get("motor2");
        motor1 = hardwareMap.dcMotor.get("motor1");
        motor4 = hardwareMap.dcMotor.get("motor4");
        motor3 = hardwareMap.dcMotor.get("motor3");
        motor1.setDirection(DcMotor.Direction.REVERSE);
        motor4.setDirection(DcMotor.Direction.REVERSE);

        //Setting up encoders
        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor4.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
