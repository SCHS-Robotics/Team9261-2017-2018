package org.firstinspires.ftc.teamcode.season.relicrecov18.polaris.v1.robot;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;
import com.sun.tools.javac.tree.DCTree;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.season.relicrecov18.polaris.v1.program.programs.AnalogSensor;
import org.firstinspires.ftc.teamcode.util.Vector;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.SubSystem;

import java.lang.reflect.Array;
import java.util.ArrayList;

import static java.lang.Thread.sleep;

/**
 * Created by Andrew on 12/6/2017.
 */

public class Drive extends SubSystem{
    private DcMotorEx motor1, motor2, motor3, motor4;
    public AnalogSensor frontSensor;
    public BNO055IMU imu;
    public int homeAngle;
    double globalAngle;
    double stoneThreshold = 2.5;
    double drivingoffThreshold = 2.5;
    private double motor1XY;
    private double motor2XY;
    private double motor3XY;
    private double motor4XY;
    boolean dpadLeftPrevState = false;
    boolean dpadRightPrevState = false;
    double targetAngle;


    public Drive(Robot robot) {
        super(robot);
    }

    @Override
    public void init() throws InterruptedException {
        motor1 = (DcMotorEx) robot.hardwareMap.dcMotor.get("motor1");
        motor2 = (DcMotorEx) robot.hardwareMap.dcMotor.get("motor2");
        motor3 = (DcMotorEx) robot.hardwareMap.dcMotor.get("motor3");
        motor4 = (DcMotorEx) robot.hardwareMap.dcMotor.get("motor4");

        motor1.setDirection(DcMotor.Direction.REVERSE);
        motor2.setDirection(DcMotor.Direction.FORWARD);
        motor3.setDirection(DcMotor.Direction.REVERSE);
        motor4.setDirection(DcMotor.Direction.FORWARD);

        frontSensor =  new AnalogSensor("frontSensor", AnalogSensor.Type.LONG_RANGE);
        frontSensor.initialize(robot.hardwareMap);
        initGyro();
        modeEncoders();
    }
    public void readSensorsSetUp() {
        for (int i = 0; i < AnalogSensor.NUM_OF_READINGS; i++ ) {
            frontSensor.addReading();
        }
    }
    public void initGyro(){
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        imu = robot.hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);

        robot.telemetry.addData("Mode", "calibrating...");
        robot.telemetry.update();
    }
    public void notifyGyroReady(){
        robot.telemetry.addData("Mode", "waiting for start");
        robot.telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        robot.telemetry.addData("Angle", getYaw());
        robot.telemetry.addData("Pitch", getPitch());
        robot.telemetry.update();
    }

    @Override
    public void handle() {
        Vector inputVector = new Vector(robot.gamepad1.left_stick_x, -robot.gamepad1.left_stick_y); //gamepads are weird, -1 is at the top of the y axis
        Vector motionVector = inputVector.rotate(-Math.PI / 4);
        robot.telemetry.addData("motionVector init r", motionVector.r);
        motionVector = motionVector.normalize(Math.pow(motionVector.r,2)*Math.sqrt(2));
        robot.telemetry.addData("motionVector final r", motionVector.r);
        robot.telemetry.addData("Aligned with Cryptobox:", isAligned());

        if ((robot.gamepad1.dpad_left != dpadLeftPrevState && robot.gamepad1.dpad_left) || (robot.gamepad1.dpad_right != dpadRightPrevState && robot.gamepad1.dpad_right)){
            targetAngle = getYaw();
        }
        if (robot.gamepad1.left_bumper && motionVector.isZeroVector()) {
            motor1.setPower(-1);
            motor2.setPower(1);
            motor3.setPower(-1);
            motor4.setPower(1);

        } else if (robot.gamepad1.right_bumper && motionVector.isZeroVector()) {
            motor1.setPower(1);
            motor2.setPower(-1);
            motor3.setPower(1);
            motor4.setPower(-1);
        }

        else if(robot.gamepad1.dpad_left || robot.gamepad1.dpad_right) {
            double error = getYaw()-targetAngle;
            double p = 0.05*error;
            double power = 0.3*(dpadRight()-dpadLeft());
            motor1.setPower(power+p);
            motor2.setPower(-power-p);
            motor3.setPower(-power+p);
            motor4.setPower(power-p);
        }

        else {
            double offset = (robot.gamepad1.left_trigger-robot.gamepad1.right_trigger) / 3.5;
            motor1.setPower(Range.clip(motionVector.x - offset, -1, 1));
            motor2.setPower(Range.clip(motionVector.y + offset, -1, 1));
            motor3.setPower(Range.clip(motionVector.y - offset, -1, 1));
            motor4.setPower(Range.clip(motionVector.x + offset, -1, 1));
        }
        dpadLeftPrevState = robot.gamepad1.dpad_left;
        dpadRightPrevState = robot.gamepad1.dpad_right;
        robot.telemetry.update();

    }
    public int dpadRight(){
        int returnVal = robot.gamepad1.dpad_right ? 1 : 0;
        return returnVal;
    }
    public int dpadLeft(){
        int returnVal = robot.gamepad1.dpad_left ? 1 : 0;
        return returnVal;
    }

    public void modeEncoders() {
        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor4.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void drive(double speed) {
        motor1.setPower(speed);
        motor2.setPower(speed);
        motor3.setPower(speed);
        motor4.setPower(speed);
    }

    public void strafeDistance(double distance, double power) throws InterruptedException {
        double encoderTicks = inchesToPositions(distance);
        resetEncoders();
        waitforEncodersStrafe(encoderTicks, power);
        stopDrive();
    }
    boolean isAligned() {
        return (int) Math.abs(getYaw()) % 90 < 3;
    }

    public void driveDistance(double distance, double power) throws InterruptedException {
        double encoderTicks = inchesToPositions(distance);
        resetEncoders();
        waitforEncoders(encoderTicks, power);
        stopDrive();
    }

    public void turnExact(double velocity, double angle, double threshold) throws InterruptedException {//velocity out of 1
        //double initialYaw = getYaw();
        //double totalDelta = Math.abs(initialYaw)-angle;
        while (Math.abs(getYaw() - angle) > threshold) {
            //double inversePercentError = 1-Math.abs((getYaw()-initialYaw)/totalDelta);
            //double pController = Range.clip(Math.cbrt(inversePercentError), 0.2, 1);
            //double outputRequest = velocity*pController;
            if (getYaw() < angle) {

                motor1.setPower(-velocity);
                motor2.setPower(velocity);
                motor3.setPower(-velocity);
                motor4.setPower(velocity);

            } else {

                motor1.setPower(velocity);
                motor2.setPower(-velocity);
                motor3.setPower(velocity);
                motor4.setPower(-velocity);

            }

            robot.telemetry.addData("Heading", getYaw());
            robot.telemetry.update();
            sleep(1);

        }

        stopDrive();

    }
    public void strafeMaintainYawUntilDistance(double distance, double velocity, double angle) throws InterruptedException{
        resetEncoders();
        waitforEncoderDistance(distance, velocity, angle);
        stopDrive();
    }
    public void waitforEncoderDistance (double distance, double velocity, double targAngle) throws InterruptedException{
        double elapsedTime = 0;
        double accumulatedError = 0;
        double lastError = 0;
        double kp = 0.075;
        double ki = 0;
        double kd = 0;
        double toGo = 0;
        while (toGo < distance) {
            double currentTime = System.currentTimeMillis();
            double error = getYaw() - targAngle;
            double p = kp * error;
            double i = ki * (error + accumulatedError) * elapsedTime;
            double d = kd * (error - lastError) / elapsedTime;
            double pi = p + i;
            double pd = p + d;
            double pid = p + i + d;
            motor1.setPower(velocity + pid);
            motor2.setPower(-velocity - pid);
            motor3.setPower(-velocity + pid);
            motor4.setPower(velocity - pid);
            lastError = error;
            accumulatedError += error;
            robot.telemetry.addData("MotorFL", motor1.getPower());
            robot.telemetry.addData("MotorFR", motor2.getPower());
            robot.telemetry.addData("MotorBL", motor3.getPower());
            robot.telemetry.addData("MotorBR", motor4.getPower());
            robot.telemetry.addData("CMAvg", frontSensor.getCmAvg());
            robot.telemetry.update();
            sleep(1);
            elapsedTime = System.currentTimeMillis() - currentTime;
            toGo = frontSensor.getCmAvg();
            frontSensor.addReading();
        }
    }

    public void driveMaintainYaw(double distance, double velocity, double angle, double threshold) throws InterruptedException {
        double encoderTicks = inchesToPositions(distance);
        resetEncoders();
        waitforEncoderwithYaw(encoderTicks, velocity, angle, threshold);
        stopDrive();
    }

    public void waitforEncoderwithYaw(double encoderTicks, double velocity, double targAngle, double threshold) throws InterruptedException {
        double elapsedTime = 1;
        double accumulatedError = 0;
        double lastError = 0;
        double kp = 0.05;
        double ki = 0;
        double kd = 0.05;
        while (getFurthestEncoder() < encoderTicks) {
            double currentTime = System.currentTimeMillis();
            double error = getYaw() - targAngle;
            double p = kp * error; //proportional component, bases the change to output request based on the amount of error present
            double i = ki * (error + accumulatedError) * elapsedTime;//integral component, bases the change to output request based on the accumulation of error present
            double d = kd * (error - lastError) / elapsedTime;//derivative component, bases the change to output request based on the rate of change of error present
            double pi = p + i;
            double pd = p + d;
            double pid = p + i + d;
            motor1.setPower(velocity + pid);
            motor2.setPower(velocity - pid);
            motor3.setPower(velocity + pid);
            motor4.setPower(velocity - pid);
            lastError = error;
            accumulatedError += error;
            robot.telemetry.addData("MotorFL", motor1.getPower());
            robot.telemetry.addData("MotorFR", motor2.getPower());
            robot.telemetry.addData("MotorBL", motor3.getPower());
            robot.telemetry.addData("MotorBR", motor4.getPower());
            robot.telemetry.addData("Yaw", getYaw());
            robot.telemetry.update();
            sleep(1);
            elapsedTime = System.currentTimeMillis() - currentTime;
            /*if(Math.abs(getYaw())-targAngle<threshold){
                motor1.setVelocity(velocity * 960, AngleUnit.DEGREES);
                motor2.setVelocity(velocity*900, AngleUnit.DEGREES);
                motor3.setVelocity(velocity*960, AngleUnit.DEGREES);
                motor4.setVelocity(velocity*900, AngleUnit.DEGREES);
                sleep(1);
            }
            else{
                motor1.setVelocity(velocity*900, AngleUnit.DEGREES);
                motor2.setVelocity(velocity*960, AngleUnit.DEGREES);
                motor3.setVelocity(velocity*900, AngleUnit.DEGREES);
                motor4.setVelocity(velocity*960, AngleUnit.DEGREES);
                sleep(1);
            }*/
        }
    }

    public void strafeMaintainYaw(double distance, double velocity, double angle, double threshold) throws InterruptedException{
        double encoderTicks = inchesToPositions(distance);
        resetEncoders();
        waitforEncoderStrafeWithYaw(encoderTicks, velocity, angle, threshold);
        stopDrive();
    }
    public void waitforEncoderStrafeWithYaw(double encoderTicks, double velocity, double targAngle, double threshold) throws InterruptedException {
        double elapsedTime = 0;
        double accumulatedError = 0;
        double lastError = 0;
        double kp = 0.075;
        double ki = 0;
        double kd = 0;
        while (getFurthestEncoder() < encoderTicks) {
            double currentTime = System.currentTimeMillis();
            double error = getYaw() - targAngle;
            double p = kp * error;
            double i = ki * (error + accumulatedError) * elapsedTime;
            double d = kd * (error - lastError) / elapsedTime;
            double pi = p + i;
            double pd = p + d;
            double pid = p + i + d;
            motor1.setPower(velocity + pid);
            motor2.setPower(-velocity - pid);
            motor3.setPower(-velocity + pid);
            motor4.setPower(velocity - pid);
            lastError = error;
            accumulatedError += error;
            robot.telemetry.addData("MotorFL", motor1.getPower());
            robot.telemetry.addData("MotorFR", motor2.getPower());
            robot.telemetry.addData("MotorBL", motor3.getPower());
            robot.telemetry.addData("MotorBR", motor4.getPower());
            robot.telemetry.update();
            sleep(1);
            elapsedTime = System.currentTimeMillis() - currentTime;
        }
    }

    public void stopDrive() {
        motor1.setPower(0);
        motor2.setPower(0);
        motor3.setPower(0);
        motor4.setPower(0);
    }

    public double getYaw() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;
    }

    public double getPitch() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.thirdAngle;
    }

    public void driveoffBalancingStone()throws InterruptedException{
        boolean stillDrivingOff = true;
        while(getPitch()<stoneThreshold){
            double error = getYaw();
            motor1.setPower(0.5+(0.05*error));
            motor2.setPower(0.5-(0.05*error));
            motor3.setPower(0.5+(0.05*error));
            motor4.setPower(0.5-(0.05*error));
            robot.telemetry.addData("Unbalancing stone", getPitch());
            robot.telemetry.update();
            sleep(10);
        }
        while(getPitch()>drivingoffThreshold && stillDrivingOff){
            double error = getYaw();
            motor1.setPower(0.5+(0.05*error));
            motor2.setPower(0.5-(0.05*error));
            motor3.setPower(0.5+(0.05*error));
            motor4.setPower(0.5-(0.05*error));
            robot.telemetry.addData("Still driving off", getPitch());
            robot.telemetry.update();
            sleep(10);
            if (getPitch()<stoneThreshold){
                stopDrive();
                stillDrivingOff = false;
                robot.telemetry.addData("Off the stone", getPitch());
                robot.telemetry.update();
            }
        }
    }

    private int getFurthestEncoder() {
        int left = Math.max(Math.abs(motor1.getCurrentPosition()), Math.abs(motor3.getCurrentPosition()));
        int right = Math.max(Math.abs(motor2.getCurrentPosition()), Math.abs(motor4.getCurrentPosition()));
        int both = Math.max(left, right);
        return both;
    }

    private void waitforEncoders(double encoderTicks, double power) throws InterruptedException {
        while (getFurthestEncoder() < encoderTicks) {
            motor1.setPower(power);
            motor2.setPower(power);
            motor3.setPower(power);
            motor4.setPower(power);
            sleep(1);
        }
    }

    private void waitforEncodersStrafe(double encoderTicks, double power) throws InterruptedException {
        while (getFurthestEncoder() < encoderTicks) {
            motor1.setPower(power);
            motor2.setPower(-power);
            motor3.setPower(-power);
            motor4.setPower(power);
            sleep(1);
        }
    }

    public int inchesToPositions(double inches) {
        return (int) Math.round(inches * 210 / Math.PI);
    }

    public void resetEncoders() {
        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor4.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void paramDrive(double x, double y, double c) throws InterruptedException {   //this is a function based on the cartesian coordinate system: positive x axis is right strafe, positive y axis is forward/back, positive c is counterclockwise
        //reference is 70rpm for y, x component max/60sec*4.006pi*4/3GR and replace 70 with 20 for c component (420 420 120 in degrees/sec), assumes y is maxed out to determine time needed to travel
        //double seconds = inchestomotorDegs(y)/420; //determines how many seconds the program is expected to run for
        double yRequest = 420 * y / Math.abs(y); //assumes y maxed out, and normalizes it to make sure it's the right direction (SPEED)
        double xRequest = Math.abs(x / y) * 420 * x / Math.abs(x); //bases the x request on the x to y ratio so they end at the same time, then normalizes it to make sure it's the right direction (SPEED)
        double spinRequest;//14.6 wheeldist as a rough estimate to extrapolate how far wheel will turn per second assuming full speed turn of 120 deg/sec
        this.motor1XY = yRequest + xRequest;
        this.motor2XY = yRequest - xRequest;
        this.motor3XY = yRequest - xRequest;
        this.motor4XY = yRequest + xRequest;

        double checkSumYX = Math.abs(yRequest) + Math.abs(xRequest);
        if (checkSumYX > 840) { //if the x and y go over the hardcap we set for speed of move/strafe, normalize it so the ratio remains the same but the fastest motor will go at cap speed
            motor1XY /= checkSumYX * 840;
            motor2XY /= checkSumYX * 840;
            motor3XY /= checkSumYX * 840;
            motor4XY /= checkSumYX * 840;
        }
        double seconds = (Math.abs(inchestomotorDegs(y)) / checkSumYX); //determines how many seconds it will take to travel the full y distance (since x will be based off this, they end at the same time)
        spinRequest = degreestomotorDegs(c) / seconds; //Gets spin speed based off of the time required
        motor1.setVelocity(motor1XY - spinRequest, AngleUnit.DEGREES);
        motor2.setVelocity(motor1XY - spinRequest, AngleUnit.DEGREES);
        motor3.setVelocity(motor1XY + spinRequest, AngleUnit.DEGREES);
        motor4.setVelocity(motor1XY + spinRequest, AngleUnit.DEGREES);
        sleep((long) seconds);
        motor1.setVelocity(0, AngleUnit.DEGREES);
        motor2.setVelocity(0, AngleUnit.DEGREES);
        motor3.setVelocity(0, AngleUnit.DEGREES);
        motor4.setVelocity(0, AngleUnit.DEGREES);
        //turnGyro((int) c, 0.2);
    }

    public double inchestomotorDegs(double inches) {
        return inches / 4.006 * Math.PI * 360 * 4 / 3;
    }

    public double degreestomotorDegs(double degrees) {
        return inchestomotorDegs(degrees / 360 * 14.6 * Math.PI);
    }

    public double getRotation(){
        return Math.toRadians(getYaw()+ 360);
    }

    @Override
    public void stop() {
    }
}