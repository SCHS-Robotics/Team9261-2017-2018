package org.firstinspires.ftc.teamcode.season.relicrecov18.polaris.v1.robot;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManager;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.util.Vector;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.SubSystem;

import static java.lang.Thread.sleep;

/**
 * Created by Andrew on 12/6/2017.
 */

public class Drive extends SubSystem {
    private DcMotor motor1, motor2, motor3, motor4;
    private BNO055IMU imu;
    public int homeAngle;
    double globalAngle;

    public Drive(Robot robot){
        super(robot);
    }

    @Override
    public void init() throws InterruptedException {
        motor1 = robot.hardwareMap.dcMotor.get("motor1");
        motor2 = robot.hardwareMap.dcMotor.get("motor2");
        motor3 = robot.hardwareMap.dcMotor.get("motor3");
        motor4 = robot.hardwareMap.dcMotor.get("motor4");

        motor1.setDirection(DcMotor.Direction.FORWARD);
        motor2.setDirection(DcMotor.Direction.REVERSE);
        motor3.setDirection(DcMotor.Direction.FORWARD);
        motor4.setDirection(DcMotor.Direction.REVERSE);

        modeEncoders();
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        imu = robot.hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);

        robot.telemetry.addData("Mode", "calibrating...");
        robot.telemetry.update();

        while(!imu.isGyroCalibrated()){
            sleep(50);
        }

        robot.telemetry.addData("Mode", "waiting for start");
        robot.telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        robot.telemetry.addData("Angle", getAngle());
        robot.telemetry.update();

    }

    @Override
    public void handle() {
        Vector inputVector = new Vector(robot.gamepad1.left_stick_x, -robot.gamepad1.left_stick_y); //gamepads are weird, -1 is at the top of the y axis
        Vector motionVector = inputVector.rotate(-Math.PI / 4);
        if (robot.gamepad1.left_bumper && motionVector.isZeroVector()) {
            motor1.setPower(-1);
            motor2.setPower(1);
            motor3.setPower(1);
            motor4.setPower(-1);

        }
        else if (robot.gamepad1.right_bumper && motionVector.isZeroVector()) {
            motor1.setPower(1);
            motor2.setPower(-1);
            motor3.setPower(-1);
            motor4.setPower(1);
        } else {
            double offset = (robot.gamepad1.left_trigger - robot.gamepad1.right_trigger) / 4;
            motor1.setPower(Range.clip(motionVector.x - offset, -1, 1));
            motor2.setPower(Range.clip(motionVector.y + offset, -1, 1));
            motor3.setPower(Range.clip(motionVector.y - offset, -1, 1));
            motor4.setPower(Range.clip(motionVector.x + offset, -1, 1));

        }

    }

    public void modeEncoders(){
        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor4.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void driveForward(double power){
        motor1.setPower(power);
        motor2.setPower(power);
        motor3.setPower(power);
        motor4.setPower(power);
    }
    public void strafeDistance(double distance, double power)throws InterruptedException{
        double encoderTicks = inchesToPositions(distance);
        resetEncoders();
        waitforEncodersStrafe(encoderTicks, power);
        stopDrive();
    }
    public void driveDistance(double distance, double power)throws InterruptedException{
        double encoderTicks = inchesToPositions(distance);
        resetEncoders();
        waitforEncoders(encoderTicks, power);
        stopDrive();
    }
    public void turnGyro(int degrees, double power)throws InterruptedException{

        if (degrees<0){
            motor1.setPower(power);
            motor2.setPower(-power);
            motor3.setPower(power);
            motor4.setPower(-power);
        }
        else if(degrees>0){
            motor1.setPower(-power);
            motor2.setPower(power);
            motor3.setPower(-power);
            motor4.setPower(power);
        }
        while(getAngle()< Math.abs(degrees) ){
            robot.telemetry.addData("Angle", getAngle());
            robot.telemetry.update();
            sleep(1);
        }
        stopDrive();
    }
    public void stopDrive(){
        motor1.setPower(0);
        motor2.setPower(0);
        motor3.setPower(0);
        motor4.setPower(0);
    }
    public double getAngle(){
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;
    }

    private int getFurthestEncoder()
    {
        int left = Math.max(Math.abs(motor1.getCurrentPosition()), Math.abs(motor3.getCurrentPosition()));
        int right =Math.max(Math.abs(motor2.getCurrentPosition()), Math.abs(motor4.getCurrentPosition()));
        int both = Math.max(left, right);
        return both;
    }
    private void waitforEncoders(double encoderTicks, double power)throws InterruptedException{
        while (getFurthestEncoder() < encoderTicks){
            motor1.setPower(power);
            motor2.setPower(power);
            motor3.setPower(power);
            motor4.setPower(power);
            sleep(1);
        }
    }
    private void waitforEncodersStrafe(double encoderTicks, double power)throws InterruptedException{
        while (getFurthestEncoder() < encoderTicks){
            motor1.setPower(power);
            motor2.setPower(-power);
            motor3.setPower(-power);
            motor4.setPower(power);
            sleep(1);
        }
    }
    public int inchesToPositions(double inches) {
        return (int) Math.round(inches*210/Math.PI);
    }
    public void resetEncoders()
    {
        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor4.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    @Override
    public void stop() {

    }
}
