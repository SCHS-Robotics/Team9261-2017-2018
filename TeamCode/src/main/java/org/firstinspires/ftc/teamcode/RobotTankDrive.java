package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Func;

import static java.lang.Thread.sleep;

/**
 * Created by Andrew on 3/4/2017.
 */

public class RobotTankDrive
{
    private LinearOpMode driveTrain;

    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;

    private double driveAxial = 0;
    private double driveYaw = 0;

    public RobotTankDrive()
    {

    }
    public void initDrive(LinearOpMode opMode)
    {
        driveTrain = opMode;

        leftDrive   = driveTrain.hardwareMap.get(DcMotor.class, "Left Drive");
        rightDrive  = driveTrain.hardwareMap.get(DcMotor.class, "Right Drive");

        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);

        setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        moveRobot(0,0);

    }
    public void manualDrive()
    {
        setAxial(-driveTrain.gamepad1.right_stick_y);
        setYaw(-(driveTrain.gamepad1.right_trigger - driveTrain.gamepad2.left_trigger));
    }
    public void moveRobot(double axial, double yaw)
    {
        setAxial(axial);
        setYaw(yaw);

        moveRobot();
    }
    public void moveRobot()
    {
        // calculate required motor speeds to achieve axis motions
        double left = driveYaw - driveAxial;
        double right = driveYaw + driveAxial;

        // normalize all motor speeds so no values exceeds 100%.
        double max = Math.max(Math.abs(right), Math.abs(left));
        if (max > 1.0)
        {
            right /= max;
            left /= max;
        }

        // Set drive motor power levels.
        leftDrive.setPower(left);
        rightDrive.setPower(right);

        // Display Telemetry
        driveTrain.telemetry.addData("Axes  ", "A[%+5.2f], Y[%+5.2f]", driveAxial, driveYaw);
        driveTrain.telemetry.addData("Wheels", "L[%+5.2f], R[%+5.2f]", left, right);
    }
    public void setAxial(double axial)      {driveAxial = Range.clip(axial, -1, 1);}
    public void setYaw(double yaw)          {driveYaw = Range.clip(yaw, -1, 1); }
    public void setMode(DcMotor.RunMode mode)
    {
        leftDrive.setMode(mode);
        rightDrive.setMode(mode);
    }
    private int inchesTheory(double inches)
    {
        return (int) Math.round(inches*280/Math.PI);
    }
    private int degreesTheory(double degrees)
    {
        return (int) Math.round(degrees * 21);
    }
    public void curveDrive(double inches, double degrees, double power, double radiusofTurn, String leftorright, String fwdBck) throws InterruptedException // power max is 0.5
    {
        double pwrProportion;
        int rightInches;
        int leftInches;
        double wheelDist = 13.5;
        String Curve = "Curve";
        if (fwdBck == "Back" || fwdBck == "back")
        {
            power = -power;
        }
        if (leftorright == "left" || leftorright =="Left")
        {
            rightInches = inchesTheory(degrees/360 * Math.PI * 2 * (radiusofTurn + wheelDist));
            leftInches = inchesTheory(degrees/360 * Math.PI * 2 * radiusofTurn);
            pwrProportion = Range.clip(rightInches/leftInches, 1, 2);
            resetEncoders();
            waitforEncoders(leftInches, rightInches, power, pwrProportion, leftorright);
        }
        else if (leftorright == "right" || leftorright == "Right")
        {
            leftInches = inchesTheory(degrees/360 * Math.PI * 2 * (radiusofTurn + wheelDist));
            rightInches = inchesTheory(degrees/360 * Math.PI * 2 * radiusofTurn);
            pwrProportion = Range.clip(leftInches/rightInches, 1, 2);
            resetEncoders();
            waitforEncoders(leftInches, rightInches, power, pwrProportion, leftorright);
        }
        leftDrive.setPower(0);
        rightDrive.setPower(0);

    }
    public void waitforEncoders(int leftInches, int rightInches, double power, double pwrProportion, String leftorright) throws InterruptedException
    {
        while (leftDrive.getCurrentPosition()/leftInches < 1 && rightDrive.getCurrentPosition()/rightInches < 1 && driveTrain.opModeIsActive())
        {
            if (leftorright == "Right"){
                leftDrive.setPower(power * pwrProportion * lBrake(leftInches));
                rightDrive.setPower(power * rBrake(rightInches));
                sleep(1);
            }
            else
            {
                leftDrive.setPower(power * lBrake(leftInches));
                rightDrive.setPower(power * pwrProportion * rBrake(rightInches));
                sleep(1);
            }
        }
    }
    private double lBrake(double encoderTicks)
    {
        return Range.clip(Math.cbrt(Math.abs(leftDrive.getCurrentPosition() - encoderTicks)/ encoderTicks), 0.2, 1);
    }
    private double rBrake(double encoderTicks)
    {
        return Range.clip(Math.cbrt(Math.abs(rightDrive.getCurrentPosition() - encoderTicks)/ encoderTicks), 0.2, 1);
    }
    public void resetEncoders(DcMotor... motors)
    {
        for (DcMotor motor: motors)
        {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
    private int getFurthestEncoder()
    {
        return Math.max(Math.abs(leftDrive.getCurrentPosition()), Math.abs(rightDrive.getCurrentPosition()));
    }
    public void drivewithPID(double targSpd, double targDist) throws InterruptedException
    {
        targDist = inchesTheory(targDist);
        double oldError, PIDCnst, dt;
        double error, errorSum, dError;
        double kp, ki, kd;
        oldError = 0;
        errorSum = 0;
        while(getFurthestEncoder() < targDist && driveTrain.opModeIsActive())
        {
            dt = 30;
            error = targDist - getFurthestEncoder();
            errorSum += error;
            dError = error - oldError;
            kp = 0.001;
            ki = 0.001;
            kd = 0.001;
            PIDCnst = error * kp + errorSum * ki + dError * kd;
            leftDrive.setPower(targSpd - PIDCnst);
            rightDrive.setPower(targSpd - PIDCnst);
            oldError = error;
            sleep((long)dt);
        }
    }
    public void drivewithPIDBackward(double targSpd, double targDist) throws InterruptedException
    {
        drivewithPID(-targSpd, targDist);
    }
    public void turnForward(int degreesleft, int degreesright, double powerleft, double powerright) throws InterruptedException
    {
        int encoderTicksleft = degreesTheory(degreesleft);
        int encoderTicksright = degreesTheory(degreesright);
        resetEncoders();
        waitforEncodersTurn(encoderTicksleft, encoderTicksright, powerleft, powerright);
        leftDrive.setPower(0);
        rightDrive.setPower(0);
    }
    public void turnBackward(int degreesleft, int degreesright, double powerleft, double powerright) throws InterruptedException
    {
        turnForward(degreesleft, degreesright, -powerleft, -powerright);
    }
    private void waitforEncodersTurn(double encoderTicksleft, double encoderTicksright, double powerleft, double powerright) throws InterruptedException
    {
        while(getFurthestEncoder() < encoderTicksleft || getFurthestEncoder() < encoderTicksright && driveTrain.opModeIsActive())
        {
            if (encoderTicksleft!=0) {
                leftDrive.setPower(powerleft * lBrake(encoderTicksleft));
            }
            else if (encoderTicksright!=0) {
                rightDrive.setPower(powerright * rBrake(encoderTicksright));
            }
            sleep(1);
        }
    }
    private void waitforEncoders(double encoderTicks, double power) throws InterruptedException
    {
        while (getFurthestEncoder() < encoderTicks && driveTrain.opModeIsActive())
        {
            leftDrive.setPower(power * lBrake(encoderTicks));
            rightDrive.setPower(power * rBrake(encoderTicks));
            sleep(1);
        }
    }
}
