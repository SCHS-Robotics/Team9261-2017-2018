package org.firstinspires.ftc.teamcode;

import android.media.MediaPlayer;
import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.text.SimpleDateFormat;
import java.util.Date;

import static android.os.SystemClock.sleep;
/**
 * Created by Sage Creek Level Up on 10/22/2017.
 */
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Mecanum Debug", group = "OpMode")
public class MecanumDebug extends OpMode{
    private DcMotor motor1; //motors 1 & 3 are left motors
    private DcMotor motor2; //motors 2 & 4 are right motors

    private DcMotor motor3;
    private DcMotor motor4;

    VuforiaLocalizer vuforiaLocalizer;
    VuforiaLocalizer.Parameters parameters;
    VuforiaTrackables visionTargets;
    VuforiaTrackable relicVuMark;
    VuforiaTrackableDefaultListener listener;

    RelicRecoveryVuMark vuMark;
    RelicRecoveryVuMark key;

    OpenGLMatrix lastKnownLocation;
    OpenGLMatrix phoneLocation;

    //XYZ
    double tX;
    double tY;
    double tZ;
    //UVW
    double rX;
    double rY;
    double rZ;

    private static final String VUFORIA_KEY = "AahcXtr/////AAAAGeTM6MJozUDjmFmQyvzpw18JQGmMgCEJS4/mut4gWK23MK4IlXByqZJODNPcUsLluTIPxylZ00ZT+dnztgAgULPHoPca6zxDfRrdHxZaK0rRhdkAubtyi0J3if7ZFxYlC32J2wpWYb0N7QvMO1KfsG5s7fU24IaeXZhK8MeoD6CmnJfaVsa4brdMv2lqy1BUeGikI9FJphmw/JtS9r0FNM0Hk5ditj1qSkiFSYzpdS28Owzlwqudf5ZovyF8GtZ1xcfCpP4GWA1I0SOLxrFsXV74LkjoQi0AGVmnL3EXScKPeGmZPJtbd8oG2AzUUzYWnCwTi1X0+hEccMqG4WB8FsWMzYiYNrmS4+HfGJMExtno";

    private float robotX = 0;
    private float robotY = 0;
    private float robotAngle = 0;

    private CRServo leftcr;
    private CRServo rightcr;
    private Servo glyph;
    private CRServo jewelcr;
    //sprivate I2cDeviceSynch blah;

    private ColorSensor color1;
    @Override
    public void init(){
        motor1 = hardwareMap.dcMotor.get("motor1");
        motor1.setDirection(DcMotor.Direction.REVERSE);
        motor2 = hardwareMap.dcMotor.get("motor2");
        motor2.setDirection(DcMotor.Direction.FORWARD);

        motor3 = hardwareMap.dcMotor.get("motor3");
        motor3.setDirection(DcMotor.Direction.REVERSE);
        motor4 = hardwareMap.dcMotor.get("motor4");
        motor4.setDirection(DcMotor.Direction.FORWARD);

        color1 = hardwareMap.colorSensor.get("color1");
        leftcr = hardwareMap.crservo.get("leftcr");     //leftcr and rightcr are flipped
        rightcr = hardwareMap.crservo.get("rightcr");
        glyph = hardwareMap.servo.get("gservo");
        jewelcr = hardwareMap.crservo.get("jewelcr");
        //blah = hardwareMap.i2cDeviceSynch.get(":(");
        I2cAddr colorAddress = I2cAddr.create8bit(0x3c); //rev is 0x39 7 bit
        color1.setI2cAddress(colorAddress);
        //I2cAddr banana = new I2cAddr(0x88);
        //blah.setI2cAddress(banana);
        telemetry.addData("address", color1.getI2cAddress().toString());
        leftcr.setDirection(CRServo.Direction.REVERSE);
        rightcr.setDirection(CRServo.Direction.FORWARD);
        glyph.setDirection(Servo.Direction.FORWARD);
        jewelcr.setDirection(CRServo.Direction.FORWARD);

        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor4.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        setupVuforia();
        lastKnownLocation = createMatrix(0,0,0,0,0,0);
        visionTargets.activate();
    }

    @Override
    public void init_loop(){

    }

    @Override
    public void start(){

    }

    @Override
    public void loop(){
        OpenGLMatrix latestLocation = listener.getUpdatedRobotLocation();
        vuMark = RelicRecoveryVuMark.from(relicVuMark);

        if(latestLocation !=null) {lastKnownLocation = latestLocation;}
        float[] coordinates = lastKnownLocation.getTranslation().getData();
        robotX = coordinates[0];
        robotY = coordinates[1];
        robotAngle = Orientation.getOrientation(lastKnownLocation, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
        if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
            key = vuMark;
        }
        double x = gamepad1.right_stick_x;
        double y = gamepad1.right_stick_y;
        double c = gamepad1.left_trigger-gamepad1.right_trigger;
        motor1.setPower(Range.clip(y-x+c, -1, 1));
        motor2.setPower(Range.clip(y+x-c, -1, 1));
        motor3.setPower(Range.clip(y+x+c, -1, 1));
        motor4.setPower(Range.clip(y-x-c, -1, 1));

        if (gamepad2.right_trigger > 0) {
            leftcr.setPower(1);
            rightcr.setPower(1);
        }else if(gamepad2.left_trigger > 0) {
            leftcr.setPower(-1);
            rightcr.setPower(-1);
        }else {
            leftcr.setPower(0.1);
            rightcr.setPower(0);
        }
        if(gamepad2.x) {
            glyph.setPosition(1);
        }
        else if(gamepad2.a) {
            glyph.setPosition(0);
        }
        else if(gamepad2.b) {
            glyph.setPosition(-1);
        }

        telemetry.addData("motor 1 pos", motor1.getCurrentPosition());
        telemetry.addData("motor 2 pos", motor2.getCurrentPosition());
        telemetry.addData("motor 3 pos", motor3.getCurrentPosition());
        telemetry.addData("motor 4 pos", motor4.getCurrentPosition());
        telemetry.addData("jewel servo pos", jewelcr.getPower());
        updateLocation();
        telemetry.addData("Last Known Location", formatMatrix(lastKnownLocation));

        telemetry.addData("Red",color1.red());
        telemetry.addData("Blue",color1.blue());

        telemetry.update();
    }

    @Override
    public void stop(){}

    public void updateLocation(){
        OpenGLMatrix pose = ((VuforiaTrackableDefaultListener)relicVuMark.getListener()).getPose();
        telemetry.addData("Pose", format(pose));

        /* We further illustrate how to decompose the pose into useful rotational and
        * translational components */
        if (pose != null) {
            VectorF trans = pose.getTranslation();
            Orientation rot = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

            // Extract the X, Y, and Z components of the offset of the target relative to the robot
            tX = trans.get(0);
            tY = trans.get(1);
            tZ = trans.get(2);

            // Extract the rotational components of the target relative to the robot
            rX = rot.firstAngle;
            rY = rot.secondAngle;
            rZ = rot.thirdAngle;
            //Z is forward-backward
            //x is sideways
        }
    }

    public void setupVuforia()
    {
        parameters = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId); //remove parameters to hide phone tracking
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        parameters.useExtendedTracking = true; //extended tracking can be inaccurate

        vuforiaLocalizer = ClassFactory.createVuforiaLocalizer(parameters);
        visionTargets = vuforiaLocalizer.loadTrackablesFromAsset("RelicVuMark");

        relicVuMark = visionTargets.get(0);
        relicVuMark.setName("RelicRecovery");
        relicVuMark.setLocation(createMatrix(0,0,0,0,0,0));

        phoneLocation = createMatrix(0,0,0,0,0,0);

        listener = (VuforiaTrackableDefaultListener) relicVuMark.getListener();
        listener.setPhoneInformation(phoneLocation, parameters.cameraDirection);
    }

    public OpenGLMatrix createMatrix(float x,float y,float z,float u,float v,float w)
    {
        return OpenGLMatrix.translation(x,y,z).multiplied(Orientation.getRotationMatrix(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES, u,v,w));
    }

    public String formatMatrix(OpenGLMatrix matrix)
    {
        return matrix.formatAsTransform();
    }

    String format(OpenGLMatrix transformationMatrix) {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }
}
