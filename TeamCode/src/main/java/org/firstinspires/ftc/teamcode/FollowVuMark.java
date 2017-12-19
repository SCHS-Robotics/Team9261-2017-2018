package org.firstinspires.ftc.teamcode;



import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.hardware.DcMotorSimple;

import com.qualcomm.robotcore.util.Range;



import org.firstinspires.ftc.robotcore.external.ClassFactory;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;



import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;

import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;



@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "FollowVuforia", group = "Autonomous")

public class FollowVuMark extends LinearOpMode

{

    private DcMotor motora;

    private DcMotor motorb;

    private DcMotor motorc;

    private DcMotor motord;



    public enum State{

        FINDINGVUMARK, FOLLOWINGVUMARK

    }



    State state;



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

    public void runOpMode() throws InterruptedException
    {
        motora = hardwareMap.dcMotor.get("motor1");
        motorb = hardwareMap.dcMotor.get("motor2");
        motorc = hardwareMap.dcMotor.get("motor4");
        motord = hardwareMap.dcMotor.get("motor3");

        motora.setDirection(DcMotorSimple.Direction.REVERSE);
        motorb.setDirection(DcMotorSimple.Direction.FORWARD);
        motorc.setDirection(DcMotorSimple.Direction.FORWARD);
        motord.setDirection(DcMotorSimple.Direction.REVERSE);

        state = State.FINDINGVUMARK;
        setupVuforia();
        lastKnownLocation = createMatrix(0,0,0,0,0,0);
        waitForStart();

        visionTargets.activate();



        while(opModeIsActive())

        {

            OpenGLMatrix latestLocation = listener.getUpdatedRobotLocation();

            vuMark = RelicRecoveryVuMark.from(relicVuMark);

            if(latestLocation !=null)

            {lastKnownLocation = latestLocation;}

            float[] coordinates = lastKnownLocation.getTranslation().getData();



            robotX = coordinates[0];

            robotY = coordinates[1];

            robotAngle = Orientation.getOrientation(lastKnownLocation, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;

            if (state == State.FINDINGVUMARK) {

                if (vuMark != RelicRecoveryVuMark.UNKNOWN) {

                    key = vuMark;

                    state = State.FOLLOWINGVUMARK;
                    
                }

            } else{



                if(rX > 0){ //rotate left around picture

                    move(-1,0,0.25);

                }else if (rX < 0){  //rotate right around picture

                    move(1,0,-0.25);



                }else if(tX > 0){ //strafe left

                    move(-1,0,0);

                }else if (tX < 0){  //strafe right

                    move(1,0,0);



                }else if(tZ > 0){ //move forward

                    move(0,1,0);

                }else if (tZ < 0){ //move backward

                    move(0,-1,0);

                }else{

                    stopMotors();

                }

            }

            updateLocation();

            telemetry.addData("Last Known Location", formatMatrix(lastKnownLocation));  //

            telemetry.addData("State", state);

            telemetry.addData("CryptoKey", key);

            telemetry.update();

        }

    }



    public void move(double x, double y, double offset){    //offset = (gamepad1.left_trigger-gamepad1.right_trigger)/4

        Vector inputVector = new Vector(x,-y); //gamepads are weird, -1 is at the top of the y axis

        Vector motionVector = inputVector.rotate(-Math.PI/4);



        motora.setPower(Range.clip(motionVector.x - offset,-1,1));

        motorb.setPower(Range.clip(motionVector.y + offset,-1,1));

        motorc.setPower(Range.clip(motionVector.x + offset,-1,1));

        motord.setPower(Range.clip(motionVector.y - offset,-1,1));

    }



    public void stopMotors(){

        motora.setPower(0);

        motorb.setPower(0);

        motorc.setPower(0);

        motord.setPower(0);

    }



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

            // Z is forward-backward
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

        return OpenGLMatrix.translation(x,y,z).

                multiplied(Orientation.getRotationMatrix(

                        AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES, u,v,w));

    }



    public String formatMatrix(OpenGLMatrix matrix)

    {

        return matrix.formatAsTransform();

    }



    String format(OpenGLMatrix transformationMatrix) {

        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";

    }





}