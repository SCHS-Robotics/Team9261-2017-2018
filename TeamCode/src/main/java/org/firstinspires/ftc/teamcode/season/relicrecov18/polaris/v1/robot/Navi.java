package org.firstinspires.ftc.teamcode.season.relicrecov18.polaris.v1.robot;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.SubSystem;

/**
 * Created by Sage Creek Level Up on 12/8/2017.
 */



public class Navi extends SubSystem {
    public Navi(Robot robot){
        super(robot);
    }

    VuforiaLocalizer vuforiaLocalizer;
    VuforiaLocalizer.Parameters parameters;
    VuforiaTrackables visionTargets;
    VuforiaTrackable relicVuMark;
    VuforiaTrackableDefaultListener listener;

    RelicRecoveryVuMark vuMark;
    public RelicRecoveryVuMark key;

    OpenGLMatrix lastKnownLocation;
    OpenGLMatrix phoneLocation;

    private static final String VUFORIA_KEY = "AahcXtr/////AAAAGeTM6MJozUDjmFmQyvzpw18JQGmMgCEJS4/mut4gWK23MK4IlXByqZJODNPcUsLluTIPxylZ00ZT+dnztgAgULPHoPca6zxDfRrdHxZaK0rRhdkAubtyi0J3if7ZFxYlC32J2wpWYb0N7QvMO1KfsG5s7fU24IaeXZhK8MeoD6CmnJfaVsa4brdMv2lqy1BUeGikI9FJphmw/JtS9r0FNM0Hk5ditj1qSkiFSYzpdS28Owzlwqudf5ZovyF8GtZ1xcfCpP4GWA1I0SOLxrFsXV74LkjoQi0AGVmnL3EXScKPeGmZPJtbd8oG2AzUUzYWnCwTi1X0+hEccMqG4WB8FsWMzYiYNrmS4+HfGJMExtno";

    /*
    private float robotX = 0;
    private float robotY = 0;
    private float robotAngle = 0;
    */
    @Override
    public void init(){
        key = RelicRecoveryVuMark.UNKNOWN;
        setupVuforia();
        startVuforia();
        for(int i=0;i<10;i++){
            updateVuforia();
        }
        if(key!=RelicRecoveryVuMark.UNKNOWN){
            stopVuforia();
        }
    }
    @Override
    public void handle(){

    }
    @Override
    public void stop(){

    }
    public void setupVuforia()
    {
        parameters = new VuforiaLocalizer.Parameters(); //remove parameters to hide phone tracking
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        parameters.useExtendedTracking = true; //extended tracking is quite inaccurate
        vuforiaLocalizer = ClassFactory.createVuforiaLocalizer(parameters);

        visionTargets = vuforiaLocalizer.loadTrackablesFromAsset("RelicVuMark");

        relicVuMark = visionTargets.get(0);
        relicVuMark.setName("RelicVuMark");
        //relicVuMark.setLocation(createMatrix(0,0,0,0,0,0));

        //phoneLocation = createMatrix(0,0,0,0,0,0);

        listener = (VuforiaTrackableDefaultListener) relicVuMark.getListener();
        //listener.setPhoneInformation(phoneLocation, parameters.cameraDirection);
    }

    public void updateVuforia(){
        //OpenGLMatrix latestLocation = listener.getUpdatedRobotLocation();
        vuMark = RelicRecoveryVuMark.from(relicVuMark);

        /*
        if(latestLocation !=null)
        {lastKnownLocation = latestLocation;}

        float[] coordinates = lastKnownLocation.getTranslation().getData();

        robotX = coordinates[0];
        robotY = coordinates[1];
        robotAngle = Orientation.getOrientation(lastKnownLocation, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
        */
        if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
            key = vuMark;
        }
        robot.telemetry.addData("key", key);
        robot.telemetry.update();
    }

    public void startVuforia(){
        visionTargets.activate();
    }
    public void stopVuforia(){
        visionTargets.deactivate();
    }

    public OpenGLMatrix createMatrix(float x, float y, float z, float u, float v, float w)
    {
        return OpenGLMatrix.translation(x,y,z).
                multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES, u,v,w));
    }

    public String formatMatrix(OpenGLMatrix matrix)
    {
        return matrix.formatAsTransform();
    }

}
