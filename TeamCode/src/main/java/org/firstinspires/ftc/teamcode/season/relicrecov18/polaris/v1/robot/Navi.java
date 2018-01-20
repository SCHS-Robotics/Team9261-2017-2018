package org.firstinspires.ftc.teamcode.season.relicrecov18.polaris.v1.robot;

import android.graphics.Bitmap;

import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;

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
import org.opencv.android.Utils;
import org.opencv.core.Mat;

/**
 * Created by Sage Creek Level Up on 12/8/2017.
 */



public class Navi extends SubSystem {
    public Navi(Robot robot){
        super(robot);
    }

    public VuforiaLocalizer vuforiaLocalizer;
    public VuforiaLocalizer.Parameters parameters;
    public VuforiaTrackables visionTargets;
    public VuforiaTrackable relicVuMark;
    public VuforiaTrackableDefaultListener listener;

    public RelicRecoveryVuMark vuMark;
    public RelicRecoveryVuMark key;

    public boolean vuforiaSetup = false;

    public static final String VUFORIA_KEY = "AahcXtr/////AAAAGeTM6MJozUDjmFmQyvzpw18JQGmMgCEJS4/mut4gWK23MK4IlXByqZJODNPcUsLluTIPxylZ00ZT+dnztgAgULPHoPca6zxDfRrdHxZaK0rRhdkAubtyi0J3if7ZFxYlC32J2wpWYb0N7QvMO1KfsG5s7fU24IaeXZhK8MeoD6CmnJfaVsa4brdMv2lqy1BUeGikI9FJphmw/JtS9r0FNM0Hk5ditj1qSkiFSYzpdS28Owzlwqudf5ZovyF8GtZ1xcfCpP4GWA1I0SOLxrFsXV74LkjoQi0AGVmnL3EXScKPeGmZPJtbd8oG2AzUUzYWnCwTi1X0+hEccMqG4WB8FsWMzYiYNrmS4+HfGJMExtno";

    @Override
    public void init(){
    }
    @Override
    public void handle(){

    }
    @Override
    public void stop(){

    }
    public void setupVuforia() {
        parameters = new VuforiaLocalizer.Parameters(); //remove parameters(R.id.cameraMonitorViewId) to hide phone tracking
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        parameters.useExtendedTracking = false; //extended tracking is quite inaccurate
        vuforiaLocalizer = ClassFactory.createVuforiaLocalizer(parameters);

        visionTargets = vuforiaLocalizer.loadTrackablesFromAsset("RelicVuMark");

        relicVuMark = visionTargets.get(0);
        relicVuMark.setName("RelicVuMark");

        listener = (VuforiaTrackableDefaultListener) relicVuMark.getListener();
        key = RelicRecoveryVuMark.UNKNOWN;
        vuforiaSetup = true;
    }

    public void setKey(){
        if(key == RelicRecoveryVuMark.UNKNOWN){
            int random = (int) Math.ceil(Math.random()*3);
            if(random == 1){
                key = RelicRecoveryVuMark.LEFT;
            }else if(random == 3){
                key = RelicRecoveryVuMark.RIGHT;
            }else{
                key = RelicRecoveryVuMark.CENTER;
            }
        }
    }

    public Mat getFrame() {
        Image vuforiaFrame;
        try {
            VuforiaLocalizer.CloseableFrame frame = vuforiaLocalizer.getFrameQueue().take();
            long numImages = frame.getNumImages();
            for (int i = 0; i < numImages; i++) {
                if (frame.getImage(i).getFormat() == PIXEL_FORMAT.RGB565) {
                    vuforiaFrame = frame.getImage(i);
                    Bitmap bitmap = Bitmap.createBitmap(vuforiaFrame.getWidth(), vuforiaFrame.getHeight(), Bitmap.Config.RGB_565);
                    bitmap.copyPixelsFromBuffer(vuforiaFrame.getPixels());
                    Bitmap bmp32 = bitmap.copy(Bitmap.Config.ARGB_8888, true);
                    Mat output = new Mat();
                    Utils.bitmapToMat(bmp32, output);
                    return output;
                }
            }
        }
        catch (Exception e) {
            e.printStackTrace();
        }
        return  null;
    }

    public void updateVuforia() {
        //OpenGLMatrix latestLocation = listener.getUpdatedRobotLocation();
        vuMark = RelicRecoveryVuMark.from(relicVuMark);

        if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
            key = vuMark;
        }
    }

    public void startVuforia() {
        if(visionTargets!=null && key==RelicRecoveryVuMark.UNKNOWN) {
            visionTargets.activate();
        }
    }

    public void stopVuforia() {
        if(visionTargets!=null) {
            visionTargets.deactivate();
        }
        vuforiaSetup = false;
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
