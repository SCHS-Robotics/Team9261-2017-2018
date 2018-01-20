package org.firstinspires.ftc.teamcode.season.relicrecov18.polaris.v1.robot;

import android.graphics.Bitmap;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.season.relicrecov18.polaris.v1.program.PolarisAutonomousProgram;
import org.firstinspires.ftc.teamcode.season.relicrecov18.polaris.v1.program.programs.BallDetector;
import org.firstinspires.ftc.teamcode.season.relicrecov18.polaris.v1.program.programs.Circle;
import org.firstinspires.ftc.teamcode.AutoTransitioner;
import org.opencv.android.CameraBridgeViewBase;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;

/**
 * Created by Andrew on 12/7/2017.
 */
@Autonomous(name = "B", group = "Autonomous")
public class BlueAutoB extends PolarisAutonomousProgram {
    private String direction;

    public VuforiaLocalizer vuforiaLocalizer;
    public VuforiaLocalizer.Parameters parameters;
    public VuforiaTrackables visionTargets;
    public VuforiaTrackable relicVuMark;
    public VuforiaTrackableDefaultListener listener;

    public RelicRecoveryVuMark vuMark;
    public RelicRecoveryVuMark key;
    private static final String VUFORIA_KEY = "AahcXtr/////AAAAGeTM6MJozUDjmFmQyvzpw18JQGmMgCEJS4/mut4gWK23MK4IlXByqZJODNPcUsLluTIPxylZ00ZT+dnztgAgULPHoPca6zxDfRrdHxZaK0rRhdkAubtyi0J3if7ZFxYlC32J2wpWYb0N7QvMO1KfsG5s7fU24IaeXZhK8MeoD6CmnJfaVsa4brdMv2lqy1BUeGikI9FJphmw/JtS9r0FNM0Hk5ditj1qSkiFSYzpdS28Owzlwqudf5ZovyF8GtZ1xcfCpP4GWA1I0SOLxrFsXV74LkjoQi0AGVmnL3EXScKPeGmZPJtbd8oG2AzUUzYWnCwTi1X0+hEccMqG4WB8FsWMzYiYNrmS4+HfGJMExtno";

    private final ExecutorService service = Executors.newFixedThreadPool(2); //The executor service that will run the two processing threads simultaneously

    //The circles previously returned by the two threads
    private Circle prevRedCircle = new Circle(new Point(0,0),0);;
    private Circle prevBlueCircle = new Circle(new Point(0,0),0);;
    @Override
    protected Robot buildRobot() {
        Robot robot = super.buildRobot();
        return robot;
    }

    @Override
    public void main() throws InterruptedException {
        drive.initGyro();
        AutoTransitioner.transitionOnStop(this, "Competition TeleOp");
        while(!drive.imu.isGyroCalibrated() && !isStarted() && !isStopRequested()){
            sleep(50);
        }
        drive.notifyGyroReady();
        setupVuforia();
        startVuforia();
        while(key == RelicRecoveryVuMark.UNKNOWN && !isStarted() && !isStopRequested()) {
            updateVuforia();
        }

        telemetry.addData("key",key);
        //jewelDetector.startOpenCV(jewelDetector);
        //startOpenCV(this);

        while(!isStarted() && !isStopRequested()) {
            opencvStuff(getFrame());
        }

        waitForStart();
        //jewelDetector.stopOpenCV();
        //sleep(500);
        //sleep(2000);
        /*
        belt.electionIsRigged();
        sleep(1000);
        //navi.stopVuforia();
        //stopOpenCV();

        belt.deploy();
        sleep(2000);
        belt.stopDeploy();

        /*startOpenCV(this);
        sleep(1000);
        stopOpenCV();*/
        jewel.moveAxe(Jewel.axePos.UPPERMID);
        belt.deploy();
        sleep(500);
        jewel.moveAxe(Jewel.axePos.LOWERMID);
        sleep(500);
        jewel.moveAxe(Jewel.axePos.DOWN);
        sleep(250);
        if(jewelDetector.direction == "left") {
            jewel.moveArm(Jewel.armPos.LEFT);
        }
        else if(jewelDetector.direction == "right") {
            jewel.moveArm(Jewel.armPos.RIGHT);
        }
        sleep(1000);
        belt.stopDeploy();
        jewel.moveArm(Jewel.armPos.MIDDLE);
        jewel.moveAxe(Jewel.axePos.UP);
        sleep(1000);
        belt.putThatWallUp();
        belt.intakeBlock();
        belt.spitBlock();
        sleep(1000);
        belt.rigThatElection();
        belt.stopIntake();
        belt.stopSpitting();
        belt.clipBlock();
        //drive.driveoffBalancingStone();
        drive.driveMaintainYaw(30, 0.6, 0, 1);
        drive.turnExact(0.2, -45, 1);
        drive.driveMaintainYaw(38, -0.8, -45, 1 );
        drive.turnExact(0.2, 0, 1);
        /*if (navi.key == RelicRecoveryVuMark.LEFT){
            drive.strafeMaintainYaw(6, -.5, 0, 1);
        }else if(navi.key == RelicRecoveryVuMark.RIGHT){
            drive.strafeMaintainYaw(6, .5, 0, 1);
        }*/
        belt.unclipBlock();
        belt.gateGranted();
        drive.driveMaintainYaw(5, -0.2, 0, 1);
        sleep(500);
        belt.spitBlock();
        sleep(3000);
        belt.stopSpitting();
        belt.requestGate();
        drive.driveMaintainYaw(2, 0.6, 0, 1);
        sleep(500);
        drive.driveMaintainYaw(12, 0.8, 0, 1);
        belt.intakeBlock();
        belt.spitBlock();
        drive.driveMaintainYaw(10, 0.4, 0, 1);
        belt.spitintake();
        sleep(500);
        belt.stopIntake();
        drive.driveMaintainYaw(11, 0.2, 0,1 );
        belt.stopSpitting();
        belt.clipBlock();
        belt.gateGranted();
        drive.driveMaintainYaw(22, -0.8, 0, 1);
        belt.unclipBlock();
        belt.spitBlock();
        sleep(3000);
        belt.stopSpitting();
        sleep(500);
        drive.driveMaintainYaw(2, 0.6, 0, 1);
        drive.driveMaintainYaw(2, -0.6, 0, 1);
        sleep(500);
        belt.spitBlock();
        sleep(750);
        belt.stopSpitting();
        drive.driveMaintainYaw(2, 0.6, 0, 1);}
        /*drive.driveDistance(20, -0.5);//correct
        drive.turnGyro(45, 0.2);
        drive.driveDistance(30, 0.5);//correct
        drive.turnGyro(0, 0.2);
*/
        //startOpenCV(this);
        // RelicRecoveryVuMark key = RelicRecoveryVuMark.UNKNOWN;
        //long utcOffset = TimeZone.getDefault().getOffset(System.currentTimeMillis());
        // boolean state = true;
        /*
        while(opModeIsActive() && type == RelicRecoveryVuMark.UNKNOWN && state) {
            key = navi.key;
            int secondsPassed = (int)((System.currentTimeMillis() + utcOffset) % DateUtils.DAY_IN_MILLIS / 1000);
            if(secondsPassed > 5) {
                telemetry.addData("Timed Out", "Of Vuforia");
                state = false;
            }
        }

        */
        // telemetry.addData("key",key);
        //telemetry.update();
        /*
        belt.deploy();
        jewel.moveAxe(Jewel.axePos.UP);
        sleep(3000);
        jewel.moveArm(Jewel.armPos.UP);
        if (direction =="left") {
            telemetry.addLine("I would have hit left");
            sleep(1000);
            jewel.moveArm(Jewel.armPos.RIGHT);
        }
        else if (direction == "right"){
            telemetry.addLine("I would have hit right");
            sleep(1000);
            jewel.moveArm(Jewel.armPos.LEFT);
        }
        sleep(1000);
        jewel.moveAxe(Jewel.axePos.FRONT);
        sleep(1000);
        jewel.moveArm(Jewel.armPos.LEFT);
        sleep(1000);
        jewel.moveAxe(Jewel.axePos.DOWN);
        stopOpenCV();
        /*
        sleep(1000);
        drive.driveDistance(30, -0.8);
        sleep(2000);
        /*
        if(key == RelicRecoveryVuMark.LEFT)
        drive.strafeDistance(6.1, -0.8);
        }else if(key == RelicRecoveryVuMark.RIGHT){
        drive.strafeDistance(6.1, 0.8);
        }
        sleep(1000);

        drive.turnGyro(-90, 0.2);
        sleep(1000);
        drive.driveDistance(1.5,-0.8);
        belt.spitBlock();
        sleep(2000);
        belt.stopSpitting();
        drive.driveDistance(3, -0.8);
        sleep(2000);
        drive.driveDistance(3,0.8);
*/
    /*
    @Override
    public Mat onCameraFrame(CameraBridgeViewBase.CvCameraViewFrame inputFrame) {

        ArrayList<Mat> rgbaChannels = new ArrayList<>();

        Mat raw = inputFrame.rgba();
        Mat hsv = new Mat();
        Mat lowerRedRange = new Mat();
        Mat upperRedRange = new Mat();
        Mat red = new Mat();
        Mat blue = new Mat();

        Mat redChannel;
        Mat blueChannel;

        //Future<Circle> means that this task will someday return a circle, but not right now
        final Future<Circle> task1;
        final Future<Circle> task2;

        Circle blueCircle;
        Circle redCircle;

        try {
            //Reduce the image size by half to increase speed of the algorithm
            Imgproc.resize(raw, raw, new Size(320, 180));

            //Blur the raw image to reduce noise
            Imgproc.medianBlur(raw, raw, 3);

            //Split the input image into its 4 component channels, red, green, blue, and alpha
            Core.split(raw,rgbaChannels);
            redChannel = rgbaChannels.get(0);
            blueChannel = rgbaChannels.get(2);

            //Convert image from RGBA to HSV format
            //This is done to account for variations in lighting, which do not affect the HSV format as much as the RGBA format
            Imgproc.cvtColor(raw, hsv, Imgproc.COLOR_RGBA2RGB);
            Imgproc.cvtColor(hsv, hsv, Imgproc.COLOR_RGB2HSV);

            //Filters for only red pixels
            //There are two ranges for red because HSV is a cylindrical coordinate system and red circles back on itself
            Core.inRange(hsv, new Scalar(0, 100, 100), new Scalar(1, 255, 255), lowerRedRange);
            Core.inRange(hsv, new Scalar(140,100,100), new Scalar(179,255,255), upperRedRange);
/*
            ArrayList<Mat> channels = new ArrayList<>();
            Imgproc.cvtColor(raw, raw, Imgproc.COLOR_RGB2Lab);
            Imgproc.GaussianBlur(raw,raw,new Size(3,3),0);
            Core.split(raw, channels);
            Imgproc.threshold(channels.get(1), red, 164, 255, Imgproc.THRESH_BINARY);
*/
            //Filters for only blue pixels
            /*Core.inRange(hsv, new Scalar(85,255,32), new Scalar(135,255,255), blue);

            hsv.release(); //Empty the hsv Mat

            //Combine the two red ranges into one Mat, then empty the Mats we used
            Core.addWeighted(lowerRedRange, 1, upperRedRange, 1, 0, red);
            lowerRedRange.release();
            upperRedRange.release();

            //Create the two new processing threads
            BallDetector processRed = new BallDetector(red, redChannel, 0.67, prevRedCircle);
            BallDetector processBlue = new BallDetector(blue, blueChannel, 0.67, prevBlueCircle);

            //Register the threads with the executor service
            task1 = service.submit(processRed);
            task2 = service.submit(processBlue);

            //Get the results of the threads
            redCircle = task1.get();
            blueCircle = task2.get();

            //Empty all the mats that were used
            redChannel.release();
            blueChannel.release();
            red.release();
            blue.release();

            //Store the returned circles as backup data for the next frame
            prevBlueCircle = blueCircle;
            prevRedCircle = redCircle;

            //Draw the returned circles
            redCircle.draw(raw, new Scalar(255,0,0));
            blueCircle.draw(raw, new Scalar(0,0,255));

            //Determine the location of each ball
            /*
            direction = redCircle.center.x < blueCircle.center.x ? "left" : redCircle.center.x > blueCircle.center.x ? "right" : "???????? blame crowforce";
            telemetry.addData("red location", direction);
            */
            /*
            direction = blueCircle.center.x < redCircle.center.x ? "left" : blueCircle.center.x > redCircle.center.x ? "right" : "???????? blame crowforce";
            telemetry.addData("blue location", direction);
        }
        //If Opencv throws an exception, it does not print what it is, so you need a try-catch statement to recognize errors
        catch (Exception e) {
            telemetry.addData("error", e.getMessage());
            e.printStackTrace();
        }

        telemetry.update();

        //Resize the image so the camera can display it and return the result
        Mat returnImage = raw;
        Imgproc.resize(returnImage, returnImage, new Size(1280, 720));
        return returnImage;
    }
    */

    public void setupVuforia()
    {
        key=RelicRecoveryVuMark.UNKNOWN;
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
        telemetry.addData("key", key);
        telemetry.update();
    }

    public void startVuforia(){
        visionTargets.activate();
    }
    public void stopVuforia(){
        visionTargets.deactivate();
    }
    public void opencvStuff(Mat raw) {
        ArrayList<Mat> rgbaChannels = new ArrayList<>();

        Mat hsv = new Mat();
        Mat lowerRedRange = new Mat();
        Mat upperRedRange = new Mat();
        Mat red = new Mat();
        Mat blue = new Mat();

        Mat redChannel;
        Mat blueChannel;

        //Future<Circle> means that this task will someday return a circle, but not right now
        final Future<Circle> task1;
        final Future<Circle> task2;

        Circle blueCircle;
        Circle redCircle;

        try {
            //Reduce the image size by half to increase speed of the algorithm
            Imgproc.resize(raw, raw, new Size(320, 180));

            //Blur the raw image to reduce noise
            Imgproc.medianBlur(raw, raw, 3);

            //Split the input image into its 4 component channels, red, green, blue, and alpha
            Core.split(raw,rgbaChannels);
            redChannel = rgbaChannels.get(0);
            blueChannel = rgbaChannels.get(2);

            //Convert image from RGBA to HSV format
            //This is done to account for variations in lighting, which do not affect the HSV format as much as the RGBA format
            Imgproc.cvtColor(raw, hsv, Imgproc.COLOR_RGBA2RGB);
            Imgproc.cvtColor(hsv, hsv, Imgproc.COLOR_RGB2HSV);

            //Filters for only red pixels
            //There are two ranges for red because HSV is a cylindrical coordinate system and red circles back on itself
            Core.inRange(hsv, new Scalar(0, 100, 100), new Scalar(1, 255, 255), lowerRedRange);
            Core.inRange(hsv, new Scalar(140,100,100), new Scalar(179,255,255), upperRedRange);
/*
            ArrayList<Mat> channels = new ArrayList<>();
            Imgproc.cvtColor(raw, raw, Imgproc.COLOR_RGB2Lab);
            Imgproc.GaussianBlur(raw,raw,new Size(3,3),0);
            Core.split(raw, channels);
            Imgproc.threshold(channels.get(1), red, 164, 255, Imgproc.THRESH_BINARY);
*/
            //Filters for only blue pixels
            Core.inRange(hsv, new Scalar(85,255,32), new Scalar(135,255,255), blue);

            hsv.release(); //Empty the hsv Mat

            //Combine the two red ranges into one Mat, then empty the Mats we used
            Core.addWeighted(lowerRedRange, 1, upperRedRange, 1, 0, red);
            lowerRedRange.release();
            upperRedRange.release();

            //Create the two new processing threads
            BallDetector processRed = new BallDetector(red, redChannel, 0.67, prevRedCircle);
            BallDetector processBlue = new BallDetector(blue, blueChannel, 0.67, prevBlueCircle);

            //Register the threads with the executor service
            task1 = service.submit(processRed);
            task2 = service.submit(processBlue);

            //Get the results of the threads
            redCircle = task1.get();
            blueCircle = task2.get();

            //Empty all the mats that were used
            redChannel.release();
            blueChannel.release();
            red.release();
            blue.release();

            //Store the returned circles as backup data for the next frame
            prevBlueCircle = blueCircle;
            prevRedCircle = redCircle;

            //Draw the returned circles
            redCircle.draw(raw, new Scalar(255,0,0));
            blueCircle.draw(raw, new Scalar(0,0,255));

            //Determine the location of each ball

            direction = redCircle.center.x < blueCircle.center.x ? "left" : redCircle.center.x > blueCircle.center.x ? "right" : "???????? blame crowforce";
            telemetry.addData("red location", direction);


            direction = blueCircle.center.x < redCircle.center.x ? "left" : blueCircle.center.x > redCircle.center.x ? "right" : "???????? blame crowforce";
            telemetry.addData("blue location", direction);
        }
        //If Opencv throws an exception, it does not print what it is, so you need a try-catch statement to recognize errors
        catch (Exception e) {
            telemetry.addData("error", e.getMessage());
            e.printStackTrace();
        }
    }
    public void startOpenCV(CameraBridgeViewBase.CvCameraViewListener2 cameraViewListener) {
        FtcRobotControllerActivity.turnOnCameraView.obtainMessage(1,cameraViewListener).sendToTarget();
    }

    public void stopOpenCV() {
        try {
            FtcRobotControllerActivity.turnOffCameraView.obtainMessage().sendToTarget();
        }
        catch(Exception e) {
            telemetry.addData("error:", e.getMessage());
            telemetry.addData("cause:", e.getCause());
            telemetry.addData("stack trace:",e.getStackTrace());
            telemetry.update();
        }
    }
}