package org.firstinspires.ftc.teamcode.season.relicrecov18.polaris.v1.program;

import android.support.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.opencv.android.CameraBridgeViewBase;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.CLAHE;
import org.opencv.imgproc.Imgproc;
import org.opencv.photo.Photo;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;

/**
 * Created by Sage Creek Level Up on 11/21/2017.
 */
@Autonomous(name = "Red Calibration", group = "meme")
public class RedCalibration extends LinearOpMode implements CameraBridgeViewBase.CvCameraViewListener2 {
    int i = 120;
    @Override
    public void runOpMode() {

        waitForStart();

        startOpenCV(this);

        while (opModeIsActive()) {

        }
        stopOpenCV();
    }

    @Override
    public void onCameraViewStarted(int width, int height) {

    }

    @Override
    public void onCameraViewStopped() {

    }

    @Override
    public Mat onCameraFrame(CameraBridgeViewBase.CvCameraViewFrame rawFrame) {
        Mat raw = rawFrame.rgba();
        Mat hsv = new Mat();
        Mat lowerRedRange = new Mat();
        Mat upperRedRange = new Mat();
        Mat blue = new Mat();
        Mat g = new Mat();
        Mat red2 = new Mat();
        List<Mat> channels = new ArrayList<>();
        try {
            Imgproc.resize(raw,raw,new Size(320,180));
            Mat b= new Mat();
            Imgproc.cvtColor(raw, b, Imgproc.COLOR_RGB2YUV);
            Imgproc.GaussianBlur(b,b,new Size(3,3),0);
            Core.split(b, channels);
            Imgproc.threshold(channels.get(1), blue, 145, 255, Imgproc.THRESH_BINARY);
            telemetry.addData("t1",blue.type());
            channels.clear();
            Core.split(raw,channels);
            Imgproc.threshold(channels.get(2), channels.get(2),140,255,Imgproc.THRESH_BINARY);
            Imgproc.cvtColor(raw, hsv, Imgproc.COLOR_RGBA2RGB);
            Imgproc.cvtColor(hsv, hsv, Imgproc.COLOR_RGB2HSV);

            telemetry.addData("i",i);

            Core.inRange(hsv, new Scalar(40,100,55), new Scalar(120,255,255), red2);

            telemetry.addData("t2",red2.type());
            Core.bitwise_and(blue,red2,blue);
            Core.bitwise_and(blue,channels.get(2),blue);

            i++;
        }
        catch (Exception e) {
            e.printStackTrace();
            telemetry.addData("error",e.getMessage());
        }
        telemetry.update();

        //Resize the image so the camera can display it and return the result
        Mat returnImage = blue;
        Imgproc.resize(returnImage, returnImage, new Size(1280, 720));
        return returnImage;
    }

    //Starts Opencv by sending a start message to FtcRobotControllerActivity
    public void startOpenCV(CameraBridgeViewBase.CvCameraViewListener2 cameraViewListener) {
        FtcRobotControllerActivity.turnOnCameraView.obtainMessage(1, cameraViewListener).sendToTarget();
    }

    //Stops Opencv by sending a start message to FtcRobotControllerActivity
    public void stopOpenCV() {
        FtcRobotControllerActivity.turnOffCameraView.obtainMessage().sendToTarget();
    }
}