package org.firstinspires.ftc.teamcode.season.relicrecov18.polaris.v1.program.programs;

import android.net.Uri;
import android.os.Environment;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.firstinspires.ftc.teamcode.R;
import org.opencv.android.CameraBridgeViewBase;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfRect;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.objdetect.CascadeClassifier;

import java.io.File;
import java.io.InputStream;
import java.util.ArrayList;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;

/**
 * Created by Sage Creek Level Up on 11/21/2017.
 */
@Autonomous(name = "Pictographer", group = "Fails")
public class PictographClassiferTest extends LinearOpMode implements CameraBridgeViewBase.CvCameraViewListener2 {
    CascadeClassifier pictographDetector = FtcRobotControllerActivity.face_cascade;
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
    public Mat onCameraFrame(CameraBridgeViewBase.CvCameraViewFrame inputFrame) {
        Mat raw = inputFrame.gray();
        MatOfRect boundingBoxes = new MatOfRect();

        try {
            //Imgproc.threshold(raw,raw,45,45,Imgproc.THRESH_BINARY);
            pictographDetector.detectMultiScale(raw,boundingBoxes);
            int i = 0;
            for(Rect rect:boundingBoxes.toList()) {
                Imgproc.rectangle(raw,new Point(rect.x,rect.y),new Point(rect.x+rect.width,rect.y+rect.height),new Scalar(0,255,0),10);
                i++;
            }
            telemetry.addData("number of detections", i);
        }
        //If Opencv throws an exception, it does not print what it is, so you need a try-catch statement to recognize errors
        catch (Exception e) {
            telemetry.addData("error", e.getMessage());
            e.printStackTrace();
        }

        telemetry.update();

        //Resize the image so the camera can display it and return the result

        return raw;
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