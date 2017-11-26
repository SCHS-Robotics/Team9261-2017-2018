package org.firstinspires.ftc.teamcode;

import android.graphics.MaskFilter;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.opencv.android.CameraBridgeViewBase;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfInt;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

import static org.opencv.imgcodecs.Imgcodecs.CV_LOAD_IMAGE_COLOR;
import static org.opencv.imgcodecs.Imgcodecs.imread;

/**
 * Created by Sage Creek Level Up on 11/21/2017.
 */
@Autonomous(name = "OpenCV Practice", group = "autonomous")
public class OpenCVPractice extends LinearOpMode implements CameraBridgeViewBase.CvCameraViewListener2 {
    @Override
    public void runOpMode() {

        waitForStart();

        startOpenCV(this);

        while (opModeIsActive()) {
            idle();
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
        Mat raw = inputFrame.rgba();
        int height = raw.height();
        int width = raw.width();
        Mat grey = inputFrame.gray();
        Mat filtered = new Mat(height, width, CvType.CV_8UC1);
        Mat edges = new Mat(height, width, CvType.CV_8UC1);
        Mat cubeLayer = new Mat(height, width, raw.type());
        Mat cubeMask = new Mat(height, width, CvType.CV_8UC1, new Scalar(0,0,0));
        List<MatOfPoint> contours = new ArrayList<MatOfPoint>();
        try {
            Imgproc.GaussianBlur(grey, grey, new Size(3, 3), 1);
            Imgproc.bilateralFilter(grey, filtered, 11, 17, 17);
            Imgproc.Canny(filtered, edges, 20, 50);
            Mat structure = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(45, 45));
            Imgproc.morphologyEx(edges, edges, Imgproc.MORPH_CLOSE, structure);
            Imgproc.findContours(edges.clone(), contours, new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
            //SORT THE CONTOURS!!! IF YOU DON'T IT WON'T WORK!!!
            List<MatOfPoint> hullPoints = new ArrayList<>();
            MatOfInt hull = new MatOfInt();
            MatOfPoint2f approx = new MatOfPoint2f();
            List<MatOfPoint> approxlist = new ArrayList<>();
            //begin heavy modification (if there's a problem, it's probably here)
            for (MatOfPoint c : contours) {
                double peri = Imgproc.arcLength(new MatOfPoint2f(c.toArray()), true);
                Imgproc.approxPolyDP(new MatOfPoint2f(c.toArray()), approx, 0.02 * peri, true);
                approxlist.clear();
                approxlist.add(new MatOfPoint(approx.toArray()));
                //get all convex hull points
                Imgproc.convexHull(c, hull);
                for (int i = 0; i < hull.size().height; i++) {
                    int index = (int) hull.get(i, 0)[0];
                    double[] point = new double[]{
                            c.get(index, 0)[0], c.get(index, 0)[1]
                    };
                    hullPoints.add(i, new MatOfPoint(new Point(point)));
                }
                Imgproc.polylines(raw, hullPoints, true, new Scalar(0, 255, 255), 4);
                if (approx.toList().size() <= 5) {
                    Imgproc.drawContours(raw, approxlist, -1, new Scalar(255, 255, 255),1);
                }
            }
            raw.copyTo(cubeLayer, cubeMask); //masks raw image
            List<Double> areas = new ArrayList<>();
            for(MatOfPoint c : contours) {
                areas.add(Imgproc.contourArea(c));
            }
            int maxIndex = 0;
            for (int i = 1; i < areas.toArray().length; i++) {
                double newnumber = 0;
                newnumber = areas.get(i);
                if ((newnumber > areas.get(maxIndex))) {
                    maxIndex = i;
                }
            }
            MatOfPoint chosen = contours.get(maxIndex);
            Rect box = Imgproc.boundingRect(chosen);
            int result = (box.x + box.width/2);

            Imgproc.rectangle(raw,new Point(box.x,box.y), new Point(box.x+box.width,box.y+box.height), new Scalar(0,255,0),2);
            Imgproc.putText(raw, "Chosen" , new Point(box.x + 10,box.y + box.height/2 - 40), Core.FONT_HERSHEY_COMPLEX, 0.8, new Scalar(0,255,0), 2);
            Imgproc.putText(raw, "RESULT: " + Integer.toString(result), new Point(result,30),0,1,new Scalar(0,255,0), 1);

        }
        catch (Exception e) {
            telemetry.addData("error",e.toString());
        }
        // This is where the magic will happen. inputFrame has all the data for each camera frame.
        telemetry.update();
        return raw;
    }
    public void startOpenCV(CameraBridgeViewBase.CvCameraViewListener2 cameraViewListener) {
        if (FtcRobotControllerActivity.mOpenCvCameraView.isEnabled())
            FtcRobotControllerActivity.mOpenCvCameraView.disableView();
        FtcRobotControllerActivity.turnOnCameraView.obtainMessage().sendToTarget();
        FtcRobotControllerActivity.mOpenCvCameraView.setCvCameraViewListener(cameraViewListener);
        FtcRobotControllerActivity.mOpenCvCameraView.enableView();
    }

    public void stopOpenCV() {
        try {
            FtcRobotControllerActivity.turnOffCameraView.obtainMessage().sendToTarget();
            FtcRobotControllerActivity.mOpenCvCameraView.disableView();

        }
        catch(Exception e) {
            telemetry.addData("error:", e.toString());
            telemetry.addData("cause:", e.getCause());
            telemetry.addData("stack trace:",e.getStackTrace());
            telemetry.update();
            sleep(3000);
        }
    }
}
