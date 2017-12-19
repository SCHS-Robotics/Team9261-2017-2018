package org.firstinspires.ftc.teamcode;

import android.provider.ContactsContract;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.opencv.android.CameraBridgeViewBase;
import org.opencv.android.Utils;
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

import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

import static org.opencv.imgcodecs.Imgcodecs.CV_LOAD_IMAGE_COLOR;
import static org.opencv.imgcodecs.Imgcodecs.imread;

/**
 * Created by Sage Creek Level Up on 11/21/2017.
 */
@Autonomous(name = "Glyph Detector", group = "autonomous")
public class GlyphDetector extends LinearOpMode implements CameraBridgeViewBase.CvCameraViewListener2 {
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

        Size imageSize = new Size(1280,720); //Resize Images to this

        Boolean debug_show_preprocessed = false; //Show the preproccessed image
        Boolean debug_show_filtered = false; //Show the filtered image
        Boolean debug_draw_stats = false; //Show stats for each rectangle (very spammy)
        Boolean debug_draw_center = false; //Draw center line on the screen
        Boolean debug_draw_rects = false; //Draw all found rectangles

        //Weights for scoring
        double score_ratio_weight = 0.9;
        double score_distance_x_weight = 1;
        double score_distance_y_weight = 1.2;
        double score_area_weight = 3;

        try {

        }
        catch (Exception e) {
            telemetry.addData("error",e.getMessage());
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
        }
    }
    //Process and find glyphs
    public void process_image(Mat input_mat,Size size) {
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Mat output = input_mat;
        Imgproc.resize(output,output,size);
        Mat processed = preprocess(input_mat,size);
        Mat filtered = apply_filters(processed);
        processed.release();
        Imgproc.findContours(filtered.clone(),contours,hierarchy,Imgproc.RETR_TREE,Imgproc.CHAIN_APPROX_SIMPLE);
        hierarchy.release();
        filtered.release();

        //magic_sort(contours)//sort by x coord
    }
    public Mat preprocess(Mat input_mat, Size size) {
        Mat resized = new Mat();
        Mat grey = new Mat();
        Imgproc.resize(input_mat,resized,size);
        Imgproc.cvtColor(resized,grey,Imgproc.COLOR_BGRA2GRAY);
        //grey = resized
        return grey;
    }
    public Mat apply_filters(Mat input_mat) {
        Mat blurred = new Mat();
        Mat edges = new Mat();
        Mat structure = new Mat();
        Imgproc.GaussianBlur(input_mat,blurred,new Size(3,3),1); //Jack-ify the image
        Imgproc.bilateralFilter(blurred,blurred,11,17,17);
        Imgproc.Canny(blurred,edges,20,50);
        structure = Imgproc.getStructuringElement(Imgproc.MORPH_RECT,new Size(45,45));
        Imgproc.morphologyEx(edges,edges,Imgproc.MORPH_CLOSE,structure);
        return edges;
    }

}