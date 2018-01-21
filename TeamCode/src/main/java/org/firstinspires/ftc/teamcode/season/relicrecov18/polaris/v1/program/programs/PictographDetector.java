package org.firstinspires.ftc.teamcode.season.relicrecov18.polaris.v1.program.programs;

import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.net.Uri;
import android.os.Environment;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.firstinspires.ftc.teamcode.R;
import org.opencv.android.CameraBridgeViewBase;
import org.opencv.android.Utils;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.DMatch;
import org.opencv.core.KeyPoint;
import org.opencv.core.Mat;
import org.opencv.core.MatOfByte;
import org.opencv.core.MatOfDMatch;
import org.opencv.core.MatOfKeyPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.MatOfRect;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.features2d.AKAZE;
import org.opencv.features2d.AgastFeatureDetector;
import org.opencv.features2d.BFMatcher;
import org.opencv.features2d.BRISK;
import org.opencv.features2d.FastFeatureDetector;
import org.opencv.features2d.Features2d;
import org.opencv.features2d.ORB;
import org.opencv.imgproc.Imgproc;
import org.opencv.objdetect.CascadeClassifier;

import java.io.File;
import java.io.IOException;
import java.io.InputStream;
import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;

import static org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity.pictographTemplate;

/**
 * Created by Sage Creek Level Up on 11/21/2017.
 */
@Autonomous(name = "Pictographer2.0", group = "Fails")
public class PictographDetector extends LinearOpMode implements CameraBridgeViewBase.CvCameraViewListener2 {

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
        Mat img_scene = inputFrame.gray();
        Mat img_matches = new Mat();
        Mat cropped = new Mat();
        try {

            Mat img_object = FtcRobotControllerActivity.pictographTemplate;

            ORB detector = ORB.create();

            MatOfKeyPoint keypoints_object = new MatOfKeyPoint();
            MatOfKeyPoint keypoints_scene  = new MatOfKeyPoint();

            detector.detect(img_object, keypoints_object);
            detector.detect(img_scene, keypoints_scene);

            ORB extractor = ORB.create(); //2 = SURF;

            System.out.println("ran");

            Mat descriptor_object = new Mat();
            Mat descriptor_scene = new Mat() ;

            extractor.compute(img_object, keypoints_object, descriptor_object);
            extractor.compute(img_scene, keypoints_scene, descriptor_scene);

            BFMatcher matcher = BFMatcher.create(BFMatcher.BRUTEFORCE_HAMMING,false); // 1 = FLANNBASED
            MatOfDMatch matches = new MatOfDMatch();

            matcher.match(descriptor_object, descriptor_scene, matches);
            List<DMatch> matchesList = matches.toList();

            //Features2d.drawMatches(img_object,keypoints_object,img_scene,keypoints_scene,matches,t);

            double max_dist = 0.0;
            double min_dist = Integer.MAX_VALUE;

            //int i = 0; i < descriptor_object.rows(); i++

            for(Object d:matchesList.toArray()){
                DMatch m = (DMatch) d;
                double dist = m.distance;
                if(dist < min_dist) min_dist = dist;
                if(dist > max_dist) max_dist = dist;
            }

            LinkedList<DMatch> good_matches = new LinkedList<>();
            MatOfDMatch gm = new MatOfDMatch();

            for(int i = 0; i < descriptor_object.rows(); i++){
                if(matchesList.get(i).distance < 3*min_dist){
                    good_matches.addLast(matchesList.get(i));
                }
            }

            gm.fromList(good_matches);

            Features2d.drawMatches(
                    img_object,
                    keypoints_object,
                    img_scene,
                    keypoints_scene,
                    gm,
                    img_matches,
                    new Scalar(255,0,0),
                    new Scalar(0,0,255),
                    new MatOfByte(),
                    2);

            //showResult(img_matches);

            LinkedList<Point> objList = new LinkedList<>();
            LinkedList<Point> sceneList = new LinkedList<>();

            List<KeyPoint> keypoints_objectList = keypoints_object.toList();
            List<KeyPoint> keypoints_sceneList = keypoints_scene.toList();

            for(int i = 0; i<good_matches.size(); i++){
                objList.addLast(keypoints_objectList.get(good_matches.get(i).queryIdx).pt);
                sceneList.addLast(keypoints_sceneList.get(good_matches.get(i).trainIdx).pt); }

            MatOfPoint2f obj = new MatOfPoint2f();
            obj.fromList(objList);

            MatOfPoint2f scene = new MatOfPoint2f();
            scene.fromList(sceneList);

            Mat H = Calib3d.findHomography(obj, scene, Calib3d.RANSAC,10);

            Mat obj_corners = new Mat(4,1,CvType.CV_32FC2);
            Mat scene_corners = new Mat(4,1, CvType.CV_32FC2);

            obj_corners.put(0, 0, new double[] {0,0});
            obj_corners.put(1, 0, new double[] {img_object.cols(),0});
            obj_corners.put(2, 0, new double[] {img_object.cols(),img_object.rows()});
            obj_corners.put(3, 0, new double[] {0,img_object.rows()});

            if(!H.empty()) {

                Core.perspectiveTransform(obj_corners, scene_corners, H);
                Mat perspectiveMatrix = Imgproc.getPerspectiveTransform(scene_corners, obj_corners);

                Imgproc.warpPerspective(img_scene, cropped, perspectiveMatrix, img_scene.size());

                Imgproc.line(img_scene, new Point(scene_corners.get(0, 0)), new Point(scene_corners.get(1, 0)), new Scalar(0, 0, 0), 4);
                Imgproc.line(img_scene, new Point(scene_corners.get(1, 0)), new Point(scene_corners.get(2, 0)), new Scalar(0, 0, 0), 4);
                Imgproc.line(img_scene, new Point(scene_corners.get(2, 0)), new Point(scene_corners.get(3, 0)), new Scalar(0, 0, 0), 4);
                Imgproc.line(img_scene, new Point(scene_corners.get(3, 0)), new Point(scene_corners.get(0, 0)), new Scalar(0, 0, 0), 4);
            }

        }
        //If Opencv throws an exception, it does not print what it is, so you need a try-catch statement to recognize errors
        catch (Exception e) {
            telemetry.addData("error", e.getMessage());
            e.printStackTrace();
        }

        telemetry.update();

        //Resize the image so the camera can display it and return the result
        telemetry.addLine("running");
        Mat returnImage = img_scene;
        Imgproc.resize(returnImage, returnImage,new Size(1280,720));

        return cropped;
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