package org.firstinspires.ftc.teamcode.season.relicrecov18.polaris.v1.program.programs;

import android.support.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.opencv.android.CameraBridgeViewBase;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.MatOfRect;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.objdetect.CascadeClassifier;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

/**
 * Created by Sage Creek Level Up on 11/21/2017.
 */
@Autonomous(name = "MSPaint", group = "meme")
public class PictographDetectorV2 extends LinearOpMode implements CameraBridgeViewBase.CvCameraViewListener2 {

    Mat templateImage;
    CascadeClassifier hex;

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
        templateImage = FtcRobotControllerActivity.pictographTemplate;
        hex = FtcRobotControllerActivity.hex_cascade;
    }

    @Override
    public void onCameraViewStopped() {

    }

    @Override
    public Mat onCameraFrame(CameraBridgeViewBase.CvCameraViewFrame inputFrame) {
        Mat raw = inputFrame.gray();

        ArrayList<Mat> images = new ArrayList<>();

        int i = 0;

        Mat blurred = new Mat();
        Mat warped = new Mat();
        Mat output = new Mat();

        Mat cropped = new Mat();

        Mat structure = Imgproc.getStructuringElement(Imgproc.MORPH_ELLIPSE,new Size(5,5));

        List<MatOfPoint> contours = new ArrayList<>();

        try {
            Imgproc.resize(raw,raw,new Size(960,540),0,0,Imgproc.INTER_AREA);
            Imgproc.GaussianBlur(raw,blurred,new Size(7,7),0);

            Imgproc.cvtColor(raw,output,Imgproc.COLOR_GRAY2RGB);

            Mat edges = auto_canny(blurred);
            Imgproc.morphologyEx(edges,edges,Imgproc.MORPH_CLOSE,structure);

            Imgproc.findContours(edges,contours,new Mat(),Imgproc.RETR_TREE,Imgproc.CHAIN_APPROX_SIMPLE);

            ArrayList<MatOfPoint> goodContours = new ArrayList<>();

            for(MatOfPoint c : contours) {
                MatOfPoint2f approx = new MatOfPoint2f();
                Imgproc.approxPolyDP(new MatOfPoint2f(c.toArray()), approx, 0.01 * Imgproc.arcLength(new MatOfPoint2f(c.toArray()), true), true);
                if (approx.toList().size() == 8) {
                    double area = Imgproc.contourArea(new MatOfPoint(approx.toArray()));
                    if (area > 10000) {
                        goodContours.add(new MatOfPoint(approx.toArray()));
                    }
                }
            }
            Collections.sort(goodContours, new Comparator<MatOfPoint>() {
                @Override
                public int compare(MatOfPoint lhs, MatOfPoint rhs) {
                    if(Imgproc.contourArea(lhs) > Imgproc.contourArea(rhs)) {
                        return 1;
                    }
                    else if(Imgproc.contourArea(lhs) < Imgproc.contourArea(rhs)) {
                        return -1;
                    }
                    else {
                        return 0;
                    }
                }
            });

            warped = new Mat();

            List<MatOfPoint> approxList = new ArrayList<>();
            approxList.add(new MatOfPoint(goodContours.get(0)));

            Imgproc.drawContours(output, approxList, 0, new Scalar(255, 0, 0), 5);

            Mat corners = getCorners(approxList.get(0), output);
            Mat templateCorners = getTemplateCorners(templateImage);

            Size boxSize = getCornersSize(templateCorners);

            Mat perspectiveMatrix = Imgproc.getPerspectiveTransform(corners,templateCorners);
            Imgproc.warpPerspective(raw,warped,perspectiveMatrix,boxSize);

            cropped = warped.submat(new Rect(0,0,(int) Math.round(boxSize.width/2),(int) Math.round(boxSize.height)));
            //cropped = new Mat(warped,new Rect(0,0,(int) Math.round(boxSize.width/2),(int) Math.round(boxSize.height)));

            telemetry.addData("hexes",hexCount(cropped,cropped));
            //telemetry.addData("is Empty?", images.size() == 0);
            telemetry.addData("how many times I ran", i);
            telemetry.update();
        }
        //If Opencv throws an exception, it does not print what it is, so you need a try-catch statement to recognize errors
        catch (Exception e) {
            telemetry.addData("error", e.getMessage());
            e.printStackTrace();
        }

        telemetry.update();

        //Resize the image so the camera can display it and return the result

        //Mat returnImage = images.isEmpty() ? output : images.get(0).empty() ? output : images.get(0);
        Mat returnImage = warped.empty() ? output : cropped;
        //Mat returnImage = output;
        //Mat returnImage = output;
        Imgproc.resize(returnImage, returnImage, new Size(1280, 720));
        return returnImage;
    }

    public Mat auto_canny(Mat image) {
        MatOfDouble mean = new MatOfDouble();
        MatOfDouble std = new MatOfDouble();
        Mat edges = new Mat();
        Core.meanStdDev(image, mean, std);
        //Imgproc.adaptiveThreshold(image,image,255,Imgproc.ADAPTIVE_THRESH_MEAN_C,Imgproc.THRESH_BINARY,51,0);
        Imgproc.Canny(image, edges, mean.get(0,0)[0] - std.get(0,0)[0], mean.get(0,0)[0] + std.get(0,0)[0]);
        return edges;
    }

    public Mat getTemplateCorners(Mat template) {
        Mat output = new Mat(4,1,CvType.CV_32FC2);
        output.put(0,0, new double[] {0.0,0.0});
        output.put(1,0, new double[] {(float) template.cols(),0.0});
        output.put(2,0, new double[] {(float) template.cols(),(float) template.rows()});
        output.put(3,0, new double[] {0.0,(float) template.rows()});
        return output;
    }

    public Size getCornersSize(Mat corners) {
        Point topRight = new Point(corners.get(1,0));
        Point bottomRight = new Point(corners.get(2,0));
        Point bottomLeft = new Point(corners.get(3,0));

        int width = (int) (bottomRight.x - bottomLeft.x);
        int height = (int) (bottomRight.y - topRight.y);

        return new Size(width,height);
    }

    public Mat getCorners(MatOfPoint pts, Mat draw) {
        ArrayList<Point> ptsList = new ArrayList<>(pts.toList());
        ArrayList<Line> lines = new ArrayList<>();
        ArrayList<Point> importantPts = new ArrayList<>();
        for(int i = 0; i < ptsList.size(); i++) {
            Point init = ptsList.get(i);
            Point lookAhead = ptsList.get((i+1)%ptsList.size());

            Line temp = new Line(init,lookAhead);
            lines.add(temp);
        }
        Collections.sort(lines, new Comparator<Line>() {
            @Override
            public int compare(Line lhs, Line rhs) {
                if(lhs.length > rhs.length) {
                    return -1;
                }
                else if(lhs.length < rhs.length) {
                    return 1;
                }
                else {
                    return 0;
                }
            }
        });

        List<Line> importantLines = new ArrayList<>();

        for(Line l : lines.subList(0,4)) {
            if(!importantPts.contains(l.start)) {
                importantPts.add(l.start);
                Imgproc.circle(draw,l.start,7,new Scalar(0,0,0),-1);
            }
            if(!importantPts.contains(l.end)) {
                importantPts.add(l.end);
                Imgproc.circle(draw,l.end,7,new Scalar(0,0,0),-1);
            }
        }
        importantLines = lines.subList(0,4);

        Collections.sort(importantLines, new Comparator<Line>() {
            @Override
            public int compare(Line lhs, Line rhs) {
                return (int) Math.round(Math.abs(lhs.m) - Math.abs(rhs.m));
            }
        });

        List<Line> topBottom = new ArrayList<>();
        List<Line> leftRight = new ArrayList<>();

        telemetry.addData("1",importantLines.get(0).m);
        telemetry.addData("2",importantLines.get(1).m);
        telemetry.addData("3",importantLines.get(2).m);
        telemetry.addData("4",importantLines.get(3).m);

        topBottom.add(importantLines.get(0));
        topBottom.add(importantLines.get(1));

        leftRight.add(importantLines.get(2));
        leftRight.add(importantLines.get(3));

        Collections.sort(leftRight, new Comparator<Line>() {
            @Override
            public int compare(Line lhs, Line rhs) {
                return (int) Math.round(rhs.start.x-lhs.start.x);
            }
        });

        Line left = leftRight.get(1);
        Line right = leftRight.get(0);

        Collections.sort(topBottom, new Comparator<Line>() {
            @Override
            public int compare(Line lhs, Line rhs) {
                return (int) Math.round(rhs.start.y-lhs.start.y);
            }
        });

        Line top = topBottom.get(1);
        Line bottom = topBottom.get(0);

        bottom.drawLabeled(draw, "bottom");
        top.drawLabeled(draw,"top");
        left.drawLabeled(draw, "left");
        right.drawLabeled(draw, "right");

        /*
        Collections.sort(importantPts, new Comparator<Point>() {
            @Override
            public int compare(Point o1, Point o2) {
                if(o1.y > o2.y){
                    return 1;
                }
                else if(o1.y < o2.y) {
                    return -1;
                }
                else {
                    return 0;
                }
            }
        });
        Line top = new Line(importantPts.get(0),importantPts.get(1));
        Line bottom = new Line(importantPts.get(importantPts.size()-2),importantPts.get(importantPts.size()-1));
        Collections.sort(importantPts, new Comparator<Point>() {
            @Override
            public int compare(Point o1, Point o2) {
                if(o1.x > o2.x){
                    return 1;
                }
                else if(o1.x < o2.x) {
                    return -1;
                }
                else {
                    return 0;
                }
            }
        });
        Line left = new Line(importantPts.get(0),importantPts.get(1));
        Line right = new Line(importantPts.get(importantPts.size()-2),importantPts.get(importantPts.size()-1));
*/
        Point topLeft = top.getIntersectWith(left);
        Point topRight = top.getIntersectWith(right);
        Point bottomLeft = bottom.getIntersectWith(left);
        Point bottomRight = bottom.getIntersectWith(right);

        Mat output = new Mat(4,1, CvType.CV_32FC2);

        output.put(0,0, new double[] {(float) topLeft.x,(float) topLeft.y});
        output.put(1,0, new double[] {(float) topRight.x,(float) topRight.y});
        output.put(2,0, new double[] {(float) bottomRight.x,(float) bottomRight.y});
        output.put(3,0, new double[] {(float) bottomLeft.x,(float) bottomLeft.y});
/*
        Imgproc.line(draw,topLeft,topRight,new Scalar(0,255,0),5);
        Imgproc.line(draw,topLeft,bottomLeft,new Scalar(0,255,0),5);
        Imgproc.line(draw,bottomLeft,bottomRight,new Scalar(0,255,0),5);
        Imgproc.line(draw,bottomRight,topRight,new Scalar(0,255,0),5);
*/
        return output;
    }

    public Mat hexFilter(Mat input) {
        Mat thresh = new Mat();
        Mat hsv = new Mat();
        Mat rgb = new Mat();
        Imgproc.cvtColor(input,rgb,Imgproc.COLOR_GRAY2RGB);
        Imgproc.cvtColor(rgb,hsv,Imgproc.COLOR_RGB2HSV);
        Core.inRange(hsv,new Scalar(0,0,0),new Scalar(255,255,40),thresh);
        return thresh;
    }

    public int hexCount(Mat input, Mat draw) {
        MatOfRect boxes = new MatOfRect();
        int hexes = 0;
        hex.detectMultiScale(input,boxes,1.1,5,0,new Size(),new Size());
        for(Rect box: boxes.toArray()) {
            Imgproc.rectangle(draw,new Point(box.x,box.y), new Point(box.x+box.width,box.y+box.height),new Scalar(0,0,0),5);
            hexes++; //Adds more curses to enemy teams
        }
        /*
        ArrayList<MatOfPoint> contours = new ArrayList<>();
        ArrayList<MatOfPoint> approxList = new ArrayList<>();
        Mat filtered = hexFilter(input);
        Mat edges = auto_canny(filtered);
        Imgproc.findContours(edges,contours,new Mat(),Imgproc.RETR_EXTERNAL,Imgproc.CHAIN_APPROX_SIMPLE);
        for(MatOfPoint c : contours) {
            approxList = new ArrayList<>();
            MatOfPoint2f approx = new MatOfPoint2f();
            double peri = Imgproc.arcLength(new MatOfPoint2f(c.toArray()),true);
            Imgproc.approxPolyDP(new MatOfPoint2f(c.toArray()),approx,0.01*peri,true);
            if(approx.toList().size() == 6) {
                approxList.add(new MatOfPoint(approx.toArray()));
                Imgproc.drawContours(draw,approxList,0,new Scalar(255,0,0),5);
            }
        }*/
        return hexes;
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
