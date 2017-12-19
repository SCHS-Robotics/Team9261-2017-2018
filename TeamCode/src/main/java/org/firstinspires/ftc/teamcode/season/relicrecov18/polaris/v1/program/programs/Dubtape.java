package org.firstinspires.ftc.teamcode.season.relicrecov18.polaris.v1.program.programs;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.opencv.android.CameraBridgeViewBase;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;

import static org.opencv.imgproc.Imgproc.contourArea;
import static org.opencv.imgproc.Imgproc.getStructuringElement;

/**
 * Created by Sage Creek Level Up on 11/21/2017.
 */
@Autonomous(name = "Dubtape", group = "meme")
public class Dubtape extends LinearOpMode implements CameraBridgeViewBase.CvCameraViewListener2 {
    private String direction;
    private StorageContainer circleBox;

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
        circleBox = new StorageContainer();
        circleBox = new StorageContainer();

        Mat raw = inputFrame.rgba();
        Mat hsv = new Mat();
        Mat lowerRedRange = new Mat();
        Mat upperRedRange = new Mat();
        Mat red = new Mat();
        Mat blue = new Mat();
        Mat redEdges = new Mat();
        Mat blueEdges = new Mat();

        Mat structure = getStructuringElement(Imgproc.MORPH_ELLIPSE, new Size(15, 15));

        MatOfPoint biggestRedContour = new MatOfPoint();
        MatOfPoint biggestBlueContour = new MatOfPoint();

        Point redCenter = new Point();
        Point blueCenter = new Point();
        Point prevCenter;

        ArrayList<MatOfPoint> contours = new ArrayList<>();
        ArrayList<MatOfPoint> circularContours = new ArrayList<>();

        double avg;

        double redContourArea;
        double blueContourArea;

        float[] redRadius = new float[1];
        float[] blueRadius = new float[1];

        double redBiggestContourArea = 0;
        double blueBiggestContourArea = 0;

        int prevRadius;

        try {

            //These are the initial values that opencv will use the first frame if it can't find the balls
            circleBox.add(0.0,1);
            circleBox.add(new Point(0,0),2);
            circleBox.add(0.0,3);
            circleBox.add(new Point(0,0),4);

            //Here we resize the image by a factor of 1/2 to increase the FPS of the algorithm
            Imgproc.resize(raw, raw, new Size(640, 360));

            //Blurs the image to reduce noise, but also preserves the image's edges
            Imgproc.medianBlur(raw, raw, 3);

            //Converts the input image to hsv format
            Imgproc.cvtColor(raw, hsv, Imgproc.COLOR_RGBA2RGB);
            Imgproc.cvtColor(hsv, hsv, Imgproc.COLOR_RGB2HSV);

            //Filters for only red pixels. There are two values for red because hsv is a cylindrical coordinate system, and red wraps back around
            Core.inRange(hsv, new Scalar(0, 100, 100), new Scalar(1, 255, 255), lowerRedRange);
            Core.inRange(hsv, new Scalar(350, 100, 100), new Scalar(360, 255, 255), upperRedRange);

            //Filters for only blue pixels
            //Core.inRange(hsv, new Scalar(80,170,100), new Scalar(135,255,255), blue);
            Core.inRange(hsv, new Scalar(70, 45, 47), new Scalar(115, 255, 130), blue);
            //Core.inRange(hsv, new Scalar(80,150,0), new Scalar(120,255,255),blue);

            //Deallocates memory that was being used to store the contents of the hsv mat
            hsv.release();

            //Combines the two ranges of red and deallocates the mats of the component ranges
            Core.addWeighted(lowerRedRange, 1, upperRedRange, 1, 0, red);
            lowerRedRange.release();
            upperRedRange.release();

            //Blurs the combined images again to further reduce noise (we might not need this step)
            Imgproc.GaussianBlur(red, red, new Size(9, 9), 2, 2);
            Imgproc.GaussianBlur(blue, blue, new Size(9, 9), 2, 2);

            //Imgproc.medianBlur(red, red, 21);

            //Start of the red ball code

            //Find the edges of all the red shapes in the frame
            Imgproc.Canny(red, redEdges, 30, 60);

            //Deallocate the red mat
            red.release();

            //Reduce the image size in half to increase operation speed, and then close any holes in the image and return it to its original size
            Imgproc.resize(redEdges, redEdges, new Size(160, 90));
            Imgproc.morphologyEx(redEdges, redEdges, Imgproc.MORPH_CLOSE, structure);
            Imgproc.resize(redEdges, redEdges, new Size(640, 360));

            //Get a list of all the contours of the red edges
            Imgproc.findContours(redEdges,contours,new Mat(),Imgproc.RETR_TREE,Imgproc.CHAIN_APPROX_SIMPLE);

            if(contours.size() > 0) {
                //Fill every contour with white. I'm not sure if we need this or not, but it makes it much easier to see
                for (MatOfPoint c : contours) {
                    Imgproc.fillConvexPoly(redEdges, c, new Scalar(255, 255, 255));
                }

                //Find the contours of the filled shapes
                Imgproc.findContours(redEdges, contours, new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

                //For every contour in the filled shapes, calculate that contour's percent circularity based on its theoretical and predicted area. If it has a circularity of >= 67%, set it aside for later analysis
                for (MatOfPoint c : contours) {
                    redContourArea = Imgproc.contourArea(c);
                    Imgproc.minEnclosingCircle(new MatOfPoint2f(c.toArray()), redCenter, redRadius);
                    Imgproc.circle(redEdges, redCenter, Math.round(redRadius[0]), new Scalar(255, 255, 255)); //Draws a circle around each contour, this is only for bugtesting purposes
                    if (redContourArea / (Math.PI * Math.pow(redRadius[0], 2)) >= 0.67) {
                        circularContours.add(c);
                    }
                }
            }
            //Pick the largest circular contour
            if(circularContours.size() > 0) {
                for (MatOfPoint c : circularContours) {
                    redContourArea = Imgproc.contourArea(c);
                    if (redContourArea > redBiggestContourArea) {
                        redBiggestContourArea = redContourArea;
                        biggestRedContour = c;
                    }
                }
            }
            //draw all the found contours on the redEdges mat. This is for bugtesting
            Imgproc.drawContours(redEdges, contours, -1, new Scalar(255, 255, 255));

            //If it found a valid circular contour, get it's center and radius and draw it on the screen. Also store it in circleBox in case it can't find any valid contours next frame
            if(biggestRedContour.size().width > 0 && biggestRedContour.size().height > 0) {
                Imgproc.minEnclosingCircle(new MatOfPoint2f(biggestRedContour.toArray()),redCenter,redRadius);
                Imgproc.circle(raw,redCenter,Math.round(redRadius[0]),new Scalar(255,0,0),3,8,0);
                Imgproc.circle(raw,redCenter,3,new Scalar(0,255,0),-1,8,0);
                circleBox.clear(1);
                circleBox.clear(2);
                circleBox.add(redRadius[0],1);
                circleBox.add(redCenter,2);
                circleBox.add(redCenter.x,5); //This is for the cumulative line average idea I had
            }
            //If it couldn't find a valid contour, just use the data from last frame (or the preset starting data)
            else {
                prevRadius = Math.round((float) circleBox.get(0,1));
                prevCenter = (Point) circleBox.get(0,2);
                Imgproc.circle(raw,prevCenter,prevRadius,new Scalar(255,0,0),3,8,0);
                Imgproc.circle(raw,prevCenter,3,new Scalar(0,255,0),-1,8,0);
                telemetry.addLine("Level Up! Robot has evolved into Red Monkey."); //Display this message on the screen to let us know it lost the ball
            }
            //Clear both lists of contours so we can use them again in the blue code
            contours.clear();
            circularContours.clear();

            //Start of Blue code
            //It works the same as the red code

            Imgproc.Canny(blue, blueEdges, 30, 60);

            blue.release();

            Imgproc.resize(blueEdges, blueEdges, new Size(160, 90));
            Imgproc.morphologyEx(blueEdges, blueEdges, Imgproc.MORPH_CLOSE, structure);
            Imgproc.resize(blueEdges, blueEdges, new Size(640, 360));

            Imgproc.findContours(blueEdges,contours,new Mat(),Imgproc.RETR_TREE,Imgproc.CHAIN_APPROX_SIMPLE);
            if(contours.size() > 0) {
                for (MatOfPoint c : contours) {
                    Imgproc.fillConvexPoly(blueEdges, c, new Scalar(255, 255, 255));
                }
                Imgproc.findContours(blueEdges, contours, new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

                for (MatOfPoint c : contours) {
                    blueContourArea = Imgproc.contourArea(c);
                    Imgproc.minEnclosingCircle(new MatOfPoint2f(c.toArray()), blueCenter, blueRadius);
                    Imgproc.circle(blueEdges, blueCenter, Math.round(blueRadius[0]), new Scalar(255, 255, 255));
                    telemetry.addData("Percent Circularity", 100*Imgproc.contourArea(c)/(Math.PI*Math.pow(blueRadius[0],2)));
                    if (blueContourArea / (Math.PI * Math.pow(blueRadius[0], 2)) >= 0.67) {
                        circularContours.add(c);
                    }
                }
            }
            if(circularContours.size() > 0) {
                for (MatOfPoint c : circularContours) {
                    blueContourArea = Imgproc.contourArea(c);
                    if (blueContourArea > blueBiggestContourArea) {
                        blueBiggestContourArea = blueContourArea;
                        biggestBlueContour = c;
                    }
                }
            }
            Imgproc.drawContours(blueEdges, contours, -1, new Scalar(255, 255, 255));

            if(biggestBlueContour.size().width > 0 && biggestBlueContour.size().height > 0) {
                Imgproc.minEnclosingCircle(new MatOfPoint2f(biggestBlueContour.toArray()),blueCenter,blueRadius);
                Imgproc.circle(raw,blueCenter,Math.round(blueRadius[0]),new Scalar(0,0,255),3,8,0);
                Imgproc.circle(raw,blueCenter,3,new Scalar(0,255,0),-1,8,0);
                circleBox.clear(3);
                circleBox.clear(4);
                circleBox.add(blueRadius[0],3);
                circleBox.add(blueCenter,4);
                circleBox.add(blueCenter.x,5);
            }
            else {
                prevRadius = Math.round((float) circleBox.get(0,3));
                prevCenter = (Point) circleBox.get(0,4);
                Imgproc.circle(raw,prevCenter,prevRadius,new Scalar(0,0,255),3,8,0);
                Imgproc.circle(raw,prevCenter,3,new Scalar(0,255,0),-1,8,0);
                telemetry.addLine("Level Up! Robot evolved into Blue Monkey");
            }

            //Commented out is the previous ball tracking algorithm

            //Imgproc.medianBlur(blue, blue, 21);
            /*
            Imgproc.Canny(blue, blueEdges, 30, 60);

            Imgproc.findContours(blueEdges,contours,new Mat(),Imgproc.RETR_TREE,Imgproc.CHAIN_APPROX_SIMPLE);

            for(MatOfPoint c:contours) {
                blueContourArea = Imgproc.contourArea(c);
                if(blueContourArea > blueBiggestContourArea) {
                    blueBiggestContourArea = blueContourArea;
                    biggestBlueContour = c;
                }
            }

            blueBiggestContourIndex = contours.indexOf(biggestBlueContour);

            blank2 = new Mat(blueEdges.size(), blueEdges.type());

            Imgproc.drawContours(blueEdges,contours,-1,new Scalar(255,255,255));
            Imgproc.drawContours(blank2,contours,blueBiggestContourIndex,new Scalar(255,255,255),10);

            Imgproc.resize(blank2, blank2, new Size(160, 90));
            Imgproc.morphologyEx(blank2, blank2, Imgproc.MORPH_CLOSE, structure);
            Imgproc.resize(blank2, blank2, new Size(640, 360));

            structure.release();

            Imgproc.HoughCircles(blank2, blueCircles, CV_HOUGH_GRADIENT, 1, blank2.rows()/8, 100, 20, 0, 0);

            //blueEdges.release();

            for (int x = 0; x < blueCircles.cols(); x++) {
                double vCircle[] = blueCircles.get(0, x);
                center = new Point(Math.round(vCircle[0]), Math.round(vCircle[1]));
                int radius = (int) Math.round(vCircle[2]);
                // draw the circle center
                Imgproc.circle(raw, center, 3, new Scalar(0, 255, 0), -1, 8, 0);
                // draw the circle outline
                Imgproc.circle(raw, center, radius, new Scalar(0, 0, 255), 3, 8, 0);

                blueXVals.add(center.x);
            }

            blueCircles.release();
*/
            avg = circleBox.getMean(5); //Calculates cumulative average of the x coordiates of the balls, getting point in between them
            Imgproc.line(raw, new Point(avg, 0), new Point(avg, 720), new Scalar(255, 255, 255)); //Draw a line at the midpoint of the two bals

            //display data
            direction = blueCenter.x < redCenter.x ? "left" : blueCenter.x > redCenter.x ? "right" : "???????? blame crowforce";
            telemetry.addData("blue location", direction);
            direction = redCenter.x < blueCenter.x ? "left" : redCenter.x > blueCenter.x ? "right" : "???????? blame crowforce";
            telemetry.addData("red location", direction);
            direction = redCenter.x < avg ? "left" : "right";
            telemetry.addData("red location (line based)", direction);
            direction = blueCenter.x < avg ? "left" : "right";
            telemetry.addData("blue location (line based)", direction);
            telemetry.addData("line value", avg);
            telemetry.update();
        } catch (Exception e) {
            telemetry.addData("error", e.getMessage());
            e.printStackTrace();
        }

        telemetry.update();

        //resize image to a size our phone can display, and return it
        Mat returnImage = raw;
        Imgproc.resize(returnImage,returnImage,new Size(1280,720));
        return returnImage;
    }

    public void startOpenCV(CameraBridgeViewBase.CvCameraViewListener2 cameraViewListener) {
        FtcRobotControllerActivity.turnOnCameraView.obtainMessage(1, cameraViewListener).sendToTarget();
    }

    public void stopOpenCV() {
        try {
            FtcRobotControllerActivity.turnOffCameraView.obtainMessage().sendToTarget();
        } catch (Exception e) {
            Log.d("Stack Trace", e.getStackTrace().toString());
            telemetry.addData("error:", e.getMessage());
            telemetry.addData("cause:", e.getCause());
            telemetry.addData("stack trace:", e.getStackTrace());
            telemetry.update();
        }
    }
}