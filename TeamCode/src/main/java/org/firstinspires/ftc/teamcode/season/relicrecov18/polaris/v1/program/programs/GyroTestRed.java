package org.firstinspires.ftc.teamcode.season.relicrecov18.polaris.v1.program.programs;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.season.relicrecov18.polaris.v1.program.PolarisAutonomousProgram;
import org.firstinspires.ftc.teamcode.season.relicrecov18.polaris.v1.robot.Jewel;
import org.opencv.android.CameraBridgeViewBase;
import org.opencv.core.Core;
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
import java.util.List;

import static org.opencv.imgproc.Imgproc.CV_HOUGH_GRADIENT;

/**
 * Created by Andrew on 12/7/2017.
 */
@Autonomous(name = "Auto TestRed", group = "Autonomous")
public class GyroTestRed extends PolarisAutonomousProgram implements CameraBridgeViewBase.CvCameraViewListener2 {
    String direction;
    @Override
    protected Robot buildRobot() {
        Robot robot = super.buildRobot();
        return robot;
    }

    @Override
    public void main() throws InterruptedException {
        startOpenCV(this);
        jewel.moveAxe(Jewel.axePos.UP);
        sleep(3000);
        if (direction =="left"){
            jewel.moveArm(Jewel.armPos.LEFT);
            sleep(1000);
            //drive.driveDistance(5, -0.8);
            jewel.moveAxe(Jewel.axePos.DOWN);
            sleep(1000);
        }
        else if (direction == "right"){
            jewel.moveArm(Jewel.armPos.LEFT);
            sleep(1000);
            //drive.driveDistance(5, 0.8);
           // jewel.moveAxe(Jewel.axePos.FRONT);
            sleep(1000);
        }
        jewel.moveAxe(Jewel.axePos.UP);
        sleep(1000);
        jewel.moveArm(Jewel.armPos.RIGHT);
        sleep(1000);
        drive.driveDistance(30, 0.8);
        sleep(2000);
        /*
        if(navi.key == RelicRecoveryVuMark.LEFT)
        drive.driveDistance(6.1, -0.8);
        }else if(navi.key == RelicRecoveryVuMark.RIGHT){
        drive.driveDistance(6.1, 0.8);
        }
        sleep(1000);
        */
       // drive.turnExact(90, 0.2);
        sleep(1000);
        drive.driveDistance(1.5,0.8);
        drive.driveDistance(3, 0.8);
        sleep(2000);
        drive.driveDistance(3,-0.8);

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
        Mat gray = inputFrame.gray();
        Mat hsv = new Mat();
        Mat lower = new Mat();
        Mat upper = new Mat();
        Mat white = new Mat();
        Mat combined = new Mat();
        Mat circles = new Mat();
        List<MatOfPoint> contours = new ArrayList<>();
        MatOfPoint2f approx = new MatOfPoint2f();
        double line = 448; //emergency value
        ArrayList<Double> xVals = new ArrayList<>();
        try {

            Imgproc.cvtColor(raw,hsv,Imgproc.COLOR_RGBA2RGB);
            Imgproc.cvtColor(hsv,hsv,Imgproc.COLOR_RGB2HSV);

            //Filters for only red pixels
            Core.inRange(hsv, new Scalar(0,100,100), new Scalar(2,255,255),lower);
            Core.inRange(hsv, new Scalar(160,100,100), new Scalar(179,255,255),upper);

            Imgproc.threshold(gray,white,200,255,Imgproc.THRESH_BINARY);

            //Smash the images together
            Core.addWeighted(lower,1,upper,1,0,combined);

            //Blur out the evidence of cheating
            Imgproc.GaussianBlur(combined,combined,new Size(9,9),2,2);
            Imgproc.GaussianBlur(white,white,new Size(9,9),2,2);

            Imgproc.findContours(white,contours,new Mat(),Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

            MatOfPoint biggestContour = new MatOfPoint();
            MatOfInt hull = new MatOfInt();


            for(MatOfPoint c:contours) {
                double peri = Imgproc.arcLength(new MatOfPoint2f(c.toArray()), true);
                Imgproc.approxPolyDP(new MatOfPoint2f(c.toArray()), approx, 0.02 * peri, true);
                Imgproc.convexHull(c, hull);
                if(approx.toList().size() == 4 && hull.size().height > biggestContour.size().height) {
                    biggestContour = c;
                    ArrayList approxlist = new ArrayList<MatOfPoint>();
                    approxlist.clear();
                    approxlist.add(c);
                }
            }
            Rect divider = Imgproc.boundingRect(biggestContour);
            line = (divider.x + divider.width/2);
            Imgproc.rectangle(raw, new Point(divider.x,divider.y),new Point(divider.x+divider.width,divider.y+divider.height),new Scalar(255,0,0),2);


            //Circle all duct tape
            Imgproc.HoughCircles(combined,circles,CV_HOUGH_GRADIENT, 1, combined.rows()/8,100, 20, 0, 0);
            for (int x = 0; x < circles.cols(); x++)
            {
                double vCircle[]=circles.get(0,x);

                Point center=new Point(Math.round(vCircle[0]), Math.round(vCircle[1]));
                int radius = (int)Math.round(vCircle[2]);

                xVals.add(center.x);
                // draw the circle center
                Imgproc.circle(raw, center, 3,new Scalar(0,255,0), -1, 8, 0 );
                // draw the circle outline
                Imgproc.circle(raw, center, radius, new Scalar(0,0,255), 3, 8, 0 );
            }
            int total = 0;
            for(double val:xVals) {
                total += val;
            }
            int avg = Math.round(total/xVals.size());
            Imgproc.line(raw,new Point(line,0),new Point(line,720),new Scalar(255,255,255));
            direction = avg <= line ? "right":"left";
            telemetry.addData("location", direction );
            telemetry.update();

        }
        catch (Exception e) {
            telemetry.addData("error",e.getMessage());
        }

        // This is where the magic will happen. inputFrame has all the data for each camera frame.
        telemetry.update();
        //Imgproc.resize(circles,circles,new Size(1280,720));
        return raw;
    }
    public void startOpenCV(CameraBridgeViewBase.CvCameraViewListener2 cameraViewListener) {
        if (FtcRobotControllerActivity.mOpenCvCameraView.isEnabled()) {
            FtcRobotControllerActivity.mOpenCvCameraView.disableView();
        }
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
            telemetry.addData("error:", e.getMessage());
            telemetry.addData("cause:", e.getCause());
            telemetry.addData("stack trace:",e.getStackTrace());
            telemetry.update();
        }
    }
}
