package org.firstinspires.ftc.teamcode.season.relicrecov18.polaris.v1.program.programs;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.opencv.core.MatOfInt;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.concurrent.Callable;

import static org.opencv.imgproc.Imgproc.drawContours;
import static org.opencv.imgproc.Imgproc.getStructuringElement;

/**
 * Created by Sage Creek Level Up on 12/12/2017.
 */

public class BallDetector implements Callable<Circle> {
    private Circle prevCircle;

    private ArrayList<MatOfPoint> hullPoints = new ArrayList<>();
    private ArrayList<MatOfPoint> contours = new ArrayList<>();
    private ArrayList<MatOfPoint> circularContours = new ArrayList<>();

    private Mat input = new Mat();
    private Mat edges = new Mat();
    private Mat mask = new Mat();

    private Mat structure = getStructuringElement(Imgproc.MORPH_ELLIPSE, new Size(15, 15));;

    private MatOfInt hull = new MatOfInt();

    private MatOfPoint hullPoint = new MatOfPoint();
    private MatOfPoint biggestContour = new MatOfPoint();

    private MatOfDouble mean = new MatOfDouble();
    private MatOfDouble std = new MatOfDouble();

    private Point center = new Point();
    private float[] radius = new float[1];

    private Point contourCenter = new Point();
    private float[] contourRadius = new float[1];

    private double contourArea = 0;
    private double biggestContourArea = 0;
    private double circularityTheshold = 0;

    //Constructor, this is where you are going to pass all the input data for the thread to run
    public BallDetector(Mat input, Mat mask, double circularityTheshold, Circle prevCircle) {
        this.input = input; //The binary input Mat
        this.mask = mask; //A mask we apply to remove errors from the inRange function
        this.circularityTheshold = circularityTheshold; //The minimum acceptable circularity percentage
        this.prevCircle = prevCircle; //The circle returned from the previous run
    }

    //Finds the MatOfPoint object that is described by a MatOfInt object
    private MatOfPoint convertIndexesToPoints(MatOfPoint contour, MatOfInt indexes) {
        int[] arrIndex = indexes.toArray();
        Point[] arrContour = contour.toArray();
        Point[] arrPoints = new Point[arrIndex.length]; //Makes an array of points that contains the same number of elements as the MatOfInt

        for (int i=0;i<arrIndex.length;i++) {
            arrPoints[i] = arrContour[arrIndex[i]]; //Add whatever point is at the MatOfInt's value to the array of points
        }

        MatOfPoint outputHull = new MatOfPoint();
        outputHull.fromArray(arrPoints);
        return outputHull;
    }

    //Call() is the code the thread will run once you start it
    @Override
    public Circle call(){

        //Apply mask to the input to remove errors
        Core.bitwise_and(input,mask,input);

        //Find edges of the masked input image using an adaptive threshold
        Core.meanStdDev(input, mean, std);
        Imgproc.Canny(input, edges, mean.get(0,0)[0] - std.get(0,0)[0], mean.get(0,0)[0] + std.get(0,0)[0]);

        //Empty the input Mat
        input.release();

        //Close all the holes in the contours
        //It is resized to increase speed and impact of the kernel
        Imgproc.resize(edges, edges, new Size(160, 90));
        Imgproc.morphologyEx(edges, edges, Imgproc.MORPH_CLOSE, structure);
        Imgproc.resize(edges, edges, new Size(640, 360));

        //Get all the contours of the edges
        Imgproc.findContours(edges,contours,new Mat(),Imgproc.RETR_TREE,Imgproc.CHAIN_APPROX_SIMPLE);

        //Only run this code if it found any contours (meaning the image isn't empty)
        //The image is null when opencv first starts, so you have to put this if statement in here to prevent errors
        if(contours.size() > 0) {

            //Get the convex hull for each of the contours, and add them to a list
            for(MatOfPoint c : contours) {
                Imgproc.convexHull(c, hull);
                hullPoint = convertIndexesToPoints(c,hull); //Finds the MatOfPoint that the hull describes
                hullPoints.add(hullPoint);
            }

            //Draw the convex hulls back on to the edges
            //This is done so that if any contour is unclosed, the algorithm can still use the convex hull for reference
            drawContours(edges,hullPoints,-1,(new Scalar(255,255,255)));

            //Perform a median blue to reduce noise
            Imgproc.medianBlur(edges,edges,5);

            //Find the contours of the edges and convex hulls
            Imgproc.findContours(edges, contours, new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

            //Clear the edges Mat
            edges.release();

            //For every contour in the filled shapes, calculate that contour's percent circularity based on its theoretical and predicted area. If it has a circularity of >= circularityThreshold, set it aside for later analysis
            for (MatOfPoint c : contours) {
                contourArea = Imgproc.contourArea(c);
                Imgproc.minEnclosingCircle(new MatOfPoint2f(c.toArray()), contourCenter, contourRadius); //Get the center and radius of the minimum size circle that encloses the contour
                if (contourArea / (Math.PI * Math.pow(contourRadius[0], 2)) >= circularityTheshold) {
                    circularContours.add(c); //Add contour to list for later analysis
                }
            }
        }
        //Pick the largest circular contour if it found any above the threshold
        if(circularContours.size() > 0) {
            for (MatOfPoint c : circularContours) {
                contourArea = Imgproc.contourArea(c);
                if (contourArea > biggestContourArea) {
                    biggestContourArea = contourArea;
                    biggestContour = c;
                }
            }
        }

        //If it found a valid circular contour, get it's center and radius and store it in the center and radius variables
        if(biggestContour.size().width > 0 && biggestContour.size().height > 0) {
            Imgproc.minEnclosingCircle(new MatOfPoint2f(biggestContour.toArray()),center,radius);
        }

        //If it couldn't find a valid contour, just use the data it got from the last frame
        else {
            radius[0] = Math.round(prevCircle.radius);
            center = prevCircle.center;
        }

        //Clear the lists of contours
        contours.clear();
        circularContours.clear();
        hullPoints.clear();

        return new Circle(center,radius[0]);
    }
}