package org.firstinspires.ftc.teamcode.season.relicrecov18.polaris.v1.program.programs;

import android.os.Environment;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.firstinspires.ftc.teamcode.opMode.DriverControlledProgram;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.season.relicrecov18.polaris.v1.robot.Polaris;
import org.opencv.android.CameraBridgeViewBase;
import org.opencv.core.Mat;
import org.opencv.imgcodecs.Imgcodecs;

/**
 * Created by Andrew on 12/31/2017.
 */
@TeleOp(name = "judgebait-pro", group = "Teleop")
public class BackgroundDrive extends DriverControlledProgram implements CameraBridgeViewBase.CvCameraViewListener2{
    private int num = 0;
    private Polaris polaris;
    @Override
    protected Robot buildRobot() {
        polaris = new Polaris(this);
        return polaris;
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
        try {
            Imgcodecs.imwrite(Environment.getExternalStorageDirectory().getAbsolutePath()+"/DataCollection/"+num+".jpg",raw);
            num++;
            polaris.telemetry.addLine("Created "+Integer.toString(num+1)+" background images");
            polaris.telemetry.update();
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