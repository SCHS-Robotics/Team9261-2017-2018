package org.firstinspires.ftc.teamcode.opMode;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManager;

import org.firstinspires.ftc.robotcontroller.internal.FtcOpModeRegister;
import org.firstinspires.ftc.teamcode.robot.Robot;

/**
 * Created by Andrew on 12/6/2017.
 */

public abstract class DriverControlledProgram extends OpMode {
    private Robot robot;



    protected abstract Robot buildRobot();

    protected void onStart(){}
    protected void onUpdate(){}
    protected void onStop(){}

    @Override
    public final void init() {
        robot = buildRobot();

        try {
            robot.init();
        }catch (Exception ex){
            telemetry.addData("ERROR!!!", ex.getMessage());
        }
    }

    @Override
    public final void start() {


        onStart();
    }

    @Override
    public final void loop() {
        robot.driverControlledUpdate();
        onUpdate();
    }

    @Override
    public void stop() {
        onStop();
    }

    protected final Robot getRobot(){
        return robot;
    }
}
