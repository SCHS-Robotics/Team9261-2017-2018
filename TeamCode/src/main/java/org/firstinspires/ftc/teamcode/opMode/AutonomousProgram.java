package org.firstinspires.ftc.teamcode.opMode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.internal.opmode.OpModeManagerImpl;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.util.Condition;

/**
 * Created by Andrew on 12/5/2017.
 */

public abstract class AutonomousProgram extends LinearOpMode {
    private Robot robot;

    protected abstract Robot buildRobot();

    public abstract void main()throws InterruptedException;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = buildRobot();

        try {
            robot.init();
        } catch (Exception ex) {
            telemetry.addData("ERROR!!!", ex.getMessage());
        }


        try {
            main();
        } catch (Exception ex) {
            telemetry.addData("ERROR!", ex.getMessage());
        }
    }

    protected final Robot getRobot() {
        return robot;
    }

    protected final void waitFor(long millis) {
        long stopTime = System.currentTimeMillis() + millis;
        while (opModeIsActive() && System.currentTimeMillis() < stopTime) {
        }
    }

    protected final void waitFor(Condition condition) {
        while (opModeIsActive() && !condition.get()) {}
    }
}
