package org.firstinspires.ftc.teamcode.season.relicrecov18.polaris.v1.program.programs;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.season.relicrecov18.polaris.v1.program.PolarisAutonomousProgram;

/**
 * Created by andre_000 on 01/01/2018.
 */
@Autonomous(name = "Wall", group = "meme")
public class Wall extends PolarisAutonomousProgram {
    @Override
    public void main() throws InterruptedException {
        belt.getThatWallBuilt();
        belt.rigThatElection();
        waitForStart();
        sleep(4000);
        belt.putThatWallUp();
        belt.electionIsRigged();
        sleep(4000);
    }
}
