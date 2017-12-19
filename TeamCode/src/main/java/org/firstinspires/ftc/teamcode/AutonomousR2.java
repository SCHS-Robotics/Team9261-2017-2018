package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.BaseOpModeR;

@Autonomous(name = "AutoR2", group = "autonomous")
public class AutonomousR2 extends BaseOpModeR
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        waitForStart();

        initilaize();

        JewelKnock(1);
        BackwardDistance(1, 36);
        SCWDegrees(90);
        ForewardDistance(1, 12);
        SCWDegrees(90);
        while (opModeIsActive())
        {
            ForewardDistance(1,6);
            GlyphPickUp(-1);
            BackwardDistance(1, 6);
            SCWDegrees(153.435);
            ForewardDistance(1, 54);
            GlyphPickUp(1);
            SCCWDegrees(180);
            ForewardDistance(1, 54);
            SCCWDegrees(26.565);

        }
    }
}
