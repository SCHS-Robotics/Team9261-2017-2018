package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.BaseOpModeR;

@Autonomous(name = "AutoB1", group = "autonomous")
public class AutonomousB1 extends BaseOpModeR
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        waitForStart();

        initilaize();

        JewelKnock(1);
        ForewardDistance(1,36); //go off the platform to the line directly accross from the cipher
        SCCWDegrees(90); //Turn 90 degrees to face the glyph pit
        ForewardDistance(1,24); //Go to glpyh pit
        while (opModeIsActive())
        {
            GlyphPickUp(-1);
            SCCWDegrees(180); //Turn 180 degrees to face cipher
            ForewardDistance(1,60); //Go to cipher
            GlyphPickUp(1);
            SCCWDegrees(180); //Turn 180 degrees to face glyph pit
            ForewardDistance(1,60); //Go to glyph pit
        }
    }
}
