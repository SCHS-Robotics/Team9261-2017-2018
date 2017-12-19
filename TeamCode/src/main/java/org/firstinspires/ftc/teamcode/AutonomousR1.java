package org.firstinspires.ftc.teamcode.Autonomous.AutoR;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.BaseOpModeR;

import javax.microedition.khronos.opengles.GL;

@Autonomous(name = "AutoR1", group = "autonomous")
public class AutonomousR1 extends BaseOpModeR
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        waitForStart();

        initilaize();

        JewelKnock(1);
        BackwardDistance(1,36); //go off the platform to the line directly accross from the cipher
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
