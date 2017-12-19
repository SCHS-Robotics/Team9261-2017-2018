package org.firstinspires.ftc.teamcode.robot;




/**
 * Created by Andrew on 12/5/2017.
 */

public abstract class SubSystem {

    protected Robot robot;
    public abstract void init() throws InterruptedException;

    public abstract void handle();

    public abstract void stop();

    public SubSystem(Robot robot){
        this.robot = robot;
    }
}
