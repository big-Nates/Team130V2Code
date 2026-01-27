package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;

public class RCRoadrunner1 extends RobCommand {
    private Hardware hardware = null;
    private Pose2d previousPose = null;
    private Action action = null;
    private MecanumDrive drive = null;


    public RCRoadrunner1(Action action, Hardware hardware){
        this.hardware = hardware;
        this.action = action;
    }

    public void run(){
        action.run(hardware.packet);
    }

    public boolean isComplete(){
        return !action.run(hardware.packet);
    }


    public Action getAction(){
        return action;
    }
}