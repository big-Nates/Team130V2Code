package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;

public class RCArtifactIntake extends RobCommand {

    public static final int CMD_OFF = 0;
    public static final int CMD_ON = 1;

    public static final int CMD_TOGGLE_POWER = 2;

    public static final int CMD_CUSTOM_POWER = -1;

    //Static initialization may cause errors
    public static boolean isIntakeDirection = true;
    private Hardware hardware = null;
    private double power = 0;
    private int intakeCMD = -1;
    private boolean skipWait = false;
    public Action action;

    private final double OUTTAKE_DELAY_TIME = 0.5;

    private double delayTime = 0.0;
    private double startTime = 0.0;
    private int num = 0;

    public RCArtifactIntake(Hardware hardware, double power , int num) {
        this.hardware = hardware;
        this.power = power;
        this.num = num;
    }

    public RCArtifactIntake(Hardware hardware, double power, boolean skipWait) {
        this.hardware = hardware;
        this.power = power;
        this.skipWait = skipWait;
    }

    public void run() {
        if(num == 1){
            action = hardware.artifactIntake.setOuterPowerAction(power);
            action.run(hardware.packet);
        }else if(num == 2){
            action = hardware.artifactIntake.setInnerPowerAction(power);
            action.run(hardware.packet);
        }else{
            action = hardware.artifactIntake.setPowerAction(power);
            action.run(hardware.packet);
        }

        hardware.logMessage(false, "RCSampleIntake", "Intake Set To " + power + " Power");
    }

    public boolean isComplete() {
        if (skipWait) {
            hardware.logMessage(false, "RCSampleIntake", "Command Complete, skipped wait");
            return true;
        }
        return !action.run(hardware.packet);
    }

    @Override
    public String toString() {
        return "RCSampleIntake{" +
                ", servoCMD=" + intakeCMD +
                ", skipWait=" + skipWait +
                ", startTime=" + startTime +
                '}';
    }
}