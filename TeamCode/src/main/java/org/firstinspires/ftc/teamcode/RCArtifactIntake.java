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

    public RCArtifactIntake(Hardware hardware, int intakeCMD, boolean isIntakeDirection, boolean skipWait) {
        this.hardware = hardware;
        this.intakeCMD = intakeCMD;
        this.isIntakeDirection = isIntakeDirection;
        this.skipWait = skipWait;
    }

    public RCArtifactIntake(Hardware hardware, double power, boolean skipWait) {
        this.hardware = hardware;
        this.power = power;
        this.skipWait = skipWait;
    }

    public void run() {
        switch (intakeCMD) {
            case CMD_OFF:
                action = hardware.artifactIntake.noRotationAction();
                action.run(hardware.packet);
                hardware.logMessage(false, "RCSampleIntake", "Intake Set To Off");
                break;
            case CMD_ON:
                if(isIntakeDirection){
                    action = hardware.artifactIntake.intakeAction();
                    action.run(hardware.packet);
                    hardware.logMessage(false, "RCSampleIntake", "Intake Set To Intaking");
                    break;
                }else{
                    action = hardware.artifactIntake.outtakeAction();
                    action.run(hardware.packet);
                    hardware.logMessage(false, "RCSampleIntake", "Intake Set To Outtaking");
                    break;
                }
            case CMD_TOGGLE_POWER:
                switch(hardware.artifactIntake.getState()){
                    case ArtifactIntake.INTAKING:
                    case ArtifactIntake.OUTTAKING:
                    case ArtifactIntake.STATIONARY:
                        if(isIntakeDirection){
                            action = hardware.artifactIntake.intakeAction();
                            action.run(hardware.packet);
                            hardware.logMessage(false, "RCSampleIntake", "Intake Set To Intaking");
                        }else{
                            action = hardware.artifactIntake.outtakeAction();
                            action.run(hardware.packet);
                            hardware.logMessage(false, "RCSampleIntake", "Intake Set To Outtaking");
                        }
                        break;
                }
                break;
            case CMD_CUSTOM_POWER:
                action = hardware.artifactIntake.setPowerAction(power);
                action.run(hardware.packet);
                hardware.logMessage(false, "RCSampleIntake", "Intake Set To " + power + " Power");
        }
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