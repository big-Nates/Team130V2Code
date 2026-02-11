package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;

public class RCServoGate extends RobCommand {
    public static final int CMD_CLOSE = 0;
    public static final int CMD_OPEN = 1;
    public static final int CMD_CUSTOM = 2;
    private Hardware hardware = null;
    private int servoCMD = 0;
    private boolean skipWait = false;
    private Action action;

    private double leftPosition;
    private double rightPosition;

    private double startTime = 0.0;

    public RCServoGate(Hardware hardware, int servoCMD, boolean skipWait) {
        this.hardware = hardware;
        this.servoCMD = servoCMD;
        this.skipWait = skipWait;
    }

    public RCServoGate(Hardware hardware, double leftPosition, double rightPosition) {
        this.hardware = hardware;
        this.leftPosition = leftPosition;
        this.rightPosition = rightPosition;
    }

    public void run() {
        startTime = hardware.getCurrentTime();
        if(servoCMD == CMD_CLOSE){
            action = hardware.servoGate.closeGateAction();
            action.run(hardware.packet);
            hardware.logMessage(false, "RCSpecimenClaw", "Specimen Claw Set To close Position");
        }else if(servoCMD == CMD_OPEN){
            action = hardware.servoGate.openGateAction();
            action.run(hardware.packet);
            hardware.logMessage(false, "RCSpecimenClaw", "Specimen Claw Set To open Position");
        }else{
            action = hardware.servoGate.setClawPositionAction(leftPosition, rightPosition);
            action.run(hardware.packet);
            hardware.logMessage(false, "RCSpecimenClaw", "Specimen Claw Set To open Position");
        }
    }


    public boolean isComplete() {
        if (skipWait) {
            hardware.logMessage(false, "RCSpecimenClaw", "Command Complete, skipped wait");
            return true;
        }
        return !action.run(hardware.packet);
    }

    @Override
    public String toString() {
        return "RCSpecimenClaw{" +
                ", servoCMD (to open)=" + servoCMD +
                ", skipWait=" + skipWait +
                ", startTime=" + startTime +
                '}';
    }
}