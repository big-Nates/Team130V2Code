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
    private double position;

    private double startTime = 0.0;

    public RCServoGate(Hardware hardware, int servoCMD, boolean skipWait) {
        this.hardware = hardware;
        this.servoCMD = servoCMD;
        this.skipWait = skipWait;
    }

    public RCServoGate(Hardware hardware, int servoCMD, boolean skipWait, double position) {
        this.hardware = hardware;
        this.servoCMD = servoCMD;
        this.skipWait = skipWait;
        this.position = position;
    }

    public void run() {
        startTime = hardware.getCurrentTime();
        switch (servoCMD) {
            case CMD_CLOSE:
                action = hardware.servoGate.closeClawAction();
                action.run(hardware.packet);
                hardware.logMessage(false, "RCSpecimenClaw", "Specimen Claw Set To close Position");
                break;
            case CMD_OPEN:
                action = hardware.servoGate.openClawAction();
                action.run(hardware.packet);
                hardware.logMessage(false, "RCSpecimenClaw", "Specimen Claw Set To open Position");
                break;
            case CMD_CUSTOM:
                action = hardware.servoGate.setClawPositionAction(position);
                action.run(hardware.packet);
                hardware.logMessage(false, "RCSpecimenClaw", "Specimen Claw Set To open Position");
                break;

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