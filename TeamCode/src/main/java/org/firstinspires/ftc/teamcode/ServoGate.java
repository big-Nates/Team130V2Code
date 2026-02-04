package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

import androidx.annotation.NonNull;

public class ServoGate {
    private OpMode opMode = null;
    private Hardware hardware = null;

    private Servo gateServo = null;
    private Servo leftGateServo = null;

    public static final double OPEN_POS = 0.15; // NEED TO BE SET

    public static final double LEFT_OPEN_POS = 0.5; // NEED TO BE SET
    public static final double CLOSE_POS = 0.6; // NEED TO BE SET
    public static final double LEFT_CLOSE_POS = 0.05; // NEED TO BE SET
    public boolean isOpen = false;

    public ServoGate(OpMode opMode, Hardware hardware) {
        this.opMode = opMode;
        this.hardware = hardware;
    }

    public void init(){
        gateServo = hardware.gateServo;
        leftGateServo = hardware.leftGateServo;
        closeGate();
    }

    public void openGate(){
        gateServo.setPosition(OPEN_POS);
        leftGateServo.setPosition(LEFT_OPEN_POS);
        isOpen = true;
    }

    public void closeGate(){
        gateServo.setPosition(CLOSE_POS);
        leftGateServo.setPosition(LEFT_CLOSE_POS);
        isOpen = false;
    }

    public Action openGateAction(){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                openGate();
                if (Math.abs(hardware.gateServo.getPosition() - OPEN_POS) < 0.01) {
                    hardware.logMessage(false, "RCServoGate", "Command Complete, at requested position");
                    return false;
                }
                return true;
            }

        };
    }

    public Action closeGateAction(){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                closeGate();
                if (Math.abs(hardware.gateServo.getPosition() - CLOSE_POS) < 0.03) {
                    hardware.logMessage(false, "RCServoGate", "Command Complete, at requested position");
                    return false;
                }
                return true;
            }

        };
    }

    public Action setClawPositionAction(double position){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                setRightServoPosition(position);
                if (Math.abs(hardware.gateServo.getPosition() - position) < 0.03) {
                    hardware.logMessage(false, "RCSpecimenClaw", "Command Complete, at requested position");
                    return false;
                }
                return true;
            }

        };
    }

    public void setRightServoPosition(double position){
        gateServo.setPosition(Math.min(Math.max(position, 0.0), 1.0));
    }

    public void setLeftServoPosition(double position){
        leftGateServo.setPosition(Math.min(Math.max(position, 0.0), 1.0));
    }
}
