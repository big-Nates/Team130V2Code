package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;

public class RCOuttake extends RobCommand {
    private Hardware hardware = null;
    private int position = 50;
    private double power = 0.0;
    private boolean skipWait = true;
    private Action action = null;

    public static double inchesToPositionConversion = .01185;

    public RCOuttake(Hardware hardware, double power) {
        this.hardware = hardware;
        this.power = power;
        action = hardware.shooter.setPowerAction(power);
    }

    public RCOuttake(Hardware hardware,double power, boolean skipWait) {
        this.hardware = hardware;
        this.power = power;
        this.skipWait = skipWait;
        action = hardware.shooter.setPowerAction(power);
    }

    public RCOuttake(Hardware hardware, int velocity){
        this.hardware = hardware;
        action = hardware.shooter.setVelocityAction(velocity);
    }

    public void run() {
        hardware.logMessage(false, "RCLiftExtend", "Command Ran, set to power " + this.power);
        action.run(hardware.packet);
    }

    public boolean isComplete() {
        if (skipWait) {
            hardware.logMessage(false, "RCLiftExtend", "Command Complete, Skip Wait");
            return true;
        }
        return !action.run(hardware.packet);
    }

    public Action getAction(){
        return action;
    }

    @Override
    public String toString() {
        return "RCLiftExtend{" +
                ", power=" + power +
                ", skipWait=" + skipWait +
                '}';
    }
}