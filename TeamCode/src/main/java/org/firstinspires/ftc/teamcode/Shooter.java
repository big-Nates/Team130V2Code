package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Shooter {

    private OpMode opMode;
    private Hardware hardware;
    private DcMotorEx shooterFlyWheel;

    private final ElapsedTime runtime = new ElapsedTime();
    private final ElapsedTime timeout = new ElapsedTime();

    public double shootingPower = 0;
    private double startTime = 0;


    public int state = 0;
    public static final int INACTIVEOUTTAKE = 0;
    public static final int ACTIVEOUTTAKE = 1;

    public Shooter(OpMode opMode, Hardware hardware) {
        this.opMode = opMode;
        this.hardware = hardware;
    }

    public void init() {
        opMode.telemetry.addData("Shooter Status", "Initializing");
        shooterFlyWheel = hardware.shooterFlyWhell;
        runtime.reset();
        timeout.reset();

        state = INACTIVEOUTTAKE;
        opMode.telemetry.update();
    }

    public void setPower(double power) {
        shooterFlyWheel.setPower(power);
        this.shootingPower = power;
        state = ACTIVEOUTTAKE;
    }

    public Action setPowerAction(double targetPower){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                setPower(targetPower);
                return true;
            }

        };
    }

    public void stop() {
        shooterFlyWheel.setPower(0.0);
        shooterFlyWheel.setVelocity(0.0);
        state = INACTIVEOUTTAKE;
    }
}