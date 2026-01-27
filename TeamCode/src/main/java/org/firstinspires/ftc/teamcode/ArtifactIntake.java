package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

public class ArtifactIntake {
    private OpMode opMode;
    private Hardware hardware;
    private DcMotorEx intakeMotor;

    private DcMotorEx innerIntakeMotor;

    private final ElapsedTime runtime = new ElapsedTime();
    private final ElapsedTime timeout = new ElapsedTime();

    private double INTAKEPOWER = 1.0;
    private double OUTTAKEPOWER = -0.4;
    public double startTime = 0;

    private static final double MAX_TIMEOUT = 5.0;

    public static final int STATIONARY = 0;
    public static final int INTAKING = 1;
    public static final int OUTTAKING = 2;
    private int state = STATIONARY;
    private boolean isRotating = false;

    public ArtifactIntake(OpMode opMode, Hardware hardware) {
        this.opMode = opMode;
        this.hardware = hardware;

    }

    public void init(){
        opMode.telemetry.addData("Intake Status", "Initializing");
        intakeMotor = hardware.intakeMotor;
        innerIntakeMotor = hardware.innerIntakeMotor;
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        innerIntakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        runtime.reset();
        timeout.reset();

        opMode.telemetry.addData("Intake Status", "Initialized");
        opMode.telemetry.update();
    }

    public void doLoop(){
        switch(state){
            case INTAKING:
                break;
            case OUTTAKING:
                if(opMode.time - startTime >= 0.5){
                    noRotation();
                }
                break;
            case STATIONARY:
                break;
        }
    }

    public String getStateString(){
        switch(state){
            case INTAKING:
                return "Intaking";
            case OUTTAKING:
                return "Outtaking";
            case STATIONARY:
                return "Stationary";
        }
        return null;
    }

    public int getState(){
        return state;
    }

    public void setPower(double power){
        intakeMotor.setPower(power);
        innerIntakeMotor.setPower(power);
        state = INTAKING;
    }

    public void singleShoot(double power){
        startTime = opMode.time;
        intakeMotor.setPower(power);
        innerIntakeMotor.setPower(power);
        state = OUTTAKING;
    }

    public void intakeRotation(){
        intakeMotor.setPower(INTAKEPOWER);
        innerIntakeMotor.setPower(INTAKEPOWER);
        state = INTAKING;
    }

    public void noRotation(){
        intakeMotor.setPower(0);
        innerIntakeMotor.setPower(0);
        state = STATIONARY;
    }

    public Action intakeAction(){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                intakeRotation();
                return false;
            }

        };
    }

    public Action outtakeAction(){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                outtakeAction();
                return false;
            }

        };
    }

    public Action noRotationAction(){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                noRotation();
                return false;
            }

        };
    }

    public Action setPowerAction(double power){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                setPower(power);
                return false;

            }

        };
    }

}