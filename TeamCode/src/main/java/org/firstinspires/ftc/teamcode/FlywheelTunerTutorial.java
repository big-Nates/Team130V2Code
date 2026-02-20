package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@TeleOp
public class FlywheelTunerTutorial extends OpMode {
    public DcMotorEx flywheelMotor;
    public Hardware hardware = new Hardware();;

    public double highVel = 3000;
    public double lowVel = 2000;
    double curTargetVelocity = highVel;

    double F = 0;
    double P = 0;
    double[] stepSizes= {10.0, 1.0, 0.1, 0.001, 0.001};
    int stepIndex = 1;

    public void init(){
        hardware.init(hardwareMap, this);
        flywheelMotor = hardware.shooterFlyWheel;
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        flywheelMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        telemetry.addLine("Init complete");
    }

    @Override
    public void init_loop() {
        hardware.updateValues();

        super.init_loop();

        hardware.init_loop();
    }


    @Override
    public void start() {
        hardware.updateValues();
        hardware.servoGate.openGate();
        super.start();
        hardware.start();
    }

    public void loop(){
        if (gamepad1.yWasPressed()){
            if(curTargetVelocity == highVel){
                curTargetVelocity = lowVel;
            }else{
                curTargetVelocity = highVel;
            }
        }

        if (gamepad1.bWasPressed()){
            stepIndex = (stepIndex + 1) % stepSizes.length;
        }

        if (gamepad1.dpadLeftWasPressed()){
            F -= stepSizes[stepIndex];
        }

        if (gamepad1.dpadRightWasPressed()){
            F += stepSizes[stepIndex];
        }

        if (gamepad1.dpadDownWasPressed()){
            P -= stepSizes[stepIndex];
        }

        if (gamepad1.dpadUpWasPressed()){
            P += stepSizes[stepIndex];
        }

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        flywheelMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        flywheelMotor.setVelocity(curTargetVelocity);
        if(gamepad1.leftBumperWasPressed()) {
            hardware.artifactIntake.setPower(1);
        }else if(gamepad1.rightBumperWasPressed()){
            hardware.artifactIntake.setPower(0);
        }

        double curVelocity = flywheelMotor.getVelocity();
        double error = curTargetVelocity - curVelocity;

        telemetry.addData("Target Velocity", curTargetVelocity);
        telemetry.addData("Current Velocity", "%.2f", curVelocity);
        telemetry.addData("Error", error);
        telemetry.addLine("--------------------------");
        telemetry.addData("Tuning P", "%.4f (D-Pad U/D)", P);
        telemetry.addData("Tuning F", "%.4f (D-Pad L/R)", F);
        telemetry.addData("Step Size", "%.4f (B Button)", stepSizes[stepIndex]);
    }

    public void stop() {
        hardware.updateValues();
        hardware.logMessage(false, "ShooterTest", "Stop Button Pressed");
        hardware.stop();
        super.stop();
    }
}
