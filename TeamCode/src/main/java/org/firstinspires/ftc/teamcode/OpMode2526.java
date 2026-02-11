/* Copyright (c) 2021 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Config
@TeleOp(name="OpMode 24-25", group="Linear OpMode")
public class OpMode2526 extends OpMode {
    private final Hardware hardware = new Hardware();

    private OpMode opMode;

    private static final double STRAFE_POWER = 0.50;
    private double prevLPower = 0.0;
    private double prevRPower = 0.0;

    private final boolean isAccelDriveMode = false;

    private RobotConfiguration robotConfiguration = null;

    private boolean isRed = false;
    private boolean isLeftStartingPos = false;

    private final boolean isPreviousManualDrive = false;
    private final boolean isCurrentManualDrive = false;

    private double startTime = 0;
    private double currentGasPedalPower = 1.0;
    private double shooterPower = 1.0;

    private boolean manualControls;
    private boolean reverseControls = false;
    private Pose2d startPose = null;



//    private int liftTargetPosition = 0;
//    private boolean liftManualMode = false;
//    private int liftPreviousManualPosition = LinearLift.LIFT_MINPOS;
//    private boolean manualIntake = false;

    @Override
    public void init() {
        System.gc();

        hardware.init(hardwareMap, this);

        hardware.drive = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));

        robotConfiguration = new RobotConfiguration();
        robotConfiguration.readConfig();
        isRed = robotConfiguration.isRed;
        isLeftStartingPos = robotConfiguration.isFarStartPos;
        manualControls = false;

        double startingAngle = 90;

        startPose = new Pose2d(0,0,Math.toRadians(0));
        telemetry.addLine("Configuration Fetched");
//        hardware.drive.setPoseEstimate(startPose);
//        hardware.odom.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 90));
//        telemetry.addData("Is Red?? ", isRed);
//        telemetry.addData("Is Left Position? ", isLeftStartingPos);
        telemetry.update();

//        startPose = new Pose2d(0, 0);
    }

    @Override
    public void init_loop() {
        hardware.updateValues();

        super.init_loop();

        hardware.init_loop();

    }

    @Override
    public void loop() {
        double targetLPower = 0.0;
        double targetRPower = 0.0;
        double desiredLPower = 0.0;
        double desiredRPower = 0.0;
        double targetLiftPower = 0.0;
        double desiredLiftPower = 0.0;
        double targetArmPower = 0.0;
        double desiredArmPower = 0.0;
        double targetSpinPower = 0.0;
        float game2LeftY = hardware.gamepad2_current_left_stick_y;
        float game2RightY = hardware.gamepad2_current_right_stick_y;
        float game1LeftY = hardware.gamepad1_current_left_stick_y;
        float game1LeftX = hardware.gamepad1_current_left_stick_x;
        float game1RightY = hardware.gamepad1_current_right_stick_y;
        float game1RightX = hardware.gamepad1_current_right_stick_x;
        double deltaExtension;
        double servoPower = 0;
        double currentGasPedal = 1.0;

        hardware.updateValues();

        Pose2d poseEstimate = hardware.drive.localizer.getPose();

        //GAMEPAD_1
        //Gas Pedal
        if (hardware.gamepad1_current_right_trigger < 0.05) {
            currentGasPedalPower = 1.0;
        } else if (hardware.gamepad1_current_right_trigger > 0.5) {
            currentGasPedalPower = (Math.max(1.0 - hardware.gamepad1_current_right_trigger, 0.6));
        }

        hardware.drive.setDrivePowers(new PoseVelocity2d(
                new Vector2d(
                        -gamepad1.left_stick_y * currentGasPedalPower,
                        -gamepad1.left_stick_x * currentGasPedalPower
                ),
                -gamepad1.right_stick_x * currentGasPedalPower
        ));

        if(hardware.gamepad2_current_y && !hardware.gamepad2_previous_y){
            manualControls = !manualControls;
        }

        //Manual Controls
        if(manualControls){
            if (hardware.gamepad2_current_right_trigger > 0.05) {
                hardware.artifactIntake.setPower(1);
            } else if (hardware.gamepad2_current_right_bumper) {
                hardware.artifactIntake.setPower(-0.75);
            } else if (hardware.gamepad2_current_y && !hardware.gamepad2_previous_y) {
                hardware.artifactIntake.singleShoot(0.9);
            } else if (hardware.artifactIntake.getState() == ArtifactIntake.INTAKING) {
                hardware.artifactIntake.noRotation();
            }

            if (hardware.gamepad2_current_left_trigger > 0.05) {
                hardware.shooter.setPower(shooterPower);
            } else {
                hardware.shooter.stop();
            }
            //0.7 1st 0.45 2nd
            if (hardware.gamepad2_current_b && !hardware.gamepad2_previous_b) {
                hardware.shooter.setPower(0.7);
            }
            if (hardware.gamepad2_current_a && !hardware.gamepad2_previous_a) {
                hardware.shooter.setPower(0.45);
            }

            if (hardware.gamepad2_current_dpad_up && !hardware.gamepad2_previous_dpad_up) {
                if (shooterPower < 1) {
                    shooterPower += 0.1;
                }
            }

            if (hardware.gamepad2_current_dpad_right && !hardware.gamepad2_previous_dpad_right) {
                if (shooterPower < 1) {
                    shooterPower += 0.05;
                }
            }

            if (hardware.gamepad2_current_dpad_down && !hardware.gamepad2_previous_dpad_down) {
                if (shooterPower-0.1 >= 0) {
                    shooterPower -= 0.1;
                }
            }

            if (hardware.gamepad2_current_x && !hardware.gamepad2_previous_x) {
                if (hardware.servoGate.isOpen) {
                    hardware.servoGate.closeGate();
                } else {
                    hardware.servoGate.openGate();

                }
            }
        }

        if (!manualControls){
            if(hardware.gamepad2_current_left_bumper && !hardware.gamepad2_previous_left_bumper){
                if(hardware.artifactIntake.getState() == ArtifactIntake.STATIONARY){
                    hardware.robo130.addCommand(new RCArtifactIntake(this.hardware, 1, true));
                }else{
                    hardware.robo130.addCommand(new RCArtifactIntake(this.hardware, 0, true));
                    hardware.robo130.addCommand(new RCOuttake(this.hardware, 1, true));
                }
            }

            if(hardware.gamepad2_current_right_bumper && !hardware.gamepad2_previous_right_bumper){
                if(hardware.shooter.getState() == Shooter.INACTIVEOUTTAKE){
                    hardware.robo130.addCommand(new RCOuttake(this.hardware, 1, true));
                }else{
                    hardware.robo130.addCommand(new RCOuttake(this.hardware, 0));
                }
            }

            if(hardware.gamepad2_current_left_bumper && !hardware.gamepad2_previous_left_bumper){
                if(hardware.artifactIntake.getState() == ArtifactIntake.STATIONARY){
                    hardware.robo130.addCommand(new RCArtifactIntake(this.hardware, 1, true));
                }else{
                    hardware.robo130.addCommand(new RCArtifactIntake(this.hardware, 0, true));
                    hardware.robo130.addCommand(new RCOuttake(this.hardware, 1, true));
                }
            }

            if(hardware.gamepad2_current_x && !hardware.gamepad2_previous_x){
                hardware.robo130.addCommand(new RCArtifactIntake(this.hardware, 0, true));
                hardware.robo130.addCommand(new RCServoGate(this.hardware,RCServoGate.CMD_OPEN,false));
                hardware.robo130.addCommand(new RCWait(this.hardware, 0.25));
                hardware.robo130.addCommand(new RCArtifactIntake(this.hardware, 1, true));
                hardware.robo130.addCommand(new RCWait(this.hardware, 0.15));
                hardware.robo130.addCommand(new RCArtifactIntake(this.hardware, 0, true));
                hardware.robo130.addCommand(new RCWait(this.hardware, 1));
                hardware.robo130.addCommand(new RCArtifactIntake(this.hardware, 1, true));
                hardware.robo130.addCommand(new RCWait(this.hardware, 0.5));
                hardware.robo130.addCommand(new RCServoGate(this.hardware,RCServoGate.CMD_CLOSE,false));
                hardware.robo130.addCommand(new RCArtifactIntake(this.hardware, 0, true));
                hardware.robo130.addCommand(new RCOuttake(this.hardware, 0, true));
            }

            if(hardware.gamepad2_current_b && !hardware.gamepad2_previous_b){
                hardware.robo130.addCommand(new RCArtifactIntake(this.hardware, -0.5, true));
                hardware.robo130.addCommand(new RCWait(this.hardware, 0.25));
                hardware.robo130.addCommand(new RCArtifactIntake(this.hardware, 0, true));
            }
        }

        hardware.robo130.processCommands();

        hardware.loop();

        prevLPower = targetLPower;
        prevRPower = targetRPower;

//        telemetry.addData("Front Distance", hardware.frontDistance.getDistance(DistanceUnit.INCH));
//        telemetry.addData("Rear Distance", hardware.rearDistance.getDistance(DistanceUnit.INCH));
        telemetry.addData("Delta Time", hardware.getDeltaTime());
        telemetry.addData("Robot Command Stack: ", Integer.toString(hardware.robo130.robotCommandStack.getNumCommands())
                + " " + Integer.toString(hardware.robo130.robotCommandStack.getCurrentCommandIndex())
                + " " + Integer.toString(hardware.robo130.robotCommandStack.getNextCommandIndex()));
//        telemetry.addData("Roadrunner Command Stack: ", Integer.toString(hardware.robo130.roadrunnerCommandStack.getNumCommands())
//                + " " + Integer.toString(hardware.robo130.roadrunnerCommandStack.getCurrentCommandIndex())
//                + " " + Integer.toString(hardware.robo130.roadrunnerCommandStack.getNextCommandIndex()));
        telemetry.addData("Status", "Running");
        telemetry.addData("Shooting power", shooterPower);
        telemetry.addData("In Manual Mode?", manualControls);

//        telemetry.addData("X position: ", hardware.odom.getPosX() / 25.4);
//        telemetry.addData("Y position: ", hardware.odom.getPosY() / 25.4);
//        telemetry.addData("Heading: ", hardware.odom.getHeading());
//        telemetry.addData("Intake state: ", hardware.sampleIntake.getStateString());
//        telemetry.addData("Specimen state: ", hardware.specimenClaw.getStateString());
        telemetry.update();
    }

    @Override
    public void stop() {
        hardware.updateValues();
        hardware.logMessage(false, "OpMode2223", "Stop Button Pressed");
        hardware.stop();
        super.stop();
    }

    @Override
    public void start() {
        hardware.updateValues();
        hardware.logMessage(false, "OpMode2223", "Start Button Pressed");
        super.start();
        hardware.start();
    }
    }