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
@TeleOp(name="Solo OpMode 25-26", group="Linear OpMode")
public class SoloOpMode2526 extends OpMode {
    private final Hardware hardware = new Hardware();


    private RobotConfiguration robotConfiguration = null;

    private boolean isRed = false;
    private boolean isFarStartingPos = false;

    private final boolean isCurrentManualDrive = false;
    private double currentGasPedalPower = 1.0;
    private double shooterPower = 1.0;
    private int intakeState;
    private int outtakeState;
    private Pose2d startPose = null;



//    private int liftTargetPosition = 0;
//    private boolean liftManualMode = false;
//    private int liftPreviousManualPosition = LinearLift.LIFT_MINPOS;
//    private boolean manualIntake = false;

    @Override
    public void init() {
        System.gc();

        hardware.init(hardwareMap, this);

        robotConfiguration = new RobotConfiguration();
        robotConfiguration.readConfig();
        isRed = robotConfiguration.isRed;
        isFarStartingPos = robotConfiguration.isFarStartPos;

        if(!isFarStartingPos && !isRed){
            //Close Blue
            startPose = new Pose2d(-5,24, Math.toRadians(90));
        }
        else if(!isFarStartingPos){
            //Close Red
            startPose = new Pose2d(-5,-24, Math.toRadians(90));
        }

        hardware.drive = new MecanumDrive(hardwareMap, startPose);
        telemetry.addLine("Configuration Fetched");
        intakeState = hardware.artifactIntake.getState();
        outtakeState = hardware.shooter.getState();
        telemetry.update();

    }

    @Override
    public void init_loop() {
        hardware.updateValues();

        super.init_loop();

        hardware.init_loop();

    }

    @Override
    public void loop() {
        hardware.updateValues();
        Pose2d poseEstimate = hardware.drive.localizer.getPose();

        //GAMEPAD_1
        //Gas Pedal
        if (hardware.gamepad1_current_left_trigger < 0.05 && hardware.gamepad1_current_right_trigger < 0.05) {
            currentGasPedalPower = 1.0;
        } else if (hardware.gamepad1_current_left_trigger > 0.5) {
            currentGasPedalPower = (Math.max(1.0 - hardware.gamepad1_current_left_trigger, 0.3));
        } else if (hardware.gamepad1_current_right_trigger > 0.5) {
            currentGasPedalPower = (Math.max(1.0 - hardware.gamepad1_current_right_trigger, 0.6));
        }

        if(hardware.gamepad1_current_a && !hardware.gamepad1_previous_a){
            hardware.robo130.addCommand(new RCRoadrunner1(this.hardware.drive.actionBuilder(hardware.drive.localizer.getPose())
                    .strafeToConstantHeading(new Vector2d(0, 0))
                    .build(), hardware));
        }

        hardware.drive.setDrivePowers(new PoseVelocity2d(
                new Vector2d(
                        -gamepad1.left_stick_y * currentGasPedalPower,
                        -gamepad1.left_stick_x * currentGasPedalPower
                ),
                -gamepad1.right_stick_x * currentGasPedalPower
        ));

        if(hardware.gamepad1_current_right_bumper && !hardware.gamepad1_previous_right_bumper){
            if(hardware.shooter.getState() == Shooter.INACTIVEOUTTAKE){
                hardware.robo130.addCommand(new RCOuttake(this.hardware, 1.0));
            }else{
                hardware.robo130.addCommand(new RCOuttake(this.hardware, 0));
            }
        }

        if(hardware.gamepad1_current_left_bumper && !hardware.gamepad1_previous_left_bumper){
            if(hardware.artifactIntake.getState() == ArtifactIntake.STATIONARY){
                hardware.robo130.addCommand(new RCArtifactIntake(this.hardware, 1, 1));
                hardware.robo130.addCommand(new RCArtifactIntake(this.hardware, 0.75, 2));
            }else{
                hardware.robo130.addCommand(new RCArtifactIntake(this.hardware, 0, true));
                hardware.robo130.addCommand(new RCOuttake(this.hardware, 1.0));
            }
        }

        if(hardware.gamepad1_current_x && !hardware.gamepad1_previous_x){
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

        if(hardware.gamepad1_current_b && !hardware.gamepad1_previous_b){
            hardware.robo130.addCommand(new RCArtifactIntake(this.hardware, -0.5, true));
            hardware.robo130.addCommand(new RCWait(this.hardware, 0.5));
            hardware.robo130.addCommand(new RCArtifactIntake(this.hardware, 0, true));
        }





        hardware.robo130.processCommands();
        hardware.loop();
        telemetry.addData("Robot Command Stack: ", (hardware.robo130.robotCommandStack.getNumCommands()
                + " " + hardware.robo130.robotCommandStack.getCurrentCommandIndex()));
        telemetry.addData("Intake State", hardware.artifactIntake.getState());
        telemetry.addData("Status", "Running");
        telemetry.addData("Shooting power", shooterPower);
        telemetry.addData("Current Position", ("X: "+poseEstimate.position.y+" Y: "+poseEstimate.position.x+" Heading: "+poseEstimate.heading.toDouble()));
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