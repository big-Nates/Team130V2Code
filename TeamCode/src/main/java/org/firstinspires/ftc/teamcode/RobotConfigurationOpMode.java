package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Robot Configuration", group = "A")

public class RobotConfigurationOpMode extends LinearOpMode {
    private boolean isRed = false;
    private boolean isFarStartPos = false;
    private boolean placeExtraPixels = false;

    private RobotConfiguration roboConfig = null;

    @Override
    public void runOpMode() {
        waitForStart();

        while (opModeIsActive()) {
            telemetry.addLine("Team Colour? RED (B) or BLUE (X) button.");
            telemetry.update();
            while (true) {
                if (gamepad1.x || gamepad2.x) {
                    isRed = false;
                    break;
                }
                if (gamepad1.b || gamepad2.b) {
                    isRed = true;
                    break;
                }
            }
            telemetry.addLine("Starting Position: Left Dpad for far, Right Dpad for close");
            telemetry.update();
            while (true) {
                if (gamepad1.dpad_left || gamepad2.dpad_left) {
                    isFarStartPos = true;
                    break;
                }
                if (gamepad1.dpad_right || gamepad2.dpad_right) {
                    isFarStartPos = false;
                    break;
                }
            }
            telemetry.addLine("Do you want to park? Dpad Up for yes Dpad down for no");
            telemetry.update();
            while (true) {
                if (gamepad1.dpad_up || gamepad2.dpad_up) {
                    placeExtraPixels = true;
                    break;
                }
                if (gamepad1.dpad_down || gamepad2.dpad_down) {
                    placeExtraPixels = false;
                    break;
                }
            }
            break;
        }

        roboConfig = new RobotConfiguration(isRed, isFarStartPos, placeExtraPixels);
        roboConfig.saveConfig();

        while (opModeIsActive()) {
            telemetry.addLine("Configuration Complete");
            telemetry.addData("Is Red?? ", isRed);
            telemetry.addData("Is Left Position? ", isFarStartPos);
            telemetry.update();
        }
    }
}