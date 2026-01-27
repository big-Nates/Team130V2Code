package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Specimen Claw Test", group = "tests")
//@Disabled
public class ServoGateTest extends OpMode {
    // Declare OpMode members.
    private final ElapsedTime runtime = new ElapsedTime();
    private final Hardware hardware = new Hardware();
    private double rightGateServoPosition = 0.5;
    private double leftGateServoPosition = 0.5;

    private Servo rightGateServo = null;
    private Servo leftGateServo = null;

    private double positionIncrement = .1;

    private double leftPositionIncrement = .1;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initializing");
        hardware.init(hardwareMap, this);

        rightGateServo = hardware.gateServo;
        leftGateServo = hardware.leftGateServo;

        telemetry.addData("Specimen Claw Servo Position", rightGateServoPosition);
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
        hardware.updateValues();

        super.init_loop();

        hardware.init_loop();
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
        hardware.updateValues();

        hardware.logMessage(false, "MyFirstJava", "Start Button Pressed");
        super.start();
        hardware.start();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        hardware.updateValues();

        if (hardware.gamepad1_current_x) {
            positionIncrement = 0.001;
        }
        if (hardware.gamepad1_current_y) {
            positionIncrement = .01;
        }
        if (hardware.gamepad1_current_b) {
            positionIncrement = .1;
        }

        if (hardware.gamepad2_current_x) {
            leftPositionIncrement = 0.001;
        }
        if (hardware.gamepad2_current_y) {
            leftPositionIncrement = .01;
        }
        if (hardware.gamepad2_current_b) {
            leftPositionIncrement = .1;
        }

        if (hardware.gamepad1_current_dpad_up & !hardware.gamepad1_previous_dpad_up) {
            rightGateServoPosition = Math.min(Math.max((rightGateServoPosition + positionIncrement), 0.0), 1.0);
        } else if (hardware.gamepad1_current_dpad_down & !hardware.gamepad1_previous_dpad_down) {
            rightGateServoPosition = Math.min(Math.max((rightGateServoPosition - positionIncrement), 0.0), 1.0);
        }

        if (hardware.gamepad2_current_dpad_up & !hardware.gamepad2_previous_dpad_up) {
            leftGateServoPosition = Math.min(Math.max((leftGateServoPosition + leftPositionIncrement), 0.0), 1.0);
        } else if (hardware.gamepad2_current_dpad_down & !hardware.gamepad2_previous_dpad_down) {
            leftGateServoPosition = Math.min(Math.max((leftGateServoPosition - leftPositionIncrement), 0.0), 1.0);
        }
        hardware.servoGate.setRightServoPosition(rightGateServoPosition);
        hardware.servoGate.setLeftServoPosition(leftGateServoPosition);

        hardware.loop();

        telemetry.addData("Status", "Run Time: " + runtime);
        telemetry.addData("Specimen Claw Servo Position", rightGateServoPosition);
        telemetry.addData("Specimen Claw Servo Hardware Position", rightGateServo.getPosition());
        telemetry.addData("Specimen Claw Servo Position", leftGateServoPosition);
        telemetry.addData("Specimen Claw Servo Hardware Position", leftGateServo.getPosition());
        telemetry.addData("Specimen Claw Servo increment", positionIncrement);
        telemetry.update();
    }


    @Override
    public void stop() {
        hardware.updateValues();

        hardware.logMessage(false, "MyFirstJava", "Stop Button Pressed");
        hardware.stop();
        super.stop();
    }
}