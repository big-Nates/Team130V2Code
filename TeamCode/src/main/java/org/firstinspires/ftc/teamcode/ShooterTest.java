package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Outtake Test", group = "tests")
//@Disabled
public class ShooterTest extends OpMode {
    // Declare OpMode members.
    private final ElapsedTime runtime = new ElapsedTime();
    private final Hardware hardware = new Hardware();

    private DcMotorEx outtakeMotor;

    private int currentVelocity = 0;
    private int velocityIncrement = 10;



    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initializing");
        hardware.init(hardwareMap, this);

        outtakeMotor = hardware.shooterFlyWhell;

        telemetry.addData("Specimen Claw Servo Position", currentVelocity);
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
        hardware.servoGate.openGate();
        hardware.logMessage(false, "ShooterTest", "Start Button Pressed");
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
            velocityIncrement = 10;
        }
        if (hardware.gamepad1_current_y) {
            velocityIncrement = 100;
        }
        if (hardware.gamepad1_current_b) {
            velocityIncrement = 1000;
        }
        if(hardware.gamepad1_current_right_trigger > 0.05){
            hardware.artifactIntake.setPower(1);
        }

        if (hardware.gamepad1_current_dpad_up & !hardware.gamepad1_previous_dpad_up) {
            currentVelocity = Math.min(Math.max((currentVelocity + velocityIncrement), 0), 10000);
        } else if (hardware.gamepad1_current_dpad_down & !hardware.gamepad1_previous_dpad_down) {
            currentVelocity = Math.min(Math.max((currentVelocity - velocityIncrement), 0), 10000);
        }


        outtakeMotor.setVelocity(currentVelocity);

        hardware.loop();

        telemetry.addData("Status", "Run Time: " + runtime);
        telemetry.addData("Current velocity", currentVelocity);
        telemetry.addData("Velocity Increment", velocityIncrement);
        telemetry.update();
    }


    @Override
    public void stop() {
        hardware.updateValues();
        hardware.logMessage(false, "ShooterTest", "Stop Button Pressed");
        hardware.stop();
        super.stop();
    }
}