package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Config
@Autonomous(name = "AutonomousMovements", group = "auto")
public class AutonomousMovements extends OpMode {
    private final Hardware hardware = new Hardware();
    private MecanumDrive drive;
    private RobotConfiguration robotConfiguration = null;
    private boolean isRed = false;
    private boolean isFarStartingPos = false;
    private boolean doParking = false;
    private boolean commandsGrabbed = false;


    private Pose2d startPose;


    @Override
    public void init() {
        System.gc();
        hardware.init(hardwareMap, this);
        robotConfiguration = new RobotConfiguration();
        robotConfiguration.readConfig();
        isRed = robotConfiguration.isRed;
        isFarStartingPos = robotConfiguration.isFarStartPos;
        doParking = robotConfiguration.doParking;

        telemetry.addLine("Configuration Fetched");
        telemetry.addData("Is Red?? ", isRed);
        telemetry.addData("Is Left Position? ", isFarStartingPos);
        telemetry.addData("Park?", doParking);
        telemetry.update();


        if(!isFarStartingPos && !isRed && doParking){
            //Close Blue
            startPose = new Pose2d(53,50, Math.toRadians(45));
            drive = new MecanumDrive(hardwareMap, startPose);
            drive.initialPose = startPose;
            hardware.drive = drive;
            hardware.gateServo.setPosition(ServoGate.CLOSE_POS);

            hardware.robo130.addCommand(new RCRoadrunner(drive.actionBuilder(new Pose2d(53,50, Math.toRadians(45)))
                    .strafeToLinearHeading(new Vector2d(25, 22), Math.toRadians(45))
                    .setTangent(Math.toRadians(30))
                    .splineToLinearHeading(new Pose2d(0, 24, Math.toRadians(90)), Math.toRadians(90))
                    .splineToSplineHeading(new Pose2d(0, 36, Math.toRadians(90)), Math.toRadians(90))
                    .setTangent(Math.toRadians(-90))
                    .splineToLinearHeading(new Pose2d(25, 22, Math.toRadians(45)), Math.toRadians(-120))

                    .setTangent(Math.toRadians(180))
                    .splineToLinearHeading(new Pose2d(-22, 24, Math.toRadians(90)), Math.toRadians(90))
                    .splineToSplineHeading(new Pose2d(-22, 36, Math.toRadians(90)), Math.toRadians(90))
                    .setTangent(Math.toRadians(-90))
                    .splineToLinearHeading(new Pose2d(25, 22, Math.toRadians(45)), Math.toRadians(180))

                    .setTangent(Math.toRadians(180))
                    .splineToLinearHeading(new Pose2d(-44, 24, Math.toRadians(90)), Math.toRadians(90))
                    .splineToSplineHeading(new Pose2d(-44, 36, Math.toRadians(90)), Math.toRadians(90))
                    .setTangent(Math.toRadians(-90))
                    .splineToLinearHeading(new Pose2d(25, 22, Math.toRadians(45)), Math.toRadians(180))
                    .strafeToLinearHeading(new Vector2d(-10, 24), Math.toRadians(90))


                    .build(), hardware));


        }
        else if(!isFarStartingPos && !isRed){
            //Close Blue
            startPose = new Pose2d(53,50, Math.toRadians(45));
            drive = new MecanumDrive(hardwareMap, startPose);
            drive.initialPose = startPose;
            hardware.drive = drive;
            hardware.gateServo.setPosition(ServoGate.CLOSE_POS);

            hardware.robo130.addCommand(new RCRoadrunner(drive.actionBuilder(new Pose2d(53,50, Math.toRadians(45)))
                    .strafeToLinearHeading(new Vector2d(25, 22), Math.toRadians(45))

                    .setTangent(Math.toRadians(180))
                    .splineToSplineHeading(new Pose2d(-22, 24, Math.toRadians(90)), Math.toRadians(90))
                    .splineToConstantHeading(new Vector2d(-22, 36), Math.toRadians(90))
                    .setTangent(0)
                    .splineToConstantHeading(new Vector2d(-12, 42), Math.toRadians(90))
                    .setTangent(Math.toRadians(-90))
                    .splineToSplineHeading(new Pose2d(25, 22, Math.toRadians(45)), Math.toRadians(180))

                    .setTangent(Math.toRadians(30))
                    .splineToSplineHeading(new Pose2d(0, 24, Math.toRadians(90)), Math.toRadians(90))
                    .splineToConstantHeading(new Vector2d(0, 36), Math.toRadians(90))
                    .setTangent(Math.toRadians(-90))
                    .splineToSplineHeading(new Pose2d(25, 22, Math.toRadians(45)), Math.toRadians(-120))




                    .setTangent(Math.toRadians(180))
                    .splineToSplineHeading(new Pose2d(-44, 24, Math.toRadians(90)), Math.toRadians(90))
                    .splineToConstantHeading(new Vector2d(-44, 36), Math.toRadians(90))
                    .setTangent(Math.toRadians(-90))
                    .splineToSplineHeading(new Pose2d(25, 22, Math.toRadians(45)), Math.toRadians(180))
                    .strafeToLinearHeading(new Vector2d(-10, 24), Math.toRadians(90))




                    .build(), hardware));

        }
        else if(!isFarStartingPos){

        }

        commandsGrabbed = true;
        telemetry.addData("Trajectory Creation", "Is complete");
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
        hardware.loop();
        if (commandsGrabbed) {
            hardware.robo130.processCommands();
        }

        telemetry.addData("x", drive.localizer.getPose().position.x);
        telemetry.addData("y", drive.localizer.getPose().position.y);
        telemetry.addData("Robot Command Stick: ", Integer.toString(hardware.robo130.robotCommandStack.getNumCommands())
                + " " + Integer.toString(hardware.robo130.robotCommandStack.getCurrentCommandIndex())
                + " " + Integer.toString(hardware.robo130.robotCommandStack.getNextCommandIndex()));

        telemetry.addData("Roadrunner Command Stick: ", Integer.toString(hardware.robo130.roadrunnerCommandStack.getNumCommands())
                + " " + Integer.toString(hardware.robo130.roadrunnerCommandStack.getCurrentCommandIndex())
                + " " + Integer.toString(hardware.robo130.roadrunnerCommandStack.getNextCommandIndex()));
//        telemetry.addData("Roadrunner Command Stick: ", Integer.toString(hardware.robo130.roadrunnerCommandStack.getNumActions())
//                + " " + Integer.toString(hardware.robo130.roadrunnerCommandStack.getCurrentActionIndex())
//                + " " + Integer.toString(hardware.robo130.roadrunnerCommandStack.getNextActionIndex()));
        telemetry.addData("Status", "Running");
        telemetry.update();
    }

    public void start() {
//        hardware.webcamPipeline.saveProcessedImages();
//        hardware.webcam.stopStreaming();
        hardware.updateValues();
        hardware.logMessage(false, "Auto2324", "Start Button Pressed");
        super.start();
        hardware.start();

    }

    public void stop() {
        hardware.updateValues();
        hardware.logMessage(false, "Auto2324", "Stop Button Pressed");
        hardware.stop();
        super.stop();
    }
}

