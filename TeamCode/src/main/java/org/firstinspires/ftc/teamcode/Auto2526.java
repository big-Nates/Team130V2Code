package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Config
@Autonomous(name = "Auto2526", group = "auto")
public class Auto2526 extends OpMode {
    private final Hardware hardware = new Hardware();
    private MecanumDrive drive;
    private RobotConfiguration robotConfiguration = null;
    private boolean isRed = false;
    private boolean isFarStartingPos = false;
    private boolean doParking = false;
    private int selectedSpikemark = -999;
    private int selectedTag = -999;
    private final int autoLiftPos = 670; //was originally 920
    private boolean SKIPCAMERA = true;
    private boolean commandsGrabbed = false;
    private final boolean firstRun = true;
    private final boolean secondRun = false;
    private final boolean testMode = false;
    private final boolean onlyPark = false;
    public double startTime = 0.0;

    private Pose2d startPose = null;


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
            //Sync Commands
            RobCommand gateCloseSync = new RCServoGate(this.hardware, RCServoGate.CMD_CLOSE, true);
            RobCommand firstRowCollected = new RCRoadrunner1(drive.actionBuilder(new Pose2d(0,35, Math.toRadians(90)))
                    .strafeToConstantHeading(new Vector2d(0, 55))
                    .build(), hardware);
            RobCommand secondRowCollected = new RCRoadrunner1(drive.actionBuilder(new Pose2d(-29,34, Math.toRadians(90)))
                    .strafeToConstantHeading(new Vector2d(-29, 57))
                    .build(), hardware);

        //Score Preloaded artifacts
            hardware.robo130.addCommand(new RCOuttake(this.hardware,0.9, true));
            hardware.robo130.addCommand(new RCRoadrunner1(drive.actionBuilder(new Pose2d(53,50, Math.toRadians(45)))
                    .strafeToLinearHeading(new Vector2d(17, 14), Math.toRadians(45))
                    .build(), hardware));
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

            //Collect First Row
            hardware.robo130.addCommand(new RCOuttake(this.hardware,1, true));
            hardware.robo130.addCommand(new RCRoadrunner1(drive.actionBuilder(new Pose2d(17,14, Math.toRadians(45)))
                    .strafeToLinearHeading(new Vector2d(0, 35), Math.toRadians(90))
                    .build(), hardware));
            hardware.robo130.addCommand(new RCArtifactIntake(this.hardware, 1, true));
            hardware.robo130.addCommand(firstRowCollected);
            hardware.robo130.addCommand(new RCArtifactIntake(this.hardware, 0, true));

            //Score First row
            hardware.robo130.addCommand(new RCOuttake(this.hardware,1, true));
            hardware.robo130.addCommand(new RCRoadrunner1(drive.actionBuilder(new Pose2d(0,55, Math.toRadians(90)))
                    .strafeToLinearHeading(new Vector2d(17, 14), Math.toRadians(45))
                    .build(), hardware));
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

            //Collect Second Row
            hardware.robo130.addCommand(new RCOuttake(this.hardware,1, true));
            hardware.robo130.addCommand(new RCRoadrunner1(drive.actionBuilder(new Pose2d(17,14, Math.toRadians(45)))
                    .strafeToLinearHeading(new Vector2d(-29, 34), Math.toRadians(90))
                    .build(), hardware));
            hardware.robo130.addCommand(new RCArtifactIntake(this.hardware, 1, true));
            hardware.robo130.addCommand(secondRowCollected);
            hardware.robo130.addCommand(new RCArtifactIntake(this.hardware, 0, true));

            //Score Second row
            hardware.robo130.addCommand(new RCOuttake(this.hardware,1, true));
            hardware.robo130.addCommand(new RCRoadrunner1(drive.actionBuilder(new Pose2d(-29,57, Math.toRadians(90)))
                    .strafeToLinearHeading(new Vector2d(28, 14), Math.toRadians(45))
                    .build(), hardware));
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

            //Leave
            hardware.robo130.addCommand(new RCRoadrunner1(drive.actionBuilder(new Pose2d(28,14, Math.toRadians(45)))
                    .strafeToLinearHeading(new Vector2d(-10, 24), Math.toRadians(90))
                    .build(), hardware));

        }

        else if(!isFarStartingPos && !isRed && !doParking){
            //Close Blue
            startPose = new Pose2d(53,50, Math.toRadians(45));
            drive = new MecanumDrive(hardwareMap, startPose);
            drive.initialPose = startPose;
            hardware.drive = drive;
            hardware.gateServo.setPosition(ServoGate.CLOSE_POS);
            //Sync Commands
            RobCommand firstRowCollected = new RCRoadrunner1(drive.actionBuilder(new Pose2d(-6,14, Math.toRadians(90)))
                    .strafeToConstantHeading(new Vector2d(-6, 55))
                    .build(), hardware);
            RobCommand secondRowCollected = new RCRoadrunner1(drive.actionBuilder(new Pose2d(-38,34, Math.toRadians(90)))
                    .strafeToConstantHeading(new Vector2d(-38, 57))
                    .build(), hardware);

            //Score Preloaded artifacts
            hardware.robo130.addCommand(new RCOuttake(this.hardware,0.9, true));
            hardware.robo130.addCommand(new RCRoadrunner1(drive.actionBuilder(new Pose2d(53,50, Math.toRadians(45)))
                    .strafeToLinearHeading(new Vector2d(17, 14), Math.toRadians(45))
                    .build(), hardware));
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
            hardware.robo130.addCommand(new RCServoGate(this.hardware, RCServoGate.CMD_CLOSE, true));

            //Collect Second Row
            hardware.robo130.addCommand(new RCOuttake(this.hardware,1, true));
            hardware.robo130.addCommand(new RCRoadrunner1(drive.actionBuilder(new Pose2d(17,14, Math.toRadians(45)))
                    .strafeToLinearHeading(new Vector2d(-38, 34), Math.toRadians(90))
                    .build(), hardware));
            hardware.robo130.addCommand(new RCArtifactIntake(this.hardware, 1, true));
            hardware.robo130.addCommand(secondRowCollected);
            hardware.robo130.addCommand(new RCArtifactIntake(this.hardware, 0, true));

            //Empty Classifier Gate
            hardware.robo130.addCommand(new RCRoadrunner1(drive.actionBuilder(new Pose2d(-38,57, Math.toRadians(90)))
                    .strafeToConstantHeading(new Vector2d(-38, 47))
                    .splineToLinearHeading(new Pose2d(-10, 78,  Math.toRadians(180)), Math.toRadians(90))
                    .build(), hardware));

            //Score Second row
            hardware.robo130.addCommand(new RCOuttake(this.hardware,1, true));
            hardware.robo130.addCommand(new RCRoadrunner1(drive.actionBuilder(new Pose2d(-12,75, Math.toRadians(180)))
                    .splineToLinearHeading(new Pose2d(-32, 34, Math.toRadians(45)), Math.toRadians(-90))
                    .strafeToConstantHeading(new Vector2d(28, 14))
                    .build(), hardware));
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
            hardware.robo130.addCommand(new RCServoGate(this.hardware, RCServoGate.CMD_CLOSE, true));

            //Collect First Row
            hardware.robo130.addCommand(new RCOuttake(this.hardware,1, true));
            hardware.robo130.addCommand(new RCRoadrunner1(drive.actionBuilder(new Pose2d(28,14, Math.toRadians(45)))
                    .strafeToLinearHeading(new Vector2d(-6, 14), Math.toRadians(90))
                    .build(), hardware));
            hardware.robo130.addCommand(new RCArtifactIntake(this.hardware, 1, true));
            hardware.robo130.addCommand(firstRowCollected);
            hardware.robo130.addCommand(new RCArtifactIntake(this.hardware, 0, true));

            //Score First row
            hardware.robo130.addCommand(new RCOuttake(this.hardware,1, true));
            hardware.robo130.addCommand(new RCRoadrunner1(drive.actionBuilder(new Pose2d(-6,55, Math.toRadians(90)))
                    .strafeToLinearHeading(new Vector2d(28, 14), Math.toRadians(45))
                    .build(), hardware));
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
            hardware.robo130.addCommand(new RCServoGate(this.hardware, RCServoGate.CMD_CLOSE, true));

            //Leave
            hardware.robo130.addCommand(new RCRoadrunner1(drive.actionBuilder(new Pose2d(28,14, Math.toRadians(45)))
                    .strafeToLinearHeading(new Vector2d(0, 24), Math.toRadians(90))
                    .build(), hardware));

        }
        else if(!isFarStartingPos){
            //Close Red
            startPose = new Pose2d(53,-50, Math.toRadians(-45));
            drive = new MecanumDrive(hardwareMap, startPose);
            drive.initialPose = startPose;
            hardware.drive = drive;
            hardware.gateServo.setPosition(ServoGate.CLOSE_POS);
            //Sync Commands
            RobCommand gateCloseSync = new RCServoGate(this.hardware, RCServoGate.CMD_CLOSE, true);
            RobCommand firstRowCollected = new RCRoadrunner1(drive.actionBuilder(drive.localizer.getPose())
                    .strafeToConstantHeading(new Vector2d(-5, -53))
                    .build(), hardware);
            RobCommand secondRowCollected = new RCRoadrunner1(drive.actionBuilder(drive.localizer.getPose())
                    .strafeToConstantHeading(new Vector2d(-38, -53))
                    .build(), hardware);

            //Score Preloaded artifacts
            hardware.robo130.addCommand(new RCOuttake(this.hardware,0.9, true));
            hardware.robo130.addCommand(new RCRoadrunner1(drive.actionBuilder(drive.localizer.getPose())
                    .strafeToLinearHeading(new Vector2d(17, -14), Math.toRadians(-45))
                    .build(), hardware));
            hardware.robo130.addCommand(new RCWait(this.hardware, 1.5)); //possibly too fast
            hardware.robo130.addCommand(new RCServoGate(this.hardware, RCServoGate.CMD_OPEN, false));
            hardware.robo130.addCommand(new RCArtifactIntake(this.hardware, 0.5, true));
            hardware.robo130.addCommand(new RCWait(this.hardware, 0.75));
            hardware.robo130.addCommand(new RCArtifactIntake(this.hardware, 0, true));
            hardware.robo130.addCommand(new RCWait(this.hardware, 1));
            hardware.robo130.addCommand(new RCArtifactIntake(this.hardware, 0.5, true));
            hardware.robo130.addCommand(new RCWait(this.hardware, 0.75));
            hardware.robo130.addCommand(new RCArtifactIntake(this.hardware, 0, true));
            hardware.robo130.addCommand(gateCloseSync);
            hardware.robo130.addCommand(new RCOuttake(this.hardware,0));

            //Collect First Row
            hardware.robo130.addCommand(new RCRoadrunner1(drive.actionBuilder(drive.localizer.getPose())
                    .strafeToLinearHeading(new Vector2d(-5, -29), Math.toRadians(-90))
                    .build(), hardware));
            hardware.robo130.addCommand(new RCArtifactIntake(this.hardware, 1, true));
            hardware.robo130.addCommand(firstRowCollected);
            hardware.robo130.addCommand(new RCOuttake(this.hardware, 1, true));
            hardware.robo130.addCommand(new RCArtifactIntake(this.hardware, 0, true));

            //Score First row
            hardware.robo130.addCommand(new RCRoadrunner1(drive.actionBuilder(drive.localizer.getPose())
                    .strafeToLinearHeading(new Vector2d(17, -14), Math.toRadians(-50))
                    .build(), hardware));
            hardware.robo130.addCommand(new RCServoGate(this.hardware, RCServoGate.CMD_OPEN, false));
            hardware.robo130.addCommand(new RCWait(this.hardware, 0.5));
            hardware.robo130.addCommand(new RCArtifactIntake(this.hardware, 0.5, false));
            hardware.robo130.addCommand(new RCWait(this.hardware, 0.75));
            hardware.robo130.addCommand(new RCArtifactIntake(this.hardware, 0, false));
            hardware.robo130.addCommand(new RCWait(this.hardware, 1.5));
            hardware.robo130.addCommand(new RCArtifactIntake(this.hardware, 0.5, false));
            hardware.robo130.addCommand(new RCWait(this.hardware, 0.75));
            hardware.robo130.addCommand(new RCArtifactIntake(this.hardware, 0, true));
            hardware.robo130.addCommand(gateCloseSync);
            hardware.robo130.addCommand(new RCOuttake(this.hardware,0));

            //Collect Second Row
            hardware.robo130.addCommand(new RCRoadrunner1(drive.actionBuilder(drive.localizer.getPose())
                    .strafeToLinearHeading(new Vector2d(-72, -45.5), Math.toRadians(-90))
                    .build(), hardware));
            hardware.robo130.addCommand(new RCArtifactIntake(this.hardware, 1, true));
            hardware.robo130.addCommand(secondRowCollected);
            hardware.robo130.addCommand(new RCOuttake(this.hardware,1));
            hardware.robo130.addCommand(new RCArtifactIntake(this.hardware, 0, true));

            //Score Second row
            hardware.robo130.addCommand(new RCRoadrunner1(drive.actionBuilder(drive.localizer.getPose())
                    .strafeToLinearHeading(new Vector2d(17, -14), Math.toRadians(-50))
                    .build(), hardware));
            hardware.robo130.addCommand(new RCServoGate(this.hardware, RCServoGate.CMD_OPEN, true));
            hardware.robo130.addCommand(new RCWait(this.hardware, 0.5));
            hardware.robo130.addCommand(new RCArtifactIntake(this.hardware, 0.5, false));
            hardware.robo130.addCommand(new RCWait(this.hardware, 0.75));
            hardware.robo130.addCommand(new RCArtifactIntake(this.hardware, 0, false));
            hardware.robo130.addCommand(new RCWait(this.hardware, 1.5));
            hardware.robo130.addCommand(new RCArtifactIntake(this.hardware, 0.5, false));
            hardware.robo130.addCommand(new RCWait(this.hardware, 0.75));
            hardware.robo130.addCommand(new RCArtifactIntake(this.hardware, 0, true));
            hardware.robo130.addCommand(gateCloseSync);
            hardware.robo130.addCommand(new RCOuttake(this.hardware,0));

            //Leave
            hardware.robo130.addCommand(new RCRoadrunner1(drive.actionBuilder(drive.localizer.getPose())
                    .strafeToLinearHeading(new Vector2d(0, -12), Math.toRadians(-90))
                    .build(), hardware));
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
        telemetry.addData("Selected Tag: ", selectedTag);
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

