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

    private boolean commandsGrabbed = false;


    @Override
    public void init() {
        System.gc();
        hardware.init(hardwareMap, this);

        RobotConfiguration robotConfiguration = new RobotConfiguration();
        robotConfiguration.readConfig();
        boolean isRed = robotConfiguration.isRed;
        boolean doParking = robotConfiguration.doParking;
        boolean isFarStartingPos = robotConfiguration.isFarStartPos;
        Pose2d startPose;


        telemetry.addLine("Configuration Fetched");
        telemetry.addData("Is Red?? ", isRed);
        telemetry.addData("Is Left Position? ", isFarStartingPos);
        telemetry.addData("Park?", doParking);


        if(!isFarStartingPos && !isRed && doParking){
            //Close Blue
            startPose = new Pose2d(53,50, Math.toRadians(45));
            drive = new MecanumDrive(hardwareMap, startPose);
            drive.initialPose = startPose;
            hardware.drive = drive;
            hardware.gateServo.setPosition(ServoGate.CLOSE_POS);
            //Sync Commands
            RobCommand gateCloseSync = new RCServoGate(this.hardware, RCServoGate.CMD_CLOSE, true);
            RobCommand firstRowCollected = new RCRoadrunner(drive.actionBuilder(new Pose2d(0,35, Math.toRadians(90)))
                    .strafeToConstantHeading(new Vector2d(0, 55))
                    .build(), hardware);
            RobCommand secondRowCollected = new RCRoadrunner(drive.actionBuilder(new Pose2d(-36,34, Math.toRadians(90)))
                    .strafeToConstantHeading(new Vector2d(-36, 57))
                    .build(), hardware);
            RobCommand thirdRowCollected = new RCRoadrunner(drive.actionBuilder(new Pose2d(-78,34, Math.toRadians(90)))
                    .strafeToConstantHeading(new Vector2d(-78, 57))
                    .build(), hardware);

            //Score Preloaded artifacts
            hardware.robo130.addCommand(new RCOuttake(this.hardware,1, true));
            hardware.robo130.addCommand(new RCRoadrunner(drive.actionBuilder(new Pose2d(53,50, Math.toRadians(45)))
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

            //Collect First Row
            hardware.robo130.addCommand(new RCRoadrunner(drive.actionBuilder(new Pose2d(17,14, Math.toRadians(45)))
                    .strafeToLinearHeading(new Vector2d(0, 35), Math.toRadians(90))
                    .build(), hardware));
            hardware.robo130.addCommand(new RCArtifactIntake(this.hardware, 1, true));
            hardware.robo130.addCommand(firstRowCollected);
            hardware.robo130.addCommand(new RCArtifactIntake(this.hardware, 0, true));

            //Score First row
            hardware.robo130.addCommand(new RCRoadrunner(drive.actionBuilder(new Pose2d(0,55, Math.toRadians(90)))
                    .strafeToLinearHeading(new Vector2d(23, 14), Math.toRadians(45))
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

            //Collect Second Row
            hardware.robo130.addCommand(new RCRoadrunner(drive.actionBuilder(new Pose2d(23,14, Math.toRadians(45)))
                    .strafeToLinearHeading(new Vector2d(-36, 34), Math.toRadians(90))
                    .build(), hardware));
            hardware.robo130.addCommand(new RCArtifactIntake(this.hardware, 1, true));
            hardware.robo130.addCommand(secondRowCollected);
            hardware.robo130.addCommand(new RCArtifactIntake(this.hardware, 0, true));

            //Score Second row
            hardware.robo130.addCommand(new RCRoadrunner(drive.actionBuilder(new Pose2d(-36,57, Math.toRadians(90)))
                    .strafeToLinearHeading(new Vector2d(28, 4), Math.toRadians(45))
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

            //Collect Third Row
            hardware.robo130.addCommand(new RCRoadrunner(drive.actionBuilder(new Pose2d(34,10, Math.toRadians(45)))
                    .strafeToLinearHeading(new Vector2d(-78, 34), Math.toRadians(90))
                    .build(), hardware));
            hardware.robo130.addCommand(new RCArtifactIntake(this.hardware, 1, true));
            hardware.robo130.addCommand(thirdRowCollected);
            hardware.robo130.addCommand(new RCArtifactIntake(this.hardware, 0, true));

            //Leave
            hardware.robo130.addCommand(new RCRoadrunner(drive.actionBuilder(new Pose2d(-78,57, Math.toRadians(90)))
                    .strafeToConstantHeading(new Vector2d(-30, 57))
                    .build(), hardware));




        }
        else if(!isFarStartingPos && !isRed){
            //Close Blue
//            startPose = new Pose2d(53,50, Math.toRadians(45));
//            drive = new MecanumDrive(hardwareMap, startPose);
//            drive.initialPose = startPose;
//            hardware.drive = drive;
//            hardware.gateServo.setPosition(ServoGate.CLOSE_POS);
//            //Sync Commands
//            RobCommand firstRowCollected = new RCRoadrunner(drive.actionBuilder(new Pose2d(-6,14, Math.toRadians(90)))
//                    .strafeToConstantHeading(new Vector2d(-6, 55))
//                    .build(), hardware);
//            RobCommand secondRowCollected = new RCRoadrunner(drive.actionBuilder(new Pose2d(-38,34, Math.toRadians(90)))
//                    .strafeToConstantHeading(new Vector2d(-38, 57))
//                    .build(), hardware);
//
//            //Score Preloaded artifacts
//            hardware.robo130.addCommand(new RCOuttake(this.hardware,0.9, true));
//            hardware.robo130.addCommand(new RCRoadrunner(drive.actionBuilder(new Pose2d(53,50, Math.toRadians(45)))
//                    .strafeToLinearHeading(new Vector2d(17, 14), Math.toRadians(45))
//                    .build(), hardware));
//            hardware.robo130.addCommand(new RCArtifactIntake(this.hardware, 0, true));
//            hardware.robo130.addCommand(new RCServoGate(this.hardware,RCServoGate.CMD_OPEN,false));
//            hardware.robo130.addCommand(new RCWait(this.hardware, 0.25));
//            hardware.robo130.addCommand(new RCArtifactIntake(this.hardware, 1, true));
//            hardware.robo130.addCommand(new RCWait(this.hardware, 0.15));
//            hardware.robo130.addCommand(new RCArtifactIntake(this.hardware, 0, true));
//            hardware.robo130.addCommand(new RCWait(this.hardware, 1));
//            hardware.robo130.addCommand(new RCArtifactIntake(this.hardware, 1, true));
//            hardware.robo130.addCommand(new RCWait(this.hardware, 0.5));
//            hardware.robo130.addCommand(new RCServoGate(this.hardware,RCServoGate.CMD_CLOSE,false));
//            hardware.robo130.addCommand(new RCArtifactIntake(this.hardware, 0, true));
//            hardware.robo130.addCommand(new RCOuttake(this.hardware, 0, true));
//            hardware.robo130.addCommand(new RCServoGate(this.hardware, RCServoGate.CMD_CLOSE, true));
//
//            //Collect Second Row
//            hardware.robo130.addCommand(new RCOuttake(this.hardware,1, true));
//            hardware.robo130.addCommand(new RCRoadrunner(drive.actionBuilder(new Pose2d(17,14, Math.toRadians(45)))
//                    .strafeToLinearHeading(new Vector2d(-38, 34), Math.toRadians(90))
//                    .build(), hardware));
//            hardware.robo130.addCommand(new RCArtifactIntake(this.hardware, 1, true));
//            hardware.robo130.addCommand(secondRowCollected);
//            hardware.robo130.addCommand(new RCArtifactIntake(this.hardware, 0, true));
//
//            //Empty Classifier Gate
//            hardware.robo130.addCommand(new RCRoadrunner(drive.actionBuilder(new Pose2d(-38,57, Math.toRadians(90)))
//                    .strafeToConstantHeading(new Vector2d(-38, 47))
//                    .splineToLinearHeading(new Pose2d(-10, 78,  Math.toRadians(180)), Math.toRadians(90))
//                    .build(), hardware));
//
//            //Score Second row
//            hardware.robo130.addCommand(new RCOuttake(this.hardware,1, true));
//            hardware.robo130.addCommand(new RCRoadrunner(drive.actionBuilder(new Pose2d(-12,75, Math.toRadians(180)))
//                    .splineToLinearHeading(new Pose2d(-32, 34, Math.toRadians(45)), Math.toRadians(-90))
//                    .strafeToConstantHeading(new Vector2d(28, 14))
//                    .build(), hardware));
//            hardware.robo130.addCommand(new RCArtifactIntake(this.hardware, 0, true));
//            hardware.robo130.addCommand(new RCServoGate(this.hardware,RCServoGate.CMD_OPEN,false));
//            hardware.robo130.addCommand(new RCWait(this.hardware, 0.25));
//            hardware.robo130.addCommand(new RCArtifactIntake(this.hardware, 1, true));
//            hardware.robo130.addCommand(new RCWait(this.hardware, 0.15));
//            hardware.robo130.addCommand(new RCArtifactIntake(this.hardware, 0, true));
//            hardware.robo130.addCommand(new RCWait(this.hardware, 1));
//            hardware.robo130.addCommand(new RCArtifactIntake(this.hardware, 1, true));
//            hardware.robo130.addCommand(new RCWait(this.hardware, 0.5));
//            hardware.robo130.addCommand(new RCServoGate(this.hardware,RCServoGate.CMD_CLOSE,false));
//            hardware.robo130.addCommand(new RCArtifactIntake(this.hardware, 0, true));
//            hardware.robo130.addCommand(new RCOuttake(this.hardware, 0, true));
//            hardware.robo130.addCommand(new RCServoGate(this.hardware, RCServoGate.CMD_CLOSE, true));
//
//            //Collect First Row
//            hardware.robo130.addCommand(new RCOuttake(this.hardware,1, true));
//            hardware.robo130.addCommand(new RCRoadrunner(drive.actionBuilder(new Pose2d(28,14, Math.toRadians(45)))
//                    .strafeToLinearHeading(new Vector2d(-6, 14), Math.toRadians(90))
//                    .build(), hardware));
//            hardware.robo130.addCommand(new RCArtifactIntake(this.hardware, 1, true));
//            hardware.robo130.addCommand(firstRowCollected);
//            hardware.robo130.addCommand(new RCArtifactIntake(this.hardware, 0, true));
//
//            //Score First row
//            hardware.robo130.addCommand(new RCOuttake(this.hardware,1, true));
//            hardware.robo130.addCommand(new RCRoadrunner(drive.actionBuilder(new Pose2d(-6,55, Math.toRadians(90)))
//                    .strafeToLinearHeading(new Vector2d(28, 14), Math.toRadians(45))
//                    .build(), hardware));
//            hardware.robo130.addCommand(new RCArtifactIntake(this.hardware, 0, true));
//            hardware.robo130.addCommand(new RCServoGate(this.hardware,RCServoGate.CMD_OPEN,false));
//            hardware.robo130.addCommand(new RCWait(this.hardware, 0.25));
//            hardware.robo130.addCommand(new RCArtifactIntake(this.hardware, 1, true));
//            hardware.robo130.addCommand(new RCWait(this.hardware, 0.15));
//            hardware.robo130.addCommand(new RCArtifactIntake(this.hardware, 0, true));
//            hardware.robo130.addCommand(new RCWait(this.hardware, 1));
//            hardware.robo130.addCommand(new RCArtifactIntake(this.hardware, 1, true));
//            hardware.robo130.addCommand(new RCWait(this.hardware, 0.5));
//            hardware.robo130.addCommand(new RCServoGate(this.hardware,RCServoGate.CMD_CLOSE,false));
//            hardware.robo130.addCommand(new RCArtifactIntake(this.hardware, 0, true));
//            hardware.robo130.addCommand(new RCOuttake(this.hardware, 0, true));
//            hardware.robo130.addCommand(new RCServoGate(this.hardware, RCServoGate.CMD_CLOSE, true));
//
//            //Leave
//            hardware.robo130.addCommand(new RCRoadrunner(drive.actionBuilder(new Pose2d(28,14, Math.toRadians(45)))
//                    .strafeToLinearHeading(new Vector2d(0, 24), Math.toRadians(90))
//                    .build(), hardware));

            startPose = new Pose2d(0,0, Math.toRadians(0));
            drive = new MecanumDrive(hardwareMap, startPose);
            drive.initialPose = startPose;
            hardware.drive = drive;
            hardware.gateServo.setPosition(ServoGate.CLOSE_POS);

            hardware.robo130.addCommand(new RCRoadrunner(drive.actionBuilder(new Pose2d(0,0, Math.toRadians(0)))
                    .strafeToConstantHeading(new Vector2d(0, 50))
                    .build(), hardware));

        }
        else if(!isFarStartingPos){
            //Close RED
            startPose = new Pose2d(53,-50, Math.toRadians(-45));
            drive = new MecanumDrive(hardwareMap, startPose);
            drive.initialPose = startPose;
            hardware.drive = drive;
            hardware.gateServo.setPosition(ServoGate.CLOSE_POS);
            //Sync Commands
            RobCommand gateCloseSync = new RCServoGate(this.hardware, RCServoGate.CMD_CLOSE, true);
            RobCommand firstRowCollected = new RCRoadrunner(drive.actionBuilder(new Pose2d(6,-35, Math.toRadians(-90)))
                    .strafeToConstantHeading(new Vector2d(6, -55))
                    .build(), hardware);
            RobCommand secondRowCollected = new RCRoadrunner(drive.actionBuilder(new Pose2d(-29,-34, Math.toRadians(-90)))
                    .strafeToConstantHeading(new Vector2d(-29, -57))
                    .build(), hardware);

            //Score Preloaded artifacts
            hardware.robo130.addCommand(new RCOuttake(this.hardware,0.9, true));
            hardware.robo130.addCommand(new RCRoadrunner(drive.actionBuilder(new Pose2d(53,-50, Math.toRadians(-45)))
                    .strafeToLinearHeading(new Vector2d(17, -14), Math.toRadians(-45))
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
            hardware.robo130.addCommand(new RCRoadrunner(drive.actionBuilder(new Pose2d(17,-14, Math.toRadians(-45)))
                    .strafeToLinearHeading(new Vector2d(6, -35), Math.toRadians(-90))
                    .build(), hardware));
            hardware.robo130.addCommand(new RCArtifactIntake(this.hardware, 1, true));
            hardware.robo130.addCommand(firstRowCollected);
            hardware.robo130.addCommand(new RCArtifactIntake(this.hardware, 0, true));

            //Score First row
            hardware.robo130.addCommand(new RCOuttake(this.hardware,1, true));
            hardware.robo130.addCommand(new RCRoadrunner(drive.actionBuilder(new Pose2d(6,-55, Math.toRadians(-90)))
                    .strafeToLinearHeading(new Vector2d(32, -9), Math.toRadians(-40))
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
            hardware.robo130.addCommand(new RCRoadrunner(drive.actionBuilder(new Pose2d(32,-9, Math.toRadians(-45)))
                    .strafeToLinearHeading(new Vector2d(-29, -34), Math.toRadians(-90))
                    .build(), hardware));
            hardware.robo130.addCommand(new RCArtifactIntake(this.hardware, 1, true));
            hardware.robo130.addCommand(secondRowCollected);
            hardware.robo130.addCommand(new RCArtifactIntake(this.hardware, 0, true));

            //Score Second row
            hardware.robo130.addCommand(new RCOuttake(this.hardware,1, true));
            hardware.robo130.addCommand(new RCRoadrunner(drive.actionBuilder(new Pose2d(-29,-57, Math.toRadians(-90)))
                    .strafeToLinearHeading(new Vector2d(23, -9), Math.toRadians(-40))
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
            hardware.robo130.addCommand(new RCRoadrunner(drive.actionBuilder(new Pose2d(23,-9, Math.toRadians(-45)))
                    .strafeToLinearHeading(new Vector2d(-10, -24), Math.toRadians(-90))
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
        telemetry.addData("Robot Command Stick: ", Integer.toString(hardware.robo130.robotCommandStack.getNumCommands())
                + " " + hardware.robo130.robotCommandStack.getCurrentCommandIndex()
                + " " + hardware.robo130.robotCommandStack.getNextCommandIndex());

        telemetry.addData("Roadrunner Command Stick: ", Integer.toString(hardware.robo130.roadrunnerCommandStack.getNumCommands())
                + " " + Integer.toString(hardware.robo130.roadrunnerCommandStack.getCurrentCommandIndex())
                + " " + Integer.toString(hardware.robo130.roadrunnerCommandStack.getNextCommandIndex()));
        telemetry.addData("Status", "Running");
        telemetry.update();
    }

    public void start() {
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

