package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.TankDrive;

public final class SplineTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(53,50, Math.toRadians(45));
        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

            waitForStart();

            Actions.runBlocking(
                drive.actionBuilder(beginPose)
//                        .strafeToLinearHeading(new Vector2d(17, 14), Math.toRadians(45))
//                        .splineToSplineHeading(new Pose2d(6, 35, Math.toRadians(90)), Math.toRadians(90))
//                        .setTangent(Math.toRadians(90))
//                        .splineToConstantHeading(new Vector2d(6, 50), Math.toRadians(90))
//                        .setTangent(Math.toRadians(90))
//                        .splineToConstantHeading(new Vector2d(16, 60), Math.toRadians(0))
//                        .setTangent(Math.toRadians(0))
//                        .splineToSplineHeading(new Pose2d(17, 14, Math.toRadians(45)), Math.toRadians(-90))
//
//                        .strafeToLinearHeading(new Vector2d(-29, 34), Math.toRadians(90))
//                        .strafeToConstantHeading(new Vector2d(-29, 57))
//                        .strafeToLinearHeading(new Vector2d(28, 14), Math.toRadians(40))
//                        .strafeToLinearHeading(new Vector2d(-10, 24), Math.toRadians(90))

                        .strafeToLinearHeading(new Vector2d(25, 22), Math.toRadians(45))
                        .setTangent(Math.toRadians(30))
                        .splineToSplineHeading(new Pose2d(0, 24, Math.toRadians(90)), Math.toRadians(90))
                        .splineToConstantHeading(new Vector2d(0, 36), Math.toRadians(90))
                        .setTangent(Math.toRadians(-90))
                        .splineToSplineHeading(new Pose2d(17, 14, Math.toRadians(45)), Math.toRadians(-135))

                        .setTangent(Math.toRadians(180))
                        .splineToSplineHeading(new Pose2d(-22, 24, Math.toRadians(90)), Math.toRadians(90))
                        .splineToConstantHeading(new Vector2d(-22, 36), Math.toRadians(90))
                        .setTangent(Math.toRadians(-90))
                        .splineToSplineHeading(new Pose2d(22, 19, Math.toRadians(45)), Math.toRadians(180))

                        .setTangent(Math.toRadians(180))
                        .splineToSplineHeading(new Pose2d(-44, 24, Math.toRadians(90)), Math.toRadians(90))
                        .splineToConstantHeading(new Vector2d(-44, 36), Math.toRadians(90))
                        .setTangent(Math.toRadians(-90))
                        .splineToSplineHeading(new Pose2d(22, 19, Math.toRadians(45)), Math.toRadians(180))
                        .strafeToLinearHeading(new Vector2d(-10, 24), Math.toRadians(90))
                        .build());
        } else if (TuningOpModes.DRIVE_CLASS.equals(TankDrive.class)) {
            TankDrive drive = new TankDrive(hardwareMap, beginPose);

            waitForStart();

            Actions.runBlocking(
                    drive.actionBuilder(beginPose)
                            .splineTo(new Vector2d(30, 30), Math.PI / 2)
                            .splineTo(new Vector2d(0, 60), Math.PI)
                            .build());
        } else {
            throw new RuntimeException();
        }
    }
}
