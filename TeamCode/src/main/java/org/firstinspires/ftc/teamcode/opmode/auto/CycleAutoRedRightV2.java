package org.firstinspires.ftc.teamcode.opmode.auto;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subsystem.Robot;
import org.firstinspires.ftc.teamcode.subsystem.SleeveDetector;
import org.firstinspires.ftc.teamcode.subsystem.lift.LiftConstants;

@Autonomous
public class CycleAutoRedRightV2 extends LinearOpMode {
    Pose2d START_POSE = new Pose2d(33, -66.5, Math.toRadians(270));
    Robot robot;
    SleeveDetector detector = new SleeveDetector();
    SleeveDetection.Color parkingPos = SleeveDetection.Color.BLUE;
    private ElapsedTime timer;

    public void runOpMode() {
        robot = new Robot(telemetry, hardwareMap);
        timer = new ElapsedTime();
        robot.init();
        robot.lift.setClaw1Pos(LiftConstants.CLAWCLOSEPOS1);
        sleep(1500);
        robot.lift.setArmPos(LiftConstants.IdleArm);
        detector.init(hardwareMap, telemetry);
        robot.lift.setOpmode("auto");

        TrajectorySequence parking1 = robot.drive.trajectorySequenceBuilder(START_POSE)
//                .addTemporalMarker(() -> {
//                    robot.lift.setClaw1Pos(LiftConstants.CLAWCLOSEPOS1);
//                })
//                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
//                    robot.lift.setTargetHeight(LiftConstants.IdleHeight);
//                    robot.lift.setArmPos(LiftConstants.IdleArm);
//                })
//                .waitSeconds(1.5)
                .setVelConstraint(robot.drive.getVelocityConstraint(40, Math.toRadians(180), DriveConstants.TRACK_WIDTH))
                .setReversed(true)
                // Preplaced
                .lineToLinearHeading(new Pose2d(45, -12, Math.toRadians(276)))
                .UNSTABLE_addTemporalMarkerOffset(-1.5, () -> {
                    robot.lift.setTargetHeight(36);
                    robot.lift.setArmPos(LiftConstants.IdleArm);
                })
                .waitSeconds(0.7)
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                    robot.lift.setArmPos(LiftConstants.IntakingArm);
                    robot.lift.setClaw1Pos(LiftConstants.CLAWOPENPOS1);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    robot.lift.setAutoRotation(240);
                    robot.lift.setArmPos(LiftConstants.IdleArm);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.55, () -> {
                    robot.lift.setTargetHeight(LiftConstants.IdleHeight);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.55, () -> {
                    robot.lift.setArmPos(LiftConstants.IntakingArm);
                })


                // Cycle #1


                .addTemporalMarker(() -> {
                })
                .setReversed(false)
                .waitSeconds(0.9)
                .setVelConstraint(robot.drive.getVelocityConstraint(40, Math.toRadians(180), DriveConstants.TRACK_WIDTH))
                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
                    robot.lift.setTargetHeight(10);
                })
                .waitSeconds(0.2)
                .lineToLinearHeading(new Pose2d(70, -12, Math.toRadians(335)))
                .setVelConstraint(robot.drive.getVelocityConstraint(20, Math.toRadians(180), DriveConstants.TRACK_WIDTH))
                .forward(12)
                .setVelConstraint(robot.drive.getVelocityConstraint(40, Math.toRadians(180), DriveConstants.TRACK_WIDTH))
                //pick up cone
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    robot.lift.setClaw1Pos(LiftConstants.CLAWCLOSEPOS1);
                })
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    robot.lift.setTargetHeight(15);
                })
                .waitSeconds(0.1)
                .addTemporalMarker(() -> {
                    robot.lift.setArmPos(LiftConstants.IdleArm);
                })
                .setReversed(true)
                .waitSeconds(0.3)
                .setVelConstraint(robot.drive.getVelocityConstraint(40, Math.toRadians(180), DriveConstants.TRACK_WIDTH))
                .lineToLinearHeading(new Pose2d(60, 2, Math.toRadians(335)))
                .UNSTABLE_addTemporalMarkerOffset(-1.5, () -> {
                    robot.lift.setAutoRotation(360);
                    robot.lift.setTargetHeight(36);
                })
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                    robot.lift.setArmPos(LiftConstants.IntakingArm);
                    robot.lift.setClaw1Pos(LiftConstants.CLAWOPENPOS1);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    robot.lift.setAutoRotation(240);
                    robot.lift.setArmPos(LiftConstants.IdleArm);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.55, () -> {
                    robot.lift.setTargetHeight(LiftConstants.IdleHeight);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.55, () -> {
                    robot.lift.setArmPos(LiftConstants.IntakingArm);
                })
                // Cycle #2


                .addTemporalMarker(() -> {
                })
                .setReversed(false)
                .waitSeconds(0.9)
                .setVelConstraint(robot.drive.getVelocityConstraint(40, Math.toRadians(180), DriveConstants.TRACK_WIDTH))
                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
                    robot.lift.setTargetHeight(8);
                })
                .waitSeconds(0.2)
                .lineToLinearHeading(new Pose2d(70, -12, Math.toRadians(328)))
                .setVelConstraint(robot.drive.getVelocityConstraint(20, Math.toRadians(180), DriveConstants.TRACK_WIDTH))
                .forward(12)
                .setVelConstraint(robot.drive.getVelocityConstraint(40, Math.toRadians(180), DriveConstants.TRACK_WIDTH))
                //pick up cone
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    robot.lift.setClaw1Pos(LiftConstants.CLAWCLOSEPOS1);
                })
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    robot.lift.setTargetHeight(15);
                })
                .waitSeconds(0.1)
                .addTemporalMarker(() -> {
                    robot.lift.setArmPos(LiftConstants.IdleArm);
                })
                .setReversed(true)
                .waitSeconds(0.3)
                .setVelConstraint(robot.drive.getVelocityConstraint(40, Math.toRadians(180), DriveConstants.TRACK_WIDTH))
                .lineToLinearHeading(new Pose2d(44, 2, Math.toRadians(328)))
                .UNSTABLE_addTemporalMarkerOffset(-1.5, () -> {
                    robot.lift.setAutoRotation(360);
                    robot.lift.setTargetHeight(36);
                })
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                    robot.lift.setArmPos(LiftConstants.IntakingArm);
                    robot.lift.setClaw1Pos(LiftConstants.CLAWOPENPOS1);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    robot.lift.setAutoRotation(240);
                    robot.lift.setArmPos(LiftConstants.IdleArm);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.55, () -> {
                    robot.lift.setTargetHeight(LiftConstants.IdleHeight);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.55, () -> {
                    robot.lift.setArmPos(LiftConstants.IntakingArm);
                })
                // Cycle #3


                .addTemporalMarker(() -> {
                })
                .setReversed(false)
                .waitSeconds(0.9)
                .setVelConstraint(robot.drive.getVelocityConstraint(40, Math.toRadians(180), DriveConstants.TRACK_WIDTH))
                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
                    robot.lift.setTargetHeight(6);
                })
                .waitSeconds(0.2)
                .lineToLinearHeading(new Pose2d(70, -12, Math.toRadians(324)))
                .setVelConstraint(robot.drive.getVelocityConstraint(20, Math.toRadians(180), DriveConstants.TRACK_WIDTH))
                .forward(12)
                .setVelConstraint(robot.drive.getVelocityConstraint(40, Math.toRadians(180), DriveConstants.TRACK_WIDTH))
                //pick up cone
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    robot.lift.setClaw1Pos(LiftConstants.CLAWCLOSEPOS1);
                })
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    robot.lift.setTargetHeight(15);
                })
                .waitSeconds(0.1)
                .addTemporalMarker(() -> {
                    robot.lift.setArmPos(LiftConstants.IdleArm);
                })
                .setReversed(true)
                .waitSeconds(0.3)
                .setVelConstraint(robot.drive.getVelocityConstraint(40, Math.toRadians(180), DriveConstants.TRACK_WIDTH))
                .lineToLinearHeading(new Pose2d(44, 2, Math.toRadians(324)))
                .UNSTABLE_addTemporalMarkerOffset(-1.5, () -> {
                    robot.lift.setAutoRotation(360);
                    robot.lift.setTargetHeight(36);
                })
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                    robot.lift.setArmPos(LiftConstants.IntakingArm);
                    robot.lift.setClaw1Pos(LiftConstants.CLAWOPENPOS1);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    robot.lift.setAutoRotation(240);
                    robot.lift.setArmPos(LiftConstants.IdleArm);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.55, () -> {
                    robot.lift.setTargetHeight(LiftConstants.IdleHeight);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.55, () -> {
                    robot.lift.setArmPos(LiftConstants.IntakingArm);
                })
                // Cycle #4


                .addTemporalMarker(() -> {
                })
                .setReversed(false)
                .waitSeconds(0.9)
                .setVelConstraint(robot.drive.getVelocityConstraint(40, Math.toRadians(180), DriveConstants.TRACK_WIDTH))
                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
                    robot.lift.setTargetHeight(4);
                })
                .waitSeconds(0.2)
                .lineToLinearHeading(new Pose2d(70, -12, Math.toRadians(320)))
                .setVelConstraint(robot.drive.getVelocityConstraint(20, Math.toRadians(180), DriveConstants.TRACK_WIDTH))
                .forward(12)
                .setVelConstraint(robot.drive.getVelocityConstraint(40, Math.toRadians(180), DriveConstants.TRACK_WIDTH))
                //pick up cone
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    robot.lift.setClaw1Pos(LiftConstants.CLAWCLOSEPOS1);
                })
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    robot.lift.setTargetHeight(15);
                })
                .waitSeconds(0.1)
                .addTemporalMarker(() -> {
                    robot.lift.setArmPos(LiftConstants.IdleArm);
                })
                .setReversed(true)
                .waitSeconds(0.3)
                .setVelConstraint(robot.drive.getVelocityConstraint(40, Math.toRadians(180), DriveConstants.TRACK_WIDTH))
                .lineToLinearHeading(new Pose2d(44, 2, Math.toRadians(332.5)))
                .UNSTABLE_addTemporalMarkerOffset(-1.5, () -> {
                    robot.lift.setAutoRotation(360);
                    robot.lift.setTargetHeight(36);
                })
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                    robot.lift.setArmPos(LiftConstants.IntakingArm);
                    robot.lift.setClaw1Pos(LiftConstants.CLAWOPENPOS1);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    robot.lift.setAutoRotation(240);
                    robot.lift.setArmPos(LiftConstants.IdleArm);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.55, () -> {
                    robot.lift.setTargetHeight(LiftConstants.IdleHeight);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.55, () -> {
                    robot.lift.setArmPos(LiftConstants.IntakingArm);
                })

                //park

                .back(10)
                .build();


        robot.drive.setPoseEstimate(START_POSE);

        // Waiting for start
        while (!isStarted() && !isStopRequested()) {
            parkingPos = detector.getColor();
            telemetry.addData("timer", timer.milliseconds());
            telemetry.update();
        }

        // Start...
        detector.stop();
        waitForStart();

        if (parkingPos == SleeveDetection.Color.MAGENTA) {
            robot.drive.followTrajectorySequenceAsync(parking1);
            detector.stop();
        } else if (parkingPos == SleeveDetection.Color.BLUE) {
            robot.drive.followTrajectorySequenceAsync(parking1);
            detector.stop();
        } else if (parkingPos == SleeveDetection.Color.RED) {
            robot.drive.followTrajectorySequenceAsync(parking1);
            detector.stop();
        }

        while (opModeIsActive()) {
            telemetry.addData("turret pos", robot.lift.getCurrentRotation());
            telemetry.addData("turret target", robot.lift.getAutoRotation());
            telemetry.addData("opmode", robot.lift.getOpmode());
            telemetry.addData("slide pos", robot.lift.getCurrentPosition());
            telemetry.update();
            robot.update();
        }
    }
}