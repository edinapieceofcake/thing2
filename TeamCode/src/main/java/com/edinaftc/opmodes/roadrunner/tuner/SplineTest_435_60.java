package com.edinaftc.opmodes.roadrunner.tuner;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.edinaftc.library.motion.roadrunner.mecanum.MecanumDriveREVOptimized_435_60;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive")
public class SplineTest_435_60 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDriveREVOptimized_435_60 drive = new MecanumDriveREVOptimized_435_60(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .splineTo(new Pose2d(30, 30, 0))
                        .build()
        );

        sleep(2000);

        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .reverse()
                        .splineTo(new Pose2d(0, 0, 0))
                        .build()
        );

        sleep (2000);
    }
}
