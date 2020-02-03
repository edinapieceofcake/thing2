package com.edinaftc.opmodes.roadrunner.tuner;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.edinaftc.library.motion.roadrunner.mecanum.MecanumDriveBase_312_50;
import com.edinaftc.library.motion.roadrunner.mecanum.MecanumDriveREVOptimized_312_50;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/*
 * Op mode for tuning follower PID coefficients (located in the drive base classes). The robot
 * drives in a DISTANCE-by-DISTANCE square indefinitely.
 */
@Config
@Autonomous(group = "drive")
@Disabled
public class FollowerPIDTuner_312_50 extends LinearOpMode {
    public static double DISTANCE = 48;

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDriveBase_312_50 drive = new MecanumDriveREVOptimized_312_50(hardwareMap);

        drive.setPoseEstimate(new Pose2d(-DISTANCE / 2, -DISTANCE / 2, 0));

        waitForStart();

        if (isStopRequested()) return;

        while (!isStopRequested()) {
            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .forward(DISTANCE)
                            .build()
            );
            drive.turnSync(Math.toRadians(90));
        }
    }
}
