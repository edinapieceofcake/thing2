package com.edinaftc.opmodes.roadrunner.tuner;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.edinaftc.library.motion.roadrunner.mecanum.MecanumDriveREVOptimized_435_60;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import kotlin.Unit;

/*
 * This is a simple routine to test translational drive capabilities.
 */
@Config
@Autonomous(group = "drive")
public class StrafeTest_435_60 extends LinearOpMode {
    public static double DISTANCE = 20;

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDriveREVOptimized_435_60 drive = new MecanumDriveREVOptimized_435_60(hardwareMap);

        Trajectory trajectory = drive.trajectoryBuilder()
                .strafeLeft(DISTANCE).addMarker(() -> { return Unit.INSTANCE;})
                .build();

        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectorySync(trajectory);
    }
}
