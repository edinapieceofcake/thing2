package com.edinaftc.opmodes.roadrunner.tuner;

import com.acmerobotics.dashboard.config.Config;
import com.edinaftc.library.motion.roadrunner.mecanum.MecanumDriveBase_312_50;
import com.edinaftc.library.motion.roadrunner.mecanum.MecanumDriveREVOptimized_312_50;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/*
 * This is a simple routine to test turning capabilities.
 */
@Config
@Autonomous(group = "drive")
@Disabled
public class TurnTest_312_50 extends LinearOpMode {
    public static double ANGLE = 90; // deg

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDriveBase_312_50 drive = new MecanumDriveREVOptimized_312_50(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;

        drive.turnSync(Math.toRadians(ANGLE));
    }
}
