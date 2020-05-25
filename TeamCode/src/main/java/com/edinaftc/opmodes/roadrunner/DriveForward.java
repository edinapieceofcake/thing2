package com.edinaftc.opmodes.roadrunner;

import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.edinaftc.library.Stickygamepad;
import com.edinaftc.library.motion.roadrunner.mecanum.DriveConstants_435_60;
import com.edinaftc.library.motion.roadrunner.mecanum.MecanumDriveBase_435_60;
import com.edinaftc.library.motion.roadrunner.mecanum.MecanumDriveREVOptimized_435_60;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="DriveForward", group="Autonomous")
//@Config
public class DriveForward extends LinearOpMode {
    private Stickygamepad _gamepad1;
    private MecanumDriveBase_435_60 drive;

    public void runOpMode() {
        long sleepTime = 0;
        double forwardDistance = 10.0;

        _gamepad1 = new Stickygamepad(gamepad1);

        drive = new MecanumDriveREVOptimized_435_60(hardwareMap);

        while (!isStarted()) {
            synchronized (this) {
                try {
                    _gamepad1.update();
                    if (_gamepad1.left_bumper) {
                        if (sleepTime > 0) {
                            sleepTime -= 500;
                        }
                    } else if (_gamepad1.right_bumper) {
                        if (sleepTime < 9000) {
                            sleepTime += 500;
                        }
                    }

                    if (_gamepad1.dpad_up) {
                        forwardDistance += .5;
                    } else if (_gamepad1.dpad_down) {
                        if (forwardDistance > 0) {
                            forwardDistance -= .5;
                        }
                    }

                    telemetry.addData("tickPerRev, Gearing, MaxRPM", "%f %f %f",
                            DriveConstants_435_60.MOTOR_CONFIG.getTicksPerRev(),
                            DriveConstants_435_60.MOTOR_CONFIG.getGearing(),
                            DriveConstants_435_60.MOTOR_CONFIG.getMaxRPM());
                    telemetry.addData("use left/right bumper to adjust sleep time", "");
                    telemetry.addData("sleep time (ms)", sleepTime);
                    telemetry.addData("use dpad up/down to increase/decrease forward distance", "");
                    telemetry.addData("distance", "%f", forwardDistance);
                    telemetry.update();
                    this.wait();
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                    return;
                }
            }
        }

        sleep(sleepTime);

        Trajectory driveForward = drive.trajectoryBuilder()
                .forward(forwardDistance)
                .build();

        drive.followTrajectorySync(driveForward);
    }
}