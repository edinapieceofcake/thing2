package com.edinaftc.opmodes.roadrunner;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.edinaftc.library.Stickygamepad;
import com.edinaftc.library.motion.roadrunner.mecanum.DriveConstants_435_35;
import com.edinaftc.library.motion.roadrunner.mecanum.MecanumDriveBase_435_35;
import com.edinaftc.library.motion.roadrunner.mecanum.MecanumDriveREVOptimized_435_35;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name="BluePullPlateAndPark", group="Autonomous")
@Config
public class BluePullPlateAndPark extends LinearOpMode {
    private Servo left;
    private Servo right;
    private Stickygamepad _gamepad1;
    private MecanumDriveBase_435_35 drive;
    private DistanceSensor distance;

    public void runOpMode() {
        long sleepTime = 0;

        distance = hardwareMap.get(DistanceSensor.class, "reardetector");

        _gamepad1 = new Stickygamepad(gamepad1);
        left = hardwareMap.servo.get("blhook");
        right = hardwareMap.servo.get("brhook");

        drive = new MecanumDriveREVOptimized_435_35(hardwareMap);

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

                    telemetry.addData("tickPerRev, Gearing, MaxRPM", "%f %f %f", DriveConstants_435_35.MOTOR_CONFIG.getTicksPerRev(), DriveConstants_435_35.MOTOR_CONFIG.getGearing(), DriveConstants_435_35.MOTOR_CONFIG.getMaxRPM());
                    telemetry.addData("use left/right bumper to adjust sleep time", "");
                    telemetry.addData("sleep time (ms)", sleepTime);
                    telemetry.addData("distance should be about 55", "%f", distance.getDistance(DistanceUnit.CM));
                    telemetry.update();
                    this.wait();
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                    return;
                }
            }
        }

        sleep(sleepTime);

        drive.setPoseEstimate(new Pose2d(40.0, 63.0, Math.toRadians(90.0)));

        Trajectory backupAndGrabPlate = drive.trajectoryBuilder()
                .reverse()
                .splineTo(new Pose2d(50.0, 30.0, Math.toRadians(90.0)))
                .build();

        drive.followTrajectorySync(backupAndGrabPlate);

        left.setPosition(.3);
        right.setPosition(.6);
        sleep(900);

        Trajectory pullPlate = drive.trajectoryBuilder()
                .lineTo(new Vector2d(50.0, 72.0)) // drag forward and turn
                .build();

        drive.followTrajectorySync(pullPlate);

        left.setPosition(.7);
        right.setPosition(.17);
        sleep(500);

        Trajectory driveToBridge = drive.trajectoryBuilder()
                .strafeTo(new Vector2d(8.0, 72)) // drive to bridge
                .build();
        drive.followTrajectorySync(driveToBridge);
    }
}