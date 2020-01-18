package com.edinaftc.opmodes.roadrunner;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.edinaftc.library.Stickygamepad;
import com.edinaftc.library.motion.roadrunner.mecanum.DriveConstants_435_40;
import com.edinaftc.library.motion.roadrunner.mecanum.MecanumDriveBase_435_40;
import com.edinaftc.library.motion.roadrunner.mecanum.MecanumDriveREVOptimized_435_40;
import com.edinaftc.library.vision.VuforiaCamera;
import com.edinaftc.skystone.vision.SkyStoneDetector;
import com.edinaftc.skystone.vision.SkystoneLocation;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name="BlueQuadBlockAndPark", group="Autonomous")
@Config
public class BlueQuadBlockAndPark extends LinearOpMode {
    private VuforiaCamera camera;
    private SkyStoneDetector skyStoneDetector;
    private Servo arm;
    private Servo flap;
    private Servo left;
    private Servo right;
    private SkystoneLocation location = SkystoneLocation.left;
    private Stickygamepad _gamepad1;
    private MecanumDriveBase_435_40 drive;
    private DistanceSensor distance;

    public void runOpMode() {
        long sleepTime = 0;
        double firstBlockLocation = 0;
        double secondBlockXLocation = 0;
        double secondblockYLocation = 0;
        double thirdBlockXLocation = 0;
        double thirdBlockYLocation = 0;
        double fourthBlockXLocation = 0;
        double fourthBlockYLocation = 0;

        skyStoneDetector = new SkyStoneDetector();
        camera = new VuforiaCamera();
        distance = hardwareMap.get(DistanceSensor.class, "reardetector");

        _gamepad1 = new Stickygamepad(gamepad1);
        arm = hardwareMap.servo.get("rightArm");
        flap = hardwareMap.servo.get("rightFlap");
        left = hardwareMap.servo.get("blhook");
        right = hardwareMap.servo.get("brhook");

        camera.addTracker(skyStoneDetector);
        skyStoneDetector.cx0 = 190;
        skyStoneDetector.cy0 = 210;
        skyStoneDetector.cx1 = 520;
        skyStoneDetector.cy1 = 210;
        skyStoneDetector.cx2 = 810;
        skyStoneDetector.cy2 = 210;

        camera.initialize();

        flap.setPosition(1);

        hardwareMap.servo.get("leftArm").setPosition(1);
        hardwareMap.servo.get("leftFlap").setPosition(1);

        drive = new MecanumDriveREVOptimized_435_40(hardwareMap);
        while (!isStarted()) {
            synchronized (this) {
                try {
                    location = skyStoneDetector.getLocation();
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

                    telemetry.addData("tickPerRev, Gearing, MaxRPM", "%f %f %f", DriveConstants_435_40.MOTOR_CONFIG.getTicksPerRev(), DriveConstants_435_40.MOTOR_CONFIG.getGearing(), DriveConstants_435_40.MOTOR_CONFIG.getMaxRPM());
                    telemetry.addData("use left/right bumper to adjust sleep time", "");
                    telemetry.addData("sleep time (ms)", sleepTime);
                    telemetry.addData("location ", location);
                    telemetry.addData("distance should be about 55", "%f", distance.getDistance(DistanceUnit.CM));
                    telemetry.update();
                    this.wait();
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                    return;
                }
            }
        }

        hardwareMap.servo.get("leftArm").setPosition(1);

        sleep(sleepTime);

        switch (location) {
            case left:
                firstBlockLocation = -22;
                secondBlockXLocation = -44;
                secondblockYLocation = 35;
                thirdBlockXLocation = -28; // middle
                thirdBlockYLocation = 34;
                fourthBlockXLocation = -36;
                fourthBlockYLocation = 34;
                break;
            case right:
                firstBlockLocation = -38;
                secondBlockXLocation = -62;
                secondblockYLocation = 33;
                thirdBlockXLocation = -20;
                thirdBlockYLocation = 34;
                fourthBlockXLocation = -28;
                fourthBlockYLocation = 34;
                break;
            case middle:
                firstBlockLocation = -30;
                secondBlockXLocation = -54;
                secondblockYLocation = 34;
                thirdBlockXLocation = -20;
                thirdBlockYLocation = 34;
                fourthBlockXLocation = -36;
                fourthBlockYLocation = 35;
                break;
        }

        drive.setPoseEstimate(new Pose2d(-40.0, 63.0, Math.toRadians(0.0)));

        Trajectory driveToFirstBlock = drive.trajectoryBuilder()
                .strafeTo(new Vector2d(firstBlockLocation, 32.0)).build(); // pick up first block

        drive.followTrajectorySync(driveToFirstBlock);
        arm.setPosition(1);
        sleep(250);
        flap.setPosition(0);
        sleep(450);
        arm.setPosition(0);
        sleep(100);

        Trajectory dropOffFirstBlock = drive.trajectoryBuilder()
                .splineTo(new Pose2d(0.0, 36.0))
                .splineTo(new Pose2d(20.0, 36.0)) // drop off first block
                .build();

        drive.followTrajectorySync(dropOffFirstBlock);

        flap.setPosition(1);
        arm.setPosition(1);
        sleep(200);
        flap.setPosition(0);
        arm.setPosition(0);

        Trajectory driveToSecondBlock = drive.trajectoryBuilder()
                .reverse() // drive backwards
                .splineTo(new Pose2d(0.0, 36.0))
                .splineTo(new Pose2d(secondBlockXLocation, secondblockYLocation)) // pick up second block
                .build();

        drive.followTrajectorySync(driveToSecondBlock);
        flap.setPosition(1);
        arm.setPosition(1);
        sleep(550);
        flap.setPosition(0);
        sleep(550);
        arm.setPosition(0);
        sleep(100);

        Trajectory dropOffSecondBlock = drive.trajectoryBuilder()
                .splineTo(new Pose2d(0.0, 36.0))
                .splineTo(new Pose2d(20.0, 36)) // drop off second block
                .build();

        drive.followTrajectorySync(dropOffSecondBlock);

        flap.setPosition(1);
        arm.setPosition(1);
        sleep(200);
        flap.setPosition(0);
        arm.setPosition(0);

        Trajectory driveToThirdBlock = drive.trajectoryBuilder()
                .reverse() // drive backwards
                .splineTo(new Pose2d(0.0, 36.0))
                .splineTo(new Pose2d(thirdBlockXLocation, thirdBlockYLocation)) // pick up second block
                .build();

        drive.followTrajectorySync(driveToThirdBlock);

        flap.setPosition(1);
        arm.setPosition(1);
        sleep(550);
        flap.setPosition(0);
        sleep(550);
        arm.setPosition(0);
        sleep(100);

        Trajectory dropOffThirdBlock = drive.trajectoryBuilder()
                .splineTo(new Pose2d(0.0, 36.0))
                .splineTo(new Pose2d(20.0, 36.0)) // drop off second block
                .build();

        drive.followTrajectorySync(dropOffThirdBlock);

        flap.setPosition(1);
        arm.setPosition(1);
        sleep(200);
        flap.setPosition(0);
        arm.setPosition(0);

        Trajectory driveToFourthBlock = drive.trajectoryBuilder()
                .reverse() // drive backwards
                .splineTo(new Pose2d(0.0, 36.0))
                .splineTo(new Pose2d(fourthBlockXLocation, fourthBlockYLocation)) // pick up second block
                .build();

        drive.followTrajectorySync(driveToFourthBlock);

        flap.setPosition(1);
        arm.setPosition(1);
        sleep(550);
        flap.setPosition(0);
        sleep(550);
        arm.setPosition(0);
        sleep(100);

        Trajectory dropOffFourthBlock = drive.trajectoryBuilder()
                .splineTo(new Pose2d(0.0, 36.0))
                .splineTo(new Pose2d(20.0, 36.0)) // drop off second block
                .build();

        drive.followTrajectorySync(dropOffFourthBlock);

        flap.setPosition(1);
        arm.setPosition(1);
        sleep(200);
        flap.setPosition(0);
        arm.setPosition(0);

        Trajectory driveToBridge = drive.trajectoryBuilder()
                .reverse()
                .splineTo(new Pose2d(4.0, 36.0))
                .build();


        drive.followTrajectorySync(driveToBridge);
    }
}