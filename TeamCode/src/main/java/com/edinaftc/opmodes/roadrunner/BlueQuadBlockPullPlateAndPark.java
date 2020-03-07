package com.edinaftc.opmodes.roadrunner;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.heading.LinearInterpolator;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.edinaftc.library.Stickygamepad;
import com.edinaftc.library.motion.roadrunner.mecanum.DriveConstants_1150_60;
import com.edinaftc.library.motion.roadrunner.mecanum.MecanumDriveBase_1150_60;
import com.edinaftc.library.motion.roadrunner.mecanum.MecanumDriveREVOptimized_1150_60;
import com.edinaftc.library.vision.VuforiaCamera;
import com.edinaftc.skystone.vision.SkyStoneDetector;
import com.edinaftc.skystone.vision.SkystoneLocation;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import kotlin.Unit;

@Autonomous(name="BlueQuadBlockPullPlateAndPark", group="Autonomous")
@Config
public class BlueQuadBlockPullPlateAndPark extends LinearOpMode {
    private VuforiaCamera camera;
    private SkyStoneDetector skyStoneDetector;
    private Servo arm;
    private Servo flap;
    private Servo left;
    private Servo right;
    private SkystoneLocation location = SkystoneLocation.left;
    private Stickygamepad _gamepad1;
    private MecanumDriveBase_1150_60 drive;
    private DistanceSensor distance;

    public static double LEFTFIRSTX = -20;
    public static double RIGHTFIRSTX = -38;
    public static double MIDDLEFIRSTX = -29;

    public static double LEFTSECONDX = -44;
    public static double LEFTSECONDY = 32.5;
    public static double RIGHTSECONDX = -62;
    public static double RIGHTSECONDY = 32.5;
    public static double MIDDLESECONDX = -53;
    public static double MIDDLESECONDY = 32.5;

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
        skyStoneDetector.cx0 = 300;
        skyStoneDetector.cy0 = 240;
        skyStoneDetector.cx1 = 590;
        skyStoneDetector.cy1 = 240;
        skyStoneDetector.cx2 = 920;
        skyStoneDetector.cy2 = 240;

        camera.initialize();

        flap.setPosition(.35);
        arm.setPosition(0);

        hardwareMap.servo.get("leftArm").setPosition(1);
        hardwareMap.servo.get("leftFlap").setPosition(0);

        drive = new MecanumDriveREVOptimized_1150_60(hardwareMap);

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

                    telemetry.addData("tickPerRev, Gearing, MaxRPM", "%f %f %f",
                            DriveConstants_1150_60.MOTOR_CONFIG.getTicksPerRev(),
                            DriveConstants_1150_60.MOTOR_CONFIG.getGearing(),
                            DriveConstants_1150_60.MOTOR_CONFIG.getMaxRPM());
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

        sleep(sleepTime);

        switch (location) {
            case left:
                firstBlockLocation = LEFTFIRSTX;
                secondBlockXLocation = LEFTSECONDX;
                secondblockYLocation = LEFTSECONDY;
                thirdBlockXLocation = MIDDLEFIRSTX;
                thirdBlockYLocation = 32;
                fourthBlockXLocation = RIGHTFIRSTX;
                fourthBlockYLocation = 32;
                break;
            case right:
                firstBlockLocation = RIGHTFIRSTX;
                secondBlockXLocation = RIGHTSECONDX;
                secondblockYLocation = RIGHTSECONDY;
                thirdBlockXLocation = LEFTFIRSTX;
                thirdBlockYLocation = 32;
                fourthBlockXLocation = MIDDLEFIRSTX;
                fourthBlockYLocation = 32;
                break;
            case middle:
                firstBlockLocation = MIDDLEFIRSTX;
                secondBlockXLocation = MIDDLESECONDX;
                secondblockYLocation = MIDDLESECONDY;
                thirdBlockXLocation = LEFTFIRSTX;
                thirdBlockYLocation = 32;
                fourthBlockXLocation = RIGHTFIRSTX;
                fourthBlockXLocation = 32;
                break;
        }

        drive.setPoseEstimate(new Pose2d(-40.0, 63.0, Math.toRadians(0.0)));

        Trajectory driveToFirstBlock = drive.trajectoryBuilder()
                .addMarker(.5, () -> { arm.setPosition(.3); return Unit.INSTANCE; })
                .strafeTo(new Vector2d(firstBlockLocation, 32.0)).build(); // pick up first block

        drive.followTrajectorySync(driveToFirstBlock);

        arm.setPosition(.35);
        flap.setPosition(.9);
        sleep(450);
        arm.setPosition(0);

        Trajectory dropOffFirstBlock = drive.trajectoryBuilder()
                .splineTo(new Pose2d(0.0, 36.0))
                .splineTo(new Pose2d(54.0, 30.0)) // drop off first block
                .build();

        drive.followTrajectorySync(dropOffFirstBlock);

        arm.setPosition(.15);
        flap.setPosition(.35);
        sleep(200);

        flap.setPosition(1);
        arm.setPosition(0);

        Trajectory driveToSecondBlock = drive.trajectoryBuilder()
                .reverse() // drive backwards
                .splineTo(new Pose2d(0.0, 36.0))
                .addMarker(new Vector2d(0.0, 36.0), () -> {flap.setPosition(.35); arm.setPosition(.13); return Unit.INSTANCE;})
                .splineTo(new Pose2d(secondBlockXLocation, secondblockYLocation)) // pick up second block
                .build();

        drive.followTrajectorySync(driveToSecondBlock);

        arm.setPosition(.35);
        sleep(100);
        flap.setPosition(.9);
        sleep(450);
        arm.setPosition(0);
        //sleep(200);

        Trajectory dropOffSecondBlock = drive.trajectoryBuilder()
                .splineTo(new Pose2d(0.0, 36.0))
                .splineTo(new Pose2d(54.0, 30.0)) // drop off second block
                .build();

        drive.followTrajectorySync(dropOffSecondBlock);

        flap.setPosition(.35);
        arm.setPosition(.35);
        sleep(200);

        flap.setPosition(1);
        arm.setPosition(0);

        Trajectory driveToThirdBlock = drive.trajectoryBuilder()
                .reverse() // drive backwards
                .splineTo(new Pose2d(0.0, 36.0))
                .addMarker(new Vector2d(0.0, 36.0), () -> {flap.setPosition(.35); arm.setPosition(.13); return Unit.INSTANCE;})
                .splineTo(new Pose2d(thirdBlockXLocation, thirdBlockYLocation)) // pick up second block
                .build();

        drive.followTrajectorySync(driveToThirdBlock);

        arm.setPosition(.35);
        sleep(100);
        flap.setPosition(.9);
        sleep(450);
        arm.setPosition(0);
        //sleep(200);

        Trajectory dropoffThirdBlock = drive.trajectoryBuilder()
                .splineTo(new Pose2d(0.0, 36.0))
                .splineTo(new Pose2d(45.0, 30.0)) // drop off second block
                .build();

        drive.followTrajectorySync(dropoffThirdBlock);

        flap.setPosition(.35);
        arm.setPosition(.35);
        sleep(200);

        flap.setPosition(1);
        arm.setPosition(0);

        Trajectory driveToFourthBlock = drive.trajectoryBuilder()
                .reverse()
                .splineTo(new Pose2d(0.0, 36.0))
                .addMarker(new Vector2d(0.0, 36.0), () -> {flap.setPosition(.35); arm.setPosition(.13); return Unit.INSTANCE;})
                .splineTo(new Pose2d(fourthBlockXLocation, fourthBlockYLocation)) // pick up second block
                .build();

        drive.followTrajectorySync(driveToFourthBlock);

        arm.setPosition(.35);
        sleep(100);
        flap.setPosition(.9);
        sleep(450);
        arm.setPosition(0);
        //slep(200)

        Trajectory dropOffFourthBlock = drive.trajectoryBuilder()
                .splineTo(new Pose2d(0.0, 36.0))
                .splineTo(new Pose2d(45.0, 30.0)) // drop off second block
                .build();

        drive.followTrajectorySync(dropOffFourthBlock);

        flap.setPosition(.35);
        arm.setPosition(.35);
        sleep(200);

        flap.setPosition(1);
        arm.setPosition(0);

        Trajectory backupAndPrepForTurn = drive.trajectoryBuilder()
                .reverse() // drive backwards
                .lineTo(new Vector2d(42.0, 36.0), new LinearInterpolator(Math.toRadians(90), Math.toRadians(-90)))
                .lineTo(new Vector2d(42, 24.0)) // backup
                .build();

        drive.followTrajectorySync(backupAndPrepForTurn);

        left.setPosition(0);
        right.setPosition(.8);
        sleep(300);

        Trajectory pullForward = drive.trajectoryBuilder()
                .lineTo(new Vector2d(42.0, 45.0)) // drag forward and turn
                .build();

        drive.followTrajectorySync(pullForward);

        drive.turnSync(Math.toRadians(90));

        left.setPosition(.8);
        right.setPosition(0);
        sleep(200);

        Trajectory driveToBridge = drive.trajectoryBuilder()
                .strafeTo(new Vector2d(30, 33))
                .lineTo(new Vector2d(4.0, 33)) // drive to bridge
                .build();

        drive.followTrajectorySync(driveToBridge);
    }
}