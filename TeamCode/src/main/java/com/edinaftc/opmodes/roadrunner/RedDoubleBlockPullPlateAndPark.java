package com.edinaftc.opmodes.roadrunner;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.heading.LinearInterpolator;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.edinaftc.library.Stickygamepad;
import com.edinaftc.library.motion.roadrunner.mecanum.DriveConstants_312_50;
import com.edinaftc.library.motion.roadrunner.mecanum.MecanumDriveBase_312_50;
import com.edinaftc.library.motion.roadrunner.mecanum.MecanumDriveREVOptimized_312_50;
import com.edinaftc.library.vision.VuforiaCamera;
import com.edinaftc.skystone.vision.SkyStoneDetector;
import com.edinaftc.skystone.vision.SkystoneLocation;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import kotlin.Unit;

@Autonomous(name="RedDoubleBlockPullPlateAndPark", group="Autonomous")
@Config
public class RedDoubleBlockPullPlateAndPark extends LinearOpMode {
    private VuforiaCamera camera;
    private SkyStoneDetector skyStoneDetector;
    private Servo arm;
    private Servo flap;
    private Servo left;
    private Servo right;
    private SkystoneLocation location = SkystoneLocation.left;
    private Stickygamepad _gamepad1;
    private MecanumDriveBase_312_50 drive;
    private DistanceSensor distance;

    public static double LEFTFIRSTX = -20;
    public static double RIGHTFIRSTX = -38;
    public static double MIDDLEFIRSTX = -30;

    public static double LEFTSECONDX = -42;
    public static double LEFTSECONDY = -33;
    public static double RIGHTSECONDX = -62;
    public static double RIGHTSECONDY = -32;
    public static double MIDDLESECONDX = -53;
    public static double MIDDLESECONDY = -33;

    public void runOpMode() {
        long sleepTime = 0;
        double firstBlockLocation = 0;
        double secondBlockXLocation = 0;
        double secondblockYLocation = 0;

        skyStoneDetector = new SkyStoneDetector();
        camera = new VuforiaCamera();
        distance = hardwareMap.get(DistanceSensor.class, "reardetector");

        _gamepad1 = new Stickygamepad(gamepad1);
        arm = hardwareMap.servo.get("leftArm");
        flap = hardwareMap.servo.get("leftFlap");
        left = hardwareMap.servo.get("blhook");
        right = hardwareMap.servo.get("brhook");

        camera.addTracker(skyStoneDetector);
        skyStoneDetector.cx0 = 280;
        skyStoneDetector.cy0 = 420;
        skyStoneDetector.cx1 = 600;
        skyStoneDetector.cy1 = 420;
        skyStoneDetector.cx2 = 900;
        skyStoneDetector.cy2 = 420;

        camera.initialize();

        flap.setPosition(1);
        arm.setPosition(1);

        hardwareMap.servo.get("rightArm").setPosition(0);
        hardwareMap.servo.get("rightFlap").setPosition(1);

        drive = new MecanumDriveREVOptimized_312_50(hardwareMap);

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
                            DriveConstants_312_50.MOTOR_CONFIG.getTicksPerRev(),
                            DriveConstants_312_50.MOTOR_CONFIG.getGearing(),
                            DriveConstants_312_50.MOTOR_CONFIG.getMaxRPM());
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
                break;
            case right:
                firstBlockLocation = RIGHTFIRSTX;
                secondBlockXLocation = RIGHTSECONDX;
                secondblockYLocation = RIGHTSECONDY;
                break;
            case middle:
                firstBlockLocation = MIDDLEFIRSTX;
                secondBlockXLocation = MIDDLESECONDX;
                secondblockYLocation = MIDDLESECONDY;
                break;
        }

        drive.setPoseEstimate(new Pose2d(-40.0, -63.0, Math.toRadians(0.0)));

        Trajectory driveToFirstBlock = drive.trajectoryBuilder()
                .addMarker(.5, () -> { arm.setPosition(.37);return Unit.INSTANCE; })
                .strafeTo(new Vector2d(firstBlockLocation, -30.0)).build(); // pick up first block

        drive.followTrajectorySync(driveToFirstBlock);

        arm.setPosition(.33);
        sleep(250);
        flap.setPosition(.2);
        sleep(650);
        arm.setPosition(1);
        sleep(200);

        Trajectory dropOffFirstBlock = drive.trajectoryBuilder()
                .splineTo(new Pose2d(0.0, -36.0))
                .splineTo(new Pose2d(60.0, -30.0)) // drop off first block
                .build();

        drive.followTrajectorySync(dropOffFirstBlock);

        arm.setPosition(.33);
        sleep(200);
        flap.setPosition(1);
        sleep(200);

        flap.setPosition(0);
        arm.setPosition(1);

        Trajectory driveToSecondBlock = drive.trajectoryBuilder()
                .reverse() // drive backwards
                .splineTo(new Pose2d(0.0, -36.0))
                .addMarker(new Vector2d(0.0, -36.0), () -> {flap.setPosition(1); arm.setPosition(.55); return Unit.INSTANCE;})
                .splineTo(new Pose2d(secondBlockXLocation, secondblockYLocation)) // pick up second block
                .build();

        drive.followTrajectorySync(driveToSecondBlock);

        arm.setPosition(.33);
        sleep(250);
        flap.setPosition(.2);
        sleep(850);
        arm.setPosition(1);
        sleep(200);

        Trajectory dropOffSecondBlock = drive.trajectoryBuilder()
                .splineTo(new Pose2d(0.0, -36.0))
                .splineTo(new Pose2d(45.0, -30.0)) // drop off second block
                .build();

        drive.followTrajectorySync(dropOffSecondBlock);

        arm.setPosition(.33);
        sleep(200);
        flap.setPosition(1);
        sleep(200);

        flap.setPosition(0);
        arm.setPosition(1);

        Trajectory backupAndPrepForTurn = drive.trajectoryBuilder()
                .reverse() // drive backwards
                .lineTo(new Vector2d(42.0, -36.0), new LinearInterpolator(Math.toRadians(-90), Math.toRadians(90)))
                .build();

        drive.followTrajectorySync(backupAndPrepForTurn);

        Trajectory backupAndGrabPlate = drive.trajectoryBuilder()
                .reverse() // drive backwards
                .lineTo(new Vector2d(42, -28.0)) // backup
                .build();

        drive.followTrajectorySync(backupAndGrabPlate);

        left.setPosition(0);
        right.setPosition(.8);
        sleep(600);

        Trajectory pullAndTurn = drive.trajectoryBuilder()
                .lineTo(new Vector2d(42.0, -53.0)) // drag forward and turn
                .build();

        drive.followTrajectorySync(pullAndTurn);

        drive.turnWithTimeoutSync(Math.toRadians(-90), 3);

        left.setPosition(.8);
        right.setPosition(0);
        sleep(500);

        Trajectory driveToBridge = drive.trajectoryBuilder()
                .strafeTo(new Vector2d(30, -33))
                .lineTo(new Vector2d(4.0, -33)) // drive to bridge
                .build();

        drive.followTrajectorySync(driveToBridge);
    }
}