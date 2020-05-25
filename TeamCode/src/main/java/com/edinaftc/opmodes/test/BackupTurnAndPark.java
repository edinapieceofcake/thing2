package com.edinaftc.opmodes.test;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.heading.LinearInterpolator;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.edinaftc.library.Stickygamepad;
import com.edinaftc.library.motion.roadrunner.mecanum.DriveConstants_435_60;
import com.edinaftc.library.motion.roadrunner.mecanum.MecanumDriveBase_435_60;
import com.edinaftc.library.motion.roadrunner.mecanum.MecanumDriveREVOptimized_435_60;
import com.edinaftc.library.vision.VuforiaCamera;
import com.edinaftc.skystone.vision.SkyStoneDetector;
import com.edinaftc.skystone.vision.SkystoneLocation;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name="BackupTurnAndPark", group="Autonomous")
@Disabled
@Config
public class BackupTurnAndPark extends LinearOpMode {
    private VuforiaCamera camera;
    private SkyStoneDetector skyStoneDetector;
    private Servo arm;
    private Servo flap;
    private Servo left;
    private Servo right;
    private SkystoneLocation location = SkystoneLocation.left;
    private Stickygamepad _gamepad1;
    private MecanumDriveBase_435_60 drive;
    private DistanceSensor distance;

    public static double LEFTFIRSTX = -20;
    public static double RIGHTFIRSTX = -38;
    public static double MIDDLEFIRSTX = -28;

    public static double LEFTSECONDX = -44;
    public static double LEFTSECONDY = 32.5;
    public static double RIGHTSECONDX = -62;
    public static double RIGHTSECONDY = 32;
    public static double MIDDLESECONDX = -53;
    public static double MIDDLESECONDY = 32.5;

    public void runOpMode() {
        long sleepTime = 0;
        double firstBlockLocation = 0;
        double secondBlockXLocation = 0;
        double secondblockYLocation = 0;

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

        drive = new MecanumDriveREVOptimized_435_60(hardwareMap);

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
                            DriveConstants_435_60.MOTOR_CONFIG.getTicksPerRev(),
                            DriveConstants_435_60.MOTOR_CONFIG.getGearing(),
                            DriveConstants_435_60.MOTOR_CONFIG.getMaxRPM());
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

        drive.setPoseEstimate(new Pose2d(42.0, 30.0, Math.toRadians(0.0)));

        Trajectory backupAndPrepForTurn = drive.trajectoryBuilder()
                .reverse() // drive backwards
                .lineTo(new Vector2d(42.0, 36.0), new LinearInterpolator(Math.toRadians(90), Math.toRadians(-90)))
                .lineTo(new Vector2d(42, 26.0)) // backup
                .build();

        drive.followTrajectorySync(backupAndPrepForTurn);

        left.setPosition(0);
        right.setPosition(.8);
        sleep(200);

        Trajectory pullAndTurn = drive.trajectoryBuilder()
                .lineTo(new Vector2d(42.0, 53.0)) // drag forward and turn
                .build();

        drive.followTrajectorySync(pullAndTurn);

        drive.turnSync(Math.toRadians(90));

        left.setPosition(.8);
        right.setPosition(0);
        sleep(200);

        Trajectory driveToBridge = drive.trajectoryBuilder()
                .strafeTo(new Vector2d(30, 33))
                .lineTo(new Vector2d(8.0, 33)) // drive to bridge
                .build();

        drive.followTrajectorySync(driveToBridge);
    }
}