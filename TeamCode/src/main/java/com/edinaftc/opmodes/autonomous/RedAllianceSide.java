package com.edinaftc.opmodes.autonomous;

import com.edinaftc.library.Stickygamepad;
import com.edinaftc.library.motion.Mecanum;
import com.edinaftc.library.vision.VuforiaCamera;
import com.edinaftc.skystone.vision.SkyStoneDetector;
import com.edinaftc.skystone.vision.SkystoneLocation;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name="Red Alliance Block Side", group="Autonomous")
@Disabled
public class RedAllianceSide extends LinearOpMode {
    private Mecanum _mecanum;
    private VuforiaCamera _camera;
    private SkyStoneDetector _skyStoneDetector;
    private Servo _arm;
    private Servo _flap;
    private SkystoneLocation _location = SkystoneLocation.left;
    private double motorPower = .5;
    private Stickygamepad _gamepad1;
    private BNO055IMU _imu = null;

    private int MINIMUMDISTANCEFORFIRSTBLOCK = 2640;
    private int MINIMUMDISTANCEFORSECONDBLOCK = 4190;
    private int MONIMUMDISTANCVETOBRIDGE = 4190;


    public enum AutonomousStates{
        STARTED,
        DRIVEN_TO_FIRST_BLOCK,
        PICKED_UP_FIRST_BLOCK,
        DRIVEN_TO_BRIDGE_FOR_FIRST_BLOCK,
        DROPPED_OFF_FIRST_BLOCK,
        DRIVEN_TO_SECOND_BLOCK,
        PICKED_UP_SECOND_BLOCK,
        DRIVEN_TO_BRIDGE_FOR_SECOND_BLOCK,
        DROPPED_OFF_SECOND_BLOCK,
        DRIVEN_UNDER_BRIDGE
    }

    public RedAllianceSide.AutonomousStates DriveToFirstBlock() {
        _mecanum.SlideLeftRunToPosition(.5, 1725, this);

        _flap.setPosition(0);

        switch (_location) {
            case left:
                _mecanum.MoveForwardRunWithEncoders(motorPower, 1125, this);
                break;

            case middle:
                _mecanum.MoveForwardRunWithEncoders(motorPower, 575, this);
                break;

            case right:
                _mecanum.MoveForwardRunWithEncoders(motorPower, 80, this);
                break;
        }

        sleep(500); // need time for flap to open

        return RedAllianceSide.AutonomousStates.DRIVEN_TO_FIRST_BLOCK;
    }

    public RedAllianceSide.AutonomousStates DriveToSecondBlock() {
        switch (_location) {
            case left:
                _mecanum.MoveBackwardsRunWithEncoders(motorPower, MINIMUMDISTANCEFORSECONDBLOCK, this);
                break;

            case middle:
                _mecanum.MoveBackwardsRunWithEncoders(motorPower, MINIMUMDISTANCEFORSECONDBLOCK + 500, this);
                break;

            case right:
                _mecanum.MoveBackwardsRunWithEncoders(motorPower, MINIMUMDISTANCEFORSECONDBLOCK + 975, this);
                break;
        }

        _flap.setPosition(0);
        sleep(500); // need time for flap to open
        return RedAllianceSide.AutonomousStates.DRIVEN_TO_SECOND_BLOCK;
    }

    public RedAllianceSide.AutonomousStates PickUpFirstBlock() {
        PickUpBlock2();

        return RedAllianceSide.AutonomousStates.PICKED_UP_FIRST_BLOCK;
    }

    public RedAllianceSide.AutonomousStates PickUpSecondBlock() {
        PickUpBlock2();

        return RedAllianceSide.AutonomousStates.PICKED_UP_SECOND_BLOCK;
    }

    private void PickUpBlock() {
        _arm.setPosition(0);
        sleep(500);
        _flap.setPosition(1);
        sleep(500);

        _arm.setPosition(1);
        sleep(500);
    }

    private void PickUpBlock2() {
        _arm.setPosition(.35);
        sleep(400);
        _mecanum.SlideLeftRunWithEncoders(.5, 200, this);
        _arm.setPosition(0);
        sleep(400);
        _flap.setPosition(1);
        sleep(750);
        _arm.setPosition(1);
        sleep(500);

        _mecanum.SlideRightRunWithEncoders(.5, 200, this);
    }

    public RedAllianceSide.AutonomousStates DriveToBridgeForFirstBlock() {
        switch (_location) {
            case left:
                _mecanum.MoveForwardRunWithEncoders(motorPower, MINIMUMDISTANCEFORFIRSTBLOCK, this);
                break;

            case middle:
                _mecanum.MoveForwardRunWithEncoders(motorPower, MINIMUMDISTANCEFORFIRSTBLOCK + 500, this);
                break;

            case right:
                _mecanum.MoveForwardRunWithEncoders(motorPower, MINIMUMDISTANCEFORFIRSTBLOCK + 950, this);
                break;
        }

        return RedAllianceSide.AutonomousStates.DRIVEN_TO_BRIDGE_FOR_FIRST_BLOCK;
    }

    public RedAllianceSide.AutonomousStates DriveToBridgeForSecondBlock() {
        _mecanum.SlideRightRunWithEncoders(0.5, 100, this);
        switch (_location) {
            case left:
                _mecanum.MoveForwardRunWithEncoders(motorPower, MONIMUMDISTANCVETOBRIDGE, this);
                break;

            case middle:
                _mecanum.MoveForwardRunWithEncoders(motorPower, MONIMUMDISTANCVETOBRIDGE + 500, this);
                break;

            case right:
                _mecanum.MoveForwardRunWithEncoders(motorPower, MONIMUMDISTANCVETOBRIDGE + 975, this);
                break;
        }

        return RedAllianceSide.AutonomousStates.DRIVEN_TO_BRIDGE_FOR_SECOND_BLOCK;
    }

    public RedAllianceSide.AutonomousStates DriveUnderBridge() {
        _mecanum.MoveBackwardsRunWithEncoders(motorPower, 1150, this);
        return RedAllianceSide.AutonomousStates.DRIVEN_UNDER_BRIDGE;
    }

    private void DropOffBlock() {
        _flap.setPosition(0);
        _arm.setPosition(0);
        sleep(500);

        _flap.setPosition(1);
        _arm.setPosition(1);
        sleep(500);
    }

    public RedAllianceSide.AutonomousStates DropOffFirstBlock() {
        DropOffBlock();

        return RedAllianceSide.AutonomousStates.DROPPED_OFF_FIRST_BLOCK;
    }

    public RedAllianceSide.AutonomousStates DropOffSecondBlock() {
        DropOffBlock();

        return RedAllianceSide.AutonomousStates.DROPPED_OFF_SECOND_BLOCK;
    }

    public void runOpMode() {
        RedAllianceSide.AutonomousStates currentState = RedAllianceSide.AutonomousStates.STARTED;
        int counter = 0;
        String[] messages = new String[]{ "\\", "|", "/", "-", "\\", "|", "/", "-" };
        long sleepTime = 0;

        _skyStoneDetector = new SkyStoneDetector();
        _camera = new VuforiaCamera();

        _mecanum = new Mecanum(hardwareMap.dcMotor.get("fl"), hardwareMap.dcMotor.get("fr"),
                hardwareMap.dcMotor.get("bl"),hardwareMap.dcMotor.get("br"), telemetry);
        _arm = hardwareMap.servo.get("leftArm");
        _flap = hardwareMap.servo.get("leftFlap");
        _gamepad1 = new Stickygamepad(gamepad1);

        _camera.addTracker(_skyStoneDetector);
        _skyStoneDetector.cx0 = 330;
        _skyStoneDetector.cy0 = 420;
        _skyStoneDetector.cx1 = 640;
        _skyStoneDetector.cy1 = 420;
        _skyStoneDetector.cx2 = 950;
        _skyStoneDetector.cy2 = 420;

        _camera.initialize();

        _flap.setPosition(1);

        hardwareMap.servo.get("rightArm").setPosition(0);
        hardwareMap.servo.get("rightFlap").setPosition(0);

        _imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        _imu.initialize(parameters);

        while (!_imu.isGyroCalibrated()) {
            idle();
            telemetry.addData("Calibrating IMU", "%s", messages[counter]);
            telemetry.update();
            if (counter == 7) {
                counter = 0;
            } else {
                counter++;
            }
        }

        while (!isStarted()) {
            synchronized (this) {
                try {
                    _location = _skyStoneDetector.getLocation();
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

                    telemetry.addData("use left/right bumper to adjust sleep time", "");
                    telemetry.addData("sleep time (ms)", sleepTime);
                    telemetry.addData("location ", _location);
                    telemetry.update();
                    this.wait();
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                    return;
                }
            }
        }

        sleep(sleepTime);

        while (opModeIsActive() && (currentState != RedAllianceSide.AutonomousStates.DRIVEN_UNDER_BRIDGE)) {
            switch (currentState) {
                case STARTED:
                    currentState = DriveToFirstBlock();
                    break;
                case DRIVEN_TO_FIRST_BLOCK:
                    currentState = PickUpFirstBlock();
                    break;
                case PICKED_UP_FIRST_BLOCK:
                    currentState = DriveToBridgeForFirstBlock();
                    break;
                case DRIVEN_TO_BRIDGE_FOR_FIRST_BLOCK:
                    currentState = DropOffFirstBlock();
                    break;
                case DROPPED_OFF_FIRST_BLOCK:
                    currentState = DriveToSecondBlock();
                    break;
                case DRIVEN_TO_SECOND_BLOCK:
                    currentState = PickUpSecondBlock();
                    break;
                case PICKED_UP_SECOND_BLOCK:
                    currentState = DriveToBridgeForSecondBlock();
                    break;
                case DRIVEN_TO_BRIDGE_FOR_SECOND_BLOCK:
                    currentState = DropOffSecondBlock();
                    break;
                case DROPPED_OFF_SECOND_BLOCK:
                    currentState = DriveUnderBridge();
                    break;
            }
        }

        _camera.close();
    }

}