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

@Autonomous(name="God Red Alliance Block Side", group="Autonomous")
@Disabled
public class GodRedAllianceSide extends LinearOpMode {
    private Mecanum _mecanum;
    private VuforiaCamera _camera;
    private SkyStoneDetector _skyStoneDetector;
    private Servo _arm;
    private Servo _flap;
    private Servo _left;
    private Servo _right;
    private SkystoneLocation _location = SkystoneLocation.left;
    private double motorPower = 1;
    private Stickygamepad _gamepad1;
    private BNO055IMU _imu = null;

    private int MINIMUMDISTANCEFORFIRSTBLOCK = 3140;
    private int MINIMUMDISTANCEFORSECONDBLOCK = 4690;
    private int MONIMUMDISTANCVETOBRIDGE = 4690;


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
        TURNED_RIGHT_TOWARDS_WALL,
        BACKED_AND_GRABBED_PLATE,
        PULLED_AND_TURNED_RIGHT_WITH_PLATE,
        DRIVEN_UNDER_BRIDGE
    }

    public GodRedAllianceSide.AutonomousStates DriveToFirstBlock() {
        _mecanum.SlideLeftRunToPosition(.5, 1725, this);

        _flap.setPosition(0);

        switch (_location) {
            case left:
                _mecanum.MoveForwardRunWithEncodersAndIMU(motorPower, 925, .2,0,this, telemetry);
                break;

            case middle:
                _mecanum.MoveForwardRunWithEncodersAndIMU(motorPower, 275, .2,0,this, telemetry);
                break;

            case right:
                _mecanum.MoveForwardRunWithEncodersAndIMU(motorPower, 80, .2,0,this, telemetry);
                break;
        }

        sleep(500); // need time for flap to open

        return GodRedAllianceSide.AutonomousStates.DRIVEN_TO_FIRST_BLOCK;
    }

    public GodRedAllianceSide.AutonomousStates DriveToSecondBlock() {
        switch (_location) {
            case left:
                _mecanum.MoveBackwardRunWithEncodersAndIMU(motorPower, MINIMUMDISTANCEFORSECONDBLOCK, .2, 0, this, telemetry);
                break;

            case middle:
                _mecanum.MoveBackwardRunWithEncodersAndIMU(motorPower, MINIMUMDISTANCEFORSECONDBLOCK + 500, .2, 0, this, telemetry);
                break;

            case right:
                _mecanum.MoveBackwardRunWithEncodersAndIMU(motorPower, MINIMUMDISTANCEFORSECONDBLOCK + 975, .2, 0, this, telemetry);
                break;
        }

        _flap.setPosition(0);
        sleep(500); // need time for flap to open
        return GodRedAllianceSide.AutonomousStates.DRIVEN_TO_SECOND_BLOCK;
    }

    public GodRedAllianceSide.AutonomousStates PickUpFirstBlock() {
        PickUpBlock2();

        return GodRedAllianceSide.AutonomousStates.PICKED_UP_FIRST_BLOCK;
    }

    public GodRedAllianceSide.AutonomousStates PickUpSecondBlock() {
        PickUpBlock2();

        return GodRedAllianceSide.AutonomousStates.PICKED_UP_SECOND_BLOCK;
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

    public GodRedAllianceSide.AutonomousStates DriveToBridgeForFirstBlock() {
        switch (_location) {
            case left:
                _mecanum.MoveForwardRunWithEncodersAndIMU(motorPower, MINIMUMDISTANCEFORFIRSTBLOCK, .2, 0,this, telemetry);
                break;

            case middle:
                _mecanum.MoveForwardRunWithEncodersAndIMU(motorPower, MINIMUMDISTANCEFORFIRSTBLOCK + 500, .2, 0, this, telemetry);
                break;

            case right:
                _mecanum.MoveForwardRunWithEncodersAndIMU(motorPower, MINIMUMDISTANCEFORFIRSTBLOCK + 950, .2, 0, this, telemetry);
                break;
        }

        return GodRedAllianceSide.AutonomousStates.DRIVEN_TO_BRIDGE_FOR_FIRST_BLOCK;
    }

    public GodRedAllianceSide.AutonomousStates DriveToBridgeForSecondBlock() {
        _mecanum.SlideRightRunWithEncoders(0.5, 100, this);
        switch (_location) {
            case left:
                _mecanum.MoveForwardRunWithEncodersAndIMU(motorPower, MONIMUMDISTANCVETOBRIDGE, .2, 0, this, telemetry);
                break;

            case middle:
                _mecanum.MoveForwardRunWithEncodersAndIMU(motorPower, MONIMUMDISTANCVETOBRIDGE + 500, .2, 0, this, telemetry);
                break;

            case right:
                _mecanum.MoveForwardRunWithEncodersAndIMU(motorPower, MONIMUMDISTANCVETOBRIDGE + 975, .2, 0, this, telemetry);
                break;
        }

        return GodRedAllianceSide.AutonomousStates.DRIVEN_TO_BRIDGE_FOR_SECOND_BLOCK;
    }

    public GodRedAllianceSide.AutonomousStates TurnRightTowardsWall() {
        _mecanum.TurnRightRunToPosition(1, 1415, this);

        return AutonomousStates.TURNED_RIGHT_TOWARDS_WALL;
    }

    public GodRedAllianceSide.AutonomousStates BackupAndGrabPlate() {
        _mecanum.MoveBackwardsRunToPosition(1, 200, this);

        _left.setPosition(.3);
        _right.setPosition(.6);

        sleep(500);

        return AutonomousStates.BACKED_AND_GRABBED_PLATE;
    }

    public GodRedAllianceSide.AutonomousStates PullAndTurnPlate() {
        _mecanum.MoveForwardRunToPosition(1, 200, this);
        _mecanum.TurnRightRunToPosition(.5, 1415, this);
        _mecanum.MoveBackwardsRunToPosition(1, 200, this);

        return AutonomousStates.PULLED_AND_TURNED_RIGHT_WITH_PLATE;
    }

    public GodRedAllianceSide.AutonomousStates DriveUnderBridge() {
        _left.setPosition(.7);
        _right.setPosition(0.17);

        sleep(500);

        _mecanum.MoveForwardRunWithEncoders(motorPower, 1150, this);
        return GodRedAllianceSide.AutonomousStates.DRIVEN_UNDER_BRIDGE;
    }

    private void DropOffBlock() {
        _flap.setPosition(0);
        _arm.setPosition(0);
        sleep(500);

        _flap.setPosition(1);
        _arm.setPosition(1);
        sleep(500);
    }

    public GodRedAllianceSide.AutonomousStates DropOffFirstBlock() {
        DropOffBlock();

        return GodRedAllianceSide.AutonomousStates.DROPPED_OFF_FIRST_BLOCK;
    }

    public GodRedAllianceSide.AutonomousStates DropOffSecondBlock() {
        DropOffBlock();

        return GodRedAllianceSide.AutonomousStates.DROPPED_OFF_SECOND_BLOCK;
    }

    public void runOpMode() {
        GodRedAllianceSide.AutonomousStates currentState = GodRedAllianceSide.AutonomousStates.STARTED;
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
        _left = hardwareMap.servo.get("blhook");
        _right = hardwareMap.servo.get("brhook");

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
        _mecanum.enableIMU(_imu, this);
        _mecanum.startIMU();

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

        while (opModeIsActive() && (currentState != GodRedAllianceSide.AutonomousStates.DRIVEN_UNDER_BRIDGE)) {
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
                    currentState = TurnRightTowardsWall();
                    break;
                case TURNED_RIGHT_TOWARDS_WALL:
                    currentState = BackupAndGrabPlate();
                    break;
                case BACKED_AND_GRABBED_PLATE:
                    currentState = PullAndTurnPlate();
                    break;
                case PULLED_AND_TURNED_RIGHT_WITH_PLATE:
                    currentState = DriveUnderBridge();
                    break;
            }
        }

        _camera.close();
        _mecanum.stopIMU();
    }

}