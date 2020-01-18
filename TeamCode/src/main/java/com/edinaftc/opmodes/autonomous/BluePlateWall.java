package com.edinaftc.opmodes.autonomous;

import com.edinaftc.library.Stickygamepad;
import com.edinaftc.library.motion.Mecanum;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous
@Disabled
public class BluePlateWall extends LinearOpMode {
    Mecanum _mecanum;
    private Servo _left;
    private Servo _right;
    private Stickygamepad _gamepad1;

    @Override
    public void runOpMode() {
        long sleepTime = 0;
        boolean runToWall = true;

        _mecanum = new Mecanum(hardwareMap.dcMotor.get("fl"), hardwareMap.dcMotor.get("fr"),
                hardwareMap.dcMotor.get("bl"),hardwareMap.dcMotor.get("br"), telemetry);
        //_mecanum.StopResetEncodersAndRunToPosition();
        _left = hardwareMap.servo.get("blhook");
        _right = hardwareMap.servo.get("brhook");

        _gamepad1 = new Stickygamepad(gamepad1);

        hardwareMap.servo.get("rightArm").setPosition(0);
        hardwareMap.servo.get("rightFlap").setPosition(0);
        hardwareMap.servo.get("leftArm").setPosition(1);
        hardwareMap.servo.get("leftFlap").setPosition(1);

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

                    if (_gamepad1.a) {
                        runToWall = true;
                    } else if (_gamepad1.b) {
                        runToWall = false;
                    }

                    telemetry.addData("use left/right bumper to adjust sleep time", "");
                    telemetry.addData("use a or b to run to wall or middle", "");
                    telemetry.addData("sleep time (ms)", sleepTime);
                    telemetry.addData("run to wall", runToWall);
                    telemetry.update();
                    this.wait();
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                    return;
                }
            }
        }

        sleep(sleepTime);

        _mecanum.MoveBackwardsRunToPosition(0.5, 500, this);
        _mecanum.SlideRightRunToPosition(0.5, 600, this);
        _mecanum.MoveBackwardsRunToPosition(0.5, 1400, this);
        DropHooks();
        sleep(500);
        _mecanum.MoveForwardRunToPosition(0.5, 1900, this);
        LiftHooks();
        sleep(250);
        _mecanum.SlideLeftRunToPosition(0.5, 2100, this);
        _mecanum.MoveBackwardsRunToPosition(0.5, 1200, this);
        _mecanum.SlideRightRunToPosition(0.5, 800, this);
        if (runToWall) {
            _mecanum.DiagonalLeftAndUpRunToPosition(0.5, 1700, this);
            _mecanum.SlideLeftRunToPosition(0.5, 1000, this);
            _mecanum.MoveForwardRunToPosition(0.5, 300, this);
        } else {
            _mecanum.DiagonalLeftAndDownRunToPosition(0.5, 500, this);
            _mecanum.SlideLeftRunToPosition(0.5, 1500, this);
            _mecanum.MoveBackwardsRunToPosition(0.5, 200, this);

        }
    }

    public void DropHooks() {
        _left.setPosition(.3);
        _right.setPosition(.6);
    }

    public void LiftHooks() {
        _left.setPosition(.7);
        _right.setPosition(0.17);
    }
}

