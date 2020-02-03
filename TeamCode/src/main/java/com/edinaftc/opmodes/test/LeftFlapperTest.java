package com.edinaftc.opmodes.test;

import com.acmerobotics.dashboard.config.Config;
import com.edinaftc.library.Stickygamepad;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp()
@Disabled
@Config
public class LeftFlapperTest extends OpMode {
    Stickygamepad _gamepad1;
    Servo _leftArm;
    Servo _leftFlap;

    public static double FLAPXPOSITION = 0;
    public static double FLAPYPOSITION = 1;
    public static double ARMAPOSITION = 1;
    public static double ARMBPOSITION = 0.33;

    @Override
    public void init() {
        _gamepad1 = new Stickygamepad(gamepad1);
        _leftArm = hardwareMap.servo.get("leftArm");
        _leftFlap = hardwareMap.servo.get("leftFlap");
    }

    @Override
    public void loop() {
        _gamepad1.update();

        if (_gamepad1.x) {
            _leftFlap.setPosition(FLAPXPOSITION);
        }

        if (_gamepad1.y) {
            _leftFlap.setPosition(FLAPYPOSITION);
        }

        if (_gamepad1.b) {
            _leftArm.setPosition(ARMBPOSITION);
        }

        if (_gamepad1.a) {
            _leftArm.setPosition(ARMAPOSITION);
        }

        telemetry.addData("larm", "%f", _leftArm.getPosition());
        telemetry.addData("lflap", "%f", _leftFlap.getPosition());

        telemetry.update();
    }
}
