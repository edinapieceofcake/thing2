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
public class GrabberTest extends OpMode {
    Stickygamepad _gamepad1;
    Servo _front;
    Servo _back;
    private Servo _capstone;

    @Override
    public void init() {
        _gamepad1 = new Stickygamepad(gamepad1);
        _front = hardwareMap.servo.get("fg");
        _back = hardwareMap.servo.get("bg");
        _capstone = hardwareMap.servo.get("capstoneservo");
    }

    @Override
    public void loop() {
        _gamepad1.update();

        if (_gamepad1.x) {
            _front.setPosition(.4);
        }

        if (_gamepad1.y) {
            _front.setPosition(1);
        }

        if (_gamepad1.b) {
            _back.setPosition(1);
        }

        if (_gamepad1.a) {
            _back.setPosition(.3);
        }

        if (_gamepad1.dpad_up) {
            _capstone.setPosition(1);
        }

        if (_gamepad1.dpad_down) {
            _capstone.setPosition(0);
        }

        telemetry.addData("front", "%f", _front.getPosition());
        telemetry.addData("back", "%f", _back.getPosition());

        telemetry.update();
    }
}
