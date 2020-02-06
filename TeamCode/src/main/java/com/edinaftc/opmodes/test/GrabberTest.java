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

    public static double FRONTOPEN = 1;
    public static double FRONTCLOSED = .4;
    public static double BACKOPEN = 1;
    public static double BACKCLOSED = .3;

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
            _front.setPosition(FRONTCLOSED);
        }

        if (_gamepad1.y) {
            _front.setPosition(FRONTOPEN);
        }

        if (_gamepad1.b) {
            _back.setPosition(BACKOPEN);
        }

        if (_gamepad1.a) {
            _back.setPosition(BACKCLOSED);
        }

        if (_gamepad1.dpad_up) {
            _capstone.setPosition(1);
        }

        if (_gamepad1.dpad_down) {
            _capstone.setPosition(0);
        }

        if (_gamepad1.left_bumper){
            _front.setPosition(FRONTCLOSED);
            _back.setPosition(BACKCLOSED);
        }

        if (_gamepad1.right_bumper) {
            _front.setPosition(FRONTOPEN);
            _back.setPosition(BACKOPEN);
        }

        telemetry.addData("front", "use x,y");
        telemetry.addData("back", "use a, b");
        telemetry.addData("capstone", "use dpad up down");
        telemetry.addData("grabber open/close", "use bumpers");
        telemetry.addData("front", "%f", _front.getPosition());
        telemetry.addData("back", "%f", _back.getPosition());

        telemetry.update();
    }
}
