package com.edinaftc.opmodes.test;

import com.edinaftc.library.Stickygamepad;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp()
//@Disabled
public class WheelLocationClampTest extends OpMode {
    Stickygamepad _gamepad1;
//    DcMotor _frontLeft;
//    DcMotor _frontRight;
//    DcMotor _backLeft;
//    DcMotor _backRight;
    Servo _backLeftServo;
    Servo _backRightServo;

    @Override
    public void init() {
        _gamepad1 = new Stickygamepad(gamepad1);
//        _frontLeft = hardwareMap.dcMotor.get("fl");
//        _frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
//        _frontRight = hardwareMap.dcMotor.get("fr");
//        _frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
//        _backLeft = hardwareMap.dcMotor.get("bl");
//        _backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
//        _backRight = hardwareMap.dcMotor.get("br");
//        _backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);


        _backLeftServo = hardwareMap.servo.get("blhook");
        _backRightServo = hardwareMap.servo.get("brhook");
    }

    @Override
    public void loop() {
        _gamepad1.update();

//        if (gamepad1.dpad_left) {
//            _frontLeft.setPower(.5);
//        }
//
//        if (!gamepad1.dpad_left) {
//            _frontLeft.setPower(0);
//        }
//
//        if (gamepad1.dpad_up) {
//            _frontRight.setPower(.5);
//        }
//
//        if (!gamepad1.dpad_up) {
//            _frontRight.setPower(0);
//        }
//
//        if (gamepad1.dpad_right) {
//            _backRight.setPower(.5);
//        }
//
//        if (!gamepad1.dpad_right) {
//            _backRight.setPower(0);
//        }
//
//        if (gamepad1.dpad_down) {
//            _backLeft.setPower(.5);
//        }
//
//        if (!gamepad1.dpad_down) {
//            _backLeft.setPower(0);
//        }

        if (_gamepad1.x) {
            _backLeftServo.setPosition(_backLeftServo.getPosition() + .1);
        }

        if (_gamepad1.y) {
            _backLeftServo.setPosition(_backLeftServo.getPosition() - .1);
        }

        if (_gamepad1.b) {
            _backRightServo.setPosition(_backRightServo.getPosition() + .1);
        }

        if (_gamepad1.a) {
            _backRightServo.setPosition(_backRightServo.getPosition() - .1);
        }

//        telemetry.addData("fl", "%d", _frontLeft.getCurrentPosition());
//        telemetry.addData("fr", "%d", _frontRight.getCurrentPosition());
//        telemetry.addData("bl", "%d", _backLeft.getCurrentPosition());
//        telemetry.addData("br", "%d", _backRight.getCurrentPosition());
        telemetry.addData("bls", "%f", _backLeftServo.getPosition());
        telemetry.addData("brs", "%f", _backRightServo.getPosition());

        telemetry.update();
    }
}
