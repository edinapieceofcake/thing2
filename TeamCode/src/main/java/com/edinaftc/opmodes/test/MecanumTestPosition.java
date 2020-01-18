package com.edinaftc.opmodes.test;

import com.edinaftc.library.Stickygamepad;
import com.edinaftc.library.motion.Mecanum;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp()
@Disabled
public class MecanumTestPosition extends LinearOpMode {
    Mecanum _mecanum;
    Stickygamepad _gamepad1;

    public void runOpMode() {
        int moveDistance = 300;
        _mecanum = new Mecanum(hardwareMap.dcMotor.get("fl"), hardwareMap.dcMotor.get("fr"),
                hardwareMap.dcMotor.get("bl"),hardwareMap.dcMotor.get("br"), telemetry);
        _gamepad1 = new Stickygamepad(gamepad1);

        waitForStart();

        while (this.opModeIsActive()) {
            _gamepad1.update();
            _mecanum.Drive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);

            if (_gamepad1.a) {
                _mecanum.MoveBackwardsRunToPosition(.75, moveDistance, this);
            } else if (_gamepad1.x) {
                _mecanum.TurnLeftRunToPosition(.75, moveDistance, this);
            } else if (_gamepad1.b) {
                _mecanum.TurnRightRunToPosition(.75, moveDistance, this);
            } else if (_gamepad1.y) {
                _mecanum.MoveForwardRunToPosition(.75, moveDistance, this);
            } else if (_gamepad1.dpad_down) {
                _mecanum.DiagonalLeftAndDownRunToPosition(.75, moveDistance, this);
            } else if (_gamepad1.dpad_up) {
                _mecanum.DiagonalRightAndUpRunToPosition(.75, moveDistance, this);
            } else if (_gamepad1.dpad_left) {
                _mecanum.DiagonalLeftAndUpRunToPosition(.75, moveDistance, this);
            } else if (_gamepad1.dpad_right) {
                _mecanum.DiagonalRightAndDownRunToPosition(.75, moveDistance, this);
            } else if (gamepad1.right_trigger != 0) {
                _mecanum.SlideRightRunToPosition(.75, moveDistance, this);
            } else if (gamepad1.left_trigger != 0) {
                _mecanum.SlideLeftRunToPosition(.75, moveDistance, this);
            }

            telemetry.addData("fl", "%d", hardwareMap.dcMotor.get("fl").getCurrentPosition());
            telemetry.addData("fr", "%d", hardwareMap.dcMotor.get("fr").getCurrentPosition());
            telemetry.addData("bl", "%d", hardwareMap.dcMotor.get("bl").getCurrentPosition());
            telemetry.addData("br", "%d", hardwareMap.dcMotor.get("br").getCurrentPosition());
            telemetry.addData("fl p", "%f", hardwareMap.dcMotor.get("fl").getPower());
            telemetry.addData("fr p", "%f", hardwareMap.dcMotor.get("fr").getPower());
            telemetry.addData("bl p", "%f", hardwareMap.dcMotor.get("bl").getPower());
            telemetry.addData("br p", "%f", hardwareMap.dcMotor.get("br").getPower());
            telemetry.update();
        }
    }
}
