package com.edinaftc.opmodes.test;

import com.edinaftc.library.Stickygamepad;
import com.edinaftc.library.motion.Mecanum;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp()
@Disabled
public class MecanumTestEncodersAndImu extends LinearOpMode {
    private Mecanum _mecanum;
    private Stickygamepad _gamepad1;

    public void runOpMode() {
        int moveDistance = 300;
        int counter = 0;
        String[] messages = new String[]{ "\\", "|", "/", "-", "\\", "|", "/", "-" };
        Orientation angles;

        _mecanum = new Mecanum(hardwareMap.dcMotor.get("fl"), hardwareMap.dcMotor.get("fr"),
        hardwareMap.dcMotor.get("bl"),hardwareMap.dcMotor.get("br"), telemetry);
        _mecanum.enableIMU(hardwareMap.get(BNO055IMU.class, "imu"), this);
        _mecanum.startIMU();
        _gamepad1 = new Stickygamepad(gamepad1);

        waitForStart();

        while (this.opModeIsActive()) {
            _gamepad1.update();
            _mecanum.Drive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);

            if (_gamepad1.a) {
                _mecanum.MoveBackwardRunWithEncodersAndIMU(1, 20 * moveDistance, .2, 0, this, telemetry);
            } else if (_gamepad1.x) {
            } else if (_gamepad1.b) {
            } else if (_gamepad1.y) {
                _mecanum.MoveForwardRunWithEncodersAndIMU(1, 20 * moveDistance, .2, 0, this, telemetry);
            } else if (_gamepad1.dpad_down) {
            } else if (_gamepad1.dpad_up) {
            } else if (_gamepad1.dpad_left) {
            } else if (_gamepad1.dpad_right) {
            } else if (_gamepad1.left_bumper) {
            } else if (_gamepad1.right_bumper) {
            }

            telemetry.addData("first angle", _mecanum.angles.firstAngle);
            telemetry.addData("second angle", _mecanum.angles.secondAngle);
            telemetry.addData("third angle", _mecanum.angles.thirdAngle);
            telemetry.addData("fl", "%d", hardwareMap.dcMotor.get("fl").getCurrentPosition());
            telemetry.addData("fr", "%d", hardwareMap.dcMotor.get("fr").getCurrentPosition());
            telemetry.addData("bl", "%d", hardwareMap.dcMotor.get("bl").getCurrentPosition());
            telemetry.addData("br", "%d", hardwareMap.dcMotor.get("br").getCurrentPosition());
            telemetry.update();
        }

        _mecanum.stopIMU();
    }
}
