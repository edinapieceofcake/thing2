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

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

@TeleOp()
@Disabled
public class MecanumTestEncoders extends LinearOpMode {
    private Mecanum _mecanum;
    private Stickygamepad _gamepad1;
    private BNO055IMU _imu = null;


    public void runOpMode() {
        int moveDistance = 300;
        int counter = 0;
        String[] messages = new String[]{ "\\", "|", "/", "-", "\\", "|", "/", "-" };
        Orientation angles;

        _mecanum = new Mecanum(hardwareMap.dcMotor.get("fl"), hardwareMap.dcMotor.get("fr"),
        hardwareMap.dcMotor.get("bl"),hardwareMap.dcMotor.get("br"), telemetry);
        _gamepad1 = new Stickygamepad(gamepad1);
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

        waitForStart();

        while (this.opModeIsActive()) {
            _gamepad1.update();
            _mecanum.Drive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);

            if (_gamepad1.a) {
                _mecanum.MoveBackwardsRunWithEncoders(.5, moveDistance, this);
            } else if (_gamepad1.x) {
                _mecanum.TurnLeftRunWithEncoders(.5, moveDistance, this);
            } else if (_gamepad1.b) {
                _mecanum.TurnRightRunWithEncoders(.5, moveDistance, this);
            } else if (_gamepad1.y) {
                _mecanum.MoveForwardRunWithEncoders(.5, moveDistance, this);
            } else if (_gamepad1.dpad_down) {
                _mecanum.DiagonalLeftAndDownRunToPosition(.5, moveDistance, this);
            } else if (_gamepad1.dpad_up) {
                _mecanum.DiagonalRightAndUpRunToPosition(.5, moveDistance, this);
            } else if (_gamepad1.dpad_left) {
                _mecanum.DiagonalLeftAndUpRunToPosition(.5, moveDistance, this);
            } else if (_gamepad1.dpad_right) {
                _mecanum.DiagonalRightAndDownRunToPosition(.5, moveDistance, this);
            } else if (_gamepad1.left_bumper) {
                _mecanum.SlideLeftRunWithEncoders(.5, moveDistance, this);
            } else if (_gamepad1.right_bumper) {
                _mecanum.SlideRightRunWithEncoders(.5, moveDistance, this);
            }

            angles = _imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            telemetry.addData("first angle", angles.firstAngle);
            telemetry.addData("second angle", angles.secondAngle);
            telemetry.addData("third angle", angles.thirdAngle);
            telemetry.addData("fl", "%d", hardwareMap.dcMotor.get("fl").getCurrentPosition());
            telemetry.addData("fr", "%d", hardwareMap.dcMotor.get("fr").getCurrentPosition());
            telemetry.addData("bl", "%d", hardwareMap.dcMotor.get("bl").getCurrentPosition());
            telemetry.addData("br", "%d", hardwareMap.dcMotor.get("br").getCurrentPosition());
            telemetry.update();
        }
    }
}
