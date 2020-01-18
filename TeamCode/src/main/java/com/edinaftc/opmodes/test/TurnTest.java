package com.edinaftc.opmodes.test;

import com.edinaftc.library.Stickygamepad;
import com.edinaftc.library.motion.Mecanum;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp
@Disabled
public class TurnTest extends LinearOpMode {
    private Mecanum driveTrain;
    private Stickygamepad _gamepad1;
    private double _speedModifier = -25;

    @Override
    public void runOpMode() {
        driveTrain = new Mecanum(hardwareMap.dcMotor.get("fl"), hardwareMap.dcMotor.get("fr"), hardwareMap.dcMotor.get("bl"), hardwareMap.dcMotor.get("br"), telemetry);
        driveTrain.StopResetEncodersRunWithEncoderAndBrakekOn();
        _gamepad1 = new Stickygamepad(gamepad1);

        waitForStart();

        while (opModeIsActive()) {
            _gamepad1.update();
            if (_gamepad1.left_bumper) {
                _speedModifier--;
            } else if (_gamepad1.right_bumper) {
                _speedModifier++;
            }

            if (_gamepad1.x) {
                driveTrain.TurnToHeading(270, _speedModifier, telemetry);
            } else if (_gamepad1.y) {
                driveTrain.TurnToHeading(0, _speedModifier, telemetry);
            } else if (_gamepad1.b) {
                driveTrain.TurnToHeading(90, _speedModifier, telemetry);
            } else if (_gamepad1.a) {
                driveTrain.TurnToHeading(180, _speedModifier, telemetry);
            } else if (_gamepad1.dpad_left) {
                driveTrain.TurnLeftRunToPosition(.5, 1415, this);
            }

            Orientation angles = driveTrain.angles;

            telemetry.addData("first, second, third angles", "%f, %f, %f,", angles.firstAngle, angles.secondAngle, angles.thirdAngle);
            telemetry.addData("speed modifier", "%f", _speedModifier);
            telemetry.update();
        }
    }
}

