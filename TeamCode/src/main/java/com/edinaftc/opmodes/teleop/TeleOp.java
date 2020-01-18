package com.edinaftc.opmodes.teleop;

import com.edinaftc.library.Stickygamepad;
import com.edinaftc.skystone.Robot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Teleop", group = "teleop")
public class TeleOp extends OpMode {
    private Robot robot;
    private Stickygamepad _gamepad1;
    private Stickygamepad _gamepad2;

    public void init() {
        _gamepad1 = new Stickygamepad(gamepad1);
        _gamepad2 = new Stickygamepad(gamepad2);
        robot = new Robot(this, telemetry);
        robot.start();
    }

    @Override
    public void start() {
//        robot.hook.turnOnUpdate();
//        robot.grabber.turnOnUpdate();
//        robot.arm.turnOnUpdate();
    }

    public void loop() {

        _gamepad1.update();
        _gamepad2.update();
/*
        robot.drive.setVelocity(gamepad1.left_stick_x, gamepad1.left_stick_y,
                gamepad1.right_stick_x, gamepad2.left_trigger, gamepad2.right_trigger);

        if (_gamepad1.left_bumper) {
            robot.intake.toggleIntake();
        } else if (_gamepad1.right_bumper) {
            robot.intake.toggleExpel();
        }

        if (gamepad1.left_trigger > 0) {
            robot.hook.liftHooks();
        }

        if (gamepad1.right_trigger > 0) {
            robot.hook.dropHooks();
        }

        if (_gamepad1.right_stick_button) {
            robot.drive.togglePID();
        }
*/
        robot.liftandarm.setLiftPower(gamepad2.left_stick_y);
        robot.liftandarm.setArmPower(gamepad2.right_stick_y);
/*
        if(_gamepad2.right_bumper) {
            robot.grabber.toggleBothGrabbers();
        }

        if(_gamepad2.left_bumper) {
            robot.grabber.loadBlock();
        }

        if (_gamepad2.dpad_up) {
            robot.liftandarm.increaseHHeight();
        }

        if (_gamepad2.dpad_down) {
            robot.liftandarm.decreaseHeight();
        }

        if (_gamepad2.right_stick_button) {
            robot.liftandarm.toggleArmPower();
        }

        if (_gamepad2.x && _gamepad2.b) {
            robot.grabber.dropCapstone();
        }

        robot.drive.displayTelemetry(telemetry);

 */
        robot.liftandarm.displayTelemetry(telemetry);

        telemetry.update();
    }

    @Override
    public  void stop() {
        robot.stop();
    }
}
