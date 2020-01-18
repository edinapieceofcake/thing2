package com.edinaftc.opmodes.test;

import com.edinaftc.skystone.IMURobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "IMUTeleop", group = "teleop")
@Disabled
public class IMUTeleOp extends OpMode {
    private IMURobot robot;

    public void init() {
        robot = new IMURobot(this, telemetry);
        robot.start();
    }

    public void start() {
        robot.imu.setEndAngle(90);
        robot.imu.start(.1);
    }

    public void loop() {
        robot.drive.setVelocity(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, gamepad2.left_trigger, gamepad2.right_trigger);
    }
}
