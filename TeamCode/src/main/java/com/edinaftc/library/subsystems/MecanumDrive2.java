package com.edinaftc.library.subsystems;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class MecanumDrive2 extends Subsystem{

    private DcMotorEx[] motors;
    public static final String[] MOTOR_NAMES = {"fl", "bl", "br", "fr"};
    private double[] powers;
    private double leftStickX;
    private double leftStickY;
    private double rightStickY;
    private double leftTrigger;
    private double rightTrigger;
    private double driveStickSpeed = 1.0;
    private double rotationStickSpeed = 1.0;

    private double currentPower = 1.4;

    public MecanumDrive2(HardwareMap map) {
        powers = new double[4];
        motors = new DcMotorEx[4];

        for (int i = 0; i < 4; i ++) {
            DcMotorEx dcMotor = map.get(DcMotorEx.class, MOTOR_NAMES[i]);
            motors[i] = dcMotor;
            motors[i].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motors[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        motors[2].setDirection(DcMotorSimple.Direction.REVERSE);
        motors[3].setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void setVelocity(double leftStickX, double leftStickY, double rightStickY, double leftTrigger, double rightTrigger,
                            boolean leftStickPressed, boolean rightStickPressed) {
        this.leftStickX = leftStickX;
        this.leftStickY = leftStickY;
        this.rightStickY = rightStickY;
        this.leftTrigger = leftTrigger;
        this.rightTrigger = rightTrigger;

        if (leftStickPressed || rightStickPressed) {
            rotationStickSpeed = .7;
            driveStickSpeed = 1;
        } else {
            driveStickSpeed = rotationStickSpeed = .4;
        }
    }

    public void update() {
        double x;
        double y;
        double rotation;
        double speed;

        if (leftStickX != 0 || leftStickY != 0 || rightStickY != 0) {
            x = Math.pow(-leftStickX, 3.0);
            y = Math.pow(leftStickY, 3.0);
            rotation = Math.pow(-rightStickY, 3.0) * rotationStickSpeed;
            speed = Math.min(1.0, Math.sqrt(x * x + y * y)) * driveStickSpeed;
        } else if (rightTrigger != 0) {
            x = Math.pow(rightTrigger, 3.0);
            y = 0;
            rotation = 0;
            speed = .4;
        } else if (leftTrigger != 0) {
            x = Math.pow(-leftTrigger, 3.0);
            y = 0;
            rotation = 0;
            speed = .4;
        } else {
            x = 0;
            y = 0;
            rotation = 0;
            speed = 0;
        }

        final double direction = Math.atan2(x, y);

        powers[0] = (speed * Math.sin(direction + Math.PI / 4.0) + rotation) * currentPower;
        powers[3] = (speed * Math.cos(direction + Math.PI / 4.0) - rotation) * currentPower;
        powers[1] = (speed * Math.cos(direction + Math.PI / 4.0) + rotation) * currentPower;
        powers[2] = (speed * Math.sin(direction + Math.PI / 4.0) - rotation) * currentPower;

        for (int i = 0; i < 4; i++) {
            motors[i].setPower(powers[i]);
        }
    }

    public void displayTelemetry(Telemetry telemetry) {
        for (int i = 0; i < 4; i++) {
            telemetry.addData(String.format("%s: position %d power %f", MOTOR_NAMES[i], motors[i].getCurrentPosition(), motors[i].getPower()), "");
        }

        telemetry.addData("x, y, r", "%f %f %f", leftStickX, leftStickY, rightStickY);
        telemetry.addData("lt rt", "%f %f", leftTrigger, rightTrigger);
    }
}
