package com.edinaftc.library.subsystems;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.MotorControlAlgorithm;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class MecanumDrive2 extends Subsystem{

    private DcMotorEx[] motors;
    public static final String[] MOTOR_NAMES = {"fl", "bl", "br", "fr"};
    private boolean slowMode = false;
    private double[] powers;
    private double leftStickX;
    private double leftStickY;
    private double rightStickY;
    private double leftTrigger;
    private double rightTrigger;
    private PIDFCoefficients normalVelocityPID = null;
    private PIDFCoefficients slowVelocityPID = new PIDFCoefficients(10, 3, 1, 0, MotorControlAlgorithm.LegacyPID);

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

        normalVelocityPID = motors[0].getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void setVelocity(double leftStickX, double leftStickY, double rightStickY, double leftTrigger, double rightTrigger) {
        this.leftStickX = leftStickX;
        this.leftStickY = leftStickY;
        this.rightStickY = rightStickY;
        this.leftTrigger = leftTrigger;
        this.rightTrigger = rightTrigger;
    }

    public void update() {
        double x;
        double y;
        double rotation;

        if (leftStickX != 0 || leftStickY != 0 || rightStickY != 0) {
            x = Math.pow(-leftStickX, 3.0);
            y = Math.pow(leftStickY, 3.0);
            rotation = Math.pow(-rightStickY, 3.0);
        } else if (rightTrigger != 0) {
            x = Math.pow(rightTrigger, 3.0) * .3;
            y = 0;
            rotation = 0;
        } else if (leftTrigger != 0) {
            x = Math.pow(-leftTrigger, 3.0) * .3;
            y = 0;
            rotation = 0;
        } else {
            x = 0;
            y = 0;
            rotation = 0;
        }

        final double direction = Math.atan2(x, y);
        final double speed = Math.min(1.0, Math.sqrt(x * x + y * y));

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

        PIDFCoefficients currentPIDF = motors[0].getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry.addData("x, y, r", "%f %f %f", leftStickX, leftStickY, rightStickY);
        telemetry.addData("lt rt", "%f %f", leftTrigger, rightTrigger);
        telemetry.addData("Slow, P, I, D, F", "%s %f %f %f %f", slowMode, currentPIDF.p, currentPIDF.i, currentPIDF.d, currentPIDF.f);
    }

    public void togglePID() {
        if (slowMode) {
            for (int i = 0; i < 4; i ++) {
                motors[i].setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, normalVelocityPID);
            }

            currentPower = 1.4;

            slowMode = false;
        } else {
            for (int i = 0; i < 4; i ++) {
                motors[i].setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, slowVelocityPID);
            }

            currentPower = .25;
            slowMode = true;
        }
    }

    public void Move(double fl, double fr, double bl, double br) {
        motors[0].setPower(fl);
        motors[3].setPower(fr);
        motors[1].setPower(bl);
        motors[2].setPower(br);
    }

    public void Stop() {
        motors[0].setPower(0);
        motors[3].setPower(0);
        motors[1].setPower(0);
        motors[2].setPower(0);
    }

}
