package com.edinaftc.library.subsystems;


import com.edinaftc.library.Vector2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import java.util.Arrays;
import java.util.Collections;

public class MecanumDrive extends Subsystem{

    private DcMotorEx[] motors;
    public static final String[] MOTOR_NAMES = {"fl", "bl", "br", "fr"};
    private double[] powers;
    private Vector2d targetVel = new Vector2d(0, 0);
    private double targetOmega = 0;

    public MecanumDrive(HardwareMap map) {
        powers = new double[4];
        motors = new DcMotorEx[4];
        for (int i = 0; i < 4; i++) {
            DcMotorEx dcMotorEx = map.get(DcMotorEx.class, MOTOR_NAMES[i]);
            motors[i] = dcMotorEx;
            //motors[i].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motors[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        motors[2].setDirection(DcMotorSimple.Direction.REVERSE);
        motors[3].setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public DcMotorEx[] getMotors() { return motors; }

    public void setVelocity(Vector2d vel, double omega) {
        internalSetVelocity(vel, omega);
    }

    private void internalSetVelocity(Vector2d vel, double omega) {
        this.targetVel = vel;
        this.targetOmega = omega;
    }


    private void updatePowers() {
        powers[0] = targetVel.x() - targetVel.y() - targetOmega;
        powers[1] = targetVel.x() + targetVel.y() - targetOmega;
        powers[2] = targetVel.x() - targetVel.y() + targetOmega;
        powers[3] = targetVel.x() + targetVel.y() + targetOmega;

        double max = Collections.max(Arrays.asList(1.0, Math.abs(powers[0]),
                Math.abs(powers[1]), Math.abs(powers[2]), Math.abs(powers[3])));

        for (int i = 0; i < 4; i++) {
            powers[i] /= max;
        }
    }

    public void update() {

        updatePowers();

        for (int i = 0; i < 4; i++) {
            motors[i].setPower(powers[i]);
        }

    }
}




