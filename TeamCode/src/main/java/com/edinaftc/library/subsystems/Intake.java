package com.edinaftc.library.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake extends Subsystem {

    private DcMotor leftIntake, rightIntake;
    private CRServo leftIntakeServo, rightIntakeServo;
    private double motorPower;
    private double servoPower;
    private boolean intakeOn;
    private boolean expelOn;

    public Intake(HardwareMap map) {
        leftIntake = map.dcMotor.get("il");
        rightIntake = map.dcMotor.get("ir");
        leftIntakeServo = map.crservo.get("isl");
        rightIntakeServo = map.crservo.get("isr");
        rightIntake.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void update() {
        leftIntake.setPower(motorPower);
        rightIntake.setPower(-motorPower);
        leftIntakeServo.setPower(-servoPower);
        rightIntakeServo.setPower(servoPower);
    }


    public void toggleIntake() {
        if (expelOn || intakeOn) {
            motorPower = 0;
            servoPower = 0;
            expelOn = false;
            intakeOn = false;
        }
        else
        {
            intakeOn = true;
            motorPower = 1;
            servoPower = -.8;
        }
    }

    public void toggleExpel() {
        if (intakeOn || expelOn) {
            motorPower = 0;
            servoPower = 0;
            expelOn = false;
            intakeOn = false;
        } else {
            expelOn = true;
            motorPower = -1;
            servoPower = .8;
        }
    }
}
