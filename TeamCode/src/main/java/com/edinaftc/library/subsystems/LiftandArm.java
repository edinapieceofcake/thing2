package com.edinaftc.library.subsystems;

import com.edinaftc.library.PIDController;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class LiftandArm extends Subsystem{
    private int liftIndex = 1;
    private boolean autoLiftLocation = false;
    private int liftLocation;
    private DcMotor leftLift;
    private DcMotor rightLift;
    private DcMotor armEncoder;
    private CRServo arm;
    private double liftPower, armPower;
    private boolean autoArmLocation = false;

    public static int TARGETPOSITION = 15000;

    public LiftandArm(HardwareMap map) {
        leftLift = map.dcMotor.get("leftLift");
        rightLift = map.dcMotor.get("rightLift");
        leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightLift.setDirection(DcMotorSimple.Direction.REVERSE);

        arm = map.crservo.get("crarm");

        armEncoder = map.dcMotor.get("il");
        armEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    @Override
    public void update() {
        if (autoLiftLocation) {
            int currentPosition = rightLift.getCurrentPosition();
            int difference = (liftLocation - currentPosition);
            double motorPower = difference * .75/liftLocation;
            if (motorPower < .25) {
                autoLiftLocation = false;
            } else {
                leftLift.setPower(motorPower);
                rightLift.setPower(motorPower);
            }
        } else {
            leftLift.setPower(this.liftPower);
            rightLift.setPower(this.liftPower);
        }

        if (autoArmLocation) {
            int currentPosition = armEncoder.getCurrentPosition();
            double motorPower = (TARGETPOSITION - currentPosition) * .75/TARGETPOSITION + .2;
            if (motorPower < .25) {
                autoArmLocation = false;
            } else {
                arm.setPower(motorPower);
            }
        } else {
            arm.setPower(armPower);
        }
    }

    public void setLiftPower(double liftPower, boolean downPressed) {
        if(liftPower > 0) {
            if (downPressed) {
                this.liftPower = -liftPower;
            } else {
                this.liftPower = -liftPower*.5;
            }
        } else {
            this.liftPower = -liftPower;
        }
        if (liftPower != 0) {
            autoLiftLocation = false;
        }
    }

    public void displayTelemetry(Telemetry telemetry) {
        telemetry.addData("lift position, power", "%d %f", rightLift.getCurrentPosition(), leftLift.getPower());
        telemetry.addData("arm position, power", "%d %f", armEncoder.getCurrentPosition(), arm.getPower());
        telemetry.addData("auto on, location", "%s %d", autoLiftLocation, liftLocation);
    }

    public void setArmPower(double armPower, boolean downPressed) {
        if (downPressed) {
            this.armPower = -.8 * armPower;
        } else {
            this.armPower = -.8 * armPower * .4;
        }

        if (armPower != 0) {
            autoLiftLocation = false;
            autoArmLocation = false;
        }
    }

    public void armForBlock() {
        autoArmLocation = true;
    }

    public void increaseHHeight() {
        liftIndex++;
        computeLocation();
    }

    public void resetHeight() {
        liftIndex = 0;
    }

    public void decreaseHeight() {
        if (liftIndex > 0) {
            liftIndex--;
        }

        computeLocation();
    }

    private void computeLocation() {
        if (liftIndex == 0) {
            liftLocation = 0;
        } else if (liftIndex ==  1){
            liftLocation = 650;
        } else {
            liftLocation = 650 + (750 * (liftIndex -1));
        }

        autoLiftLocation = true;
    }
}
