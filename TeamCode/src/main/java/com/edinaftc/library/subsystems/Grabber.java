package com.edinaftc.library.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Grabber extends Subsystem {
    private boolean frontGrabberOpen = false;
    private boolean backGrabberOpen = true;
    private Servo front;
    private Servo back;
    private Servo capstone;
    private DcMotor lift;
    private DistanceSensor sensorRange;
    private boolean _canUpdate = false;
    private boolean grabberClosed = false;
    private boolean dropCapstone = false;

    public Grabber(HardwareMap map) {
        front = map.servo.get("fg");
        back = map.servo.get("bg");
        capstone = map.servo.get("capstoneservo");
        lift = map.dcMotor.get("lift");
        sensorRange = map.get(DistanceSensor.class, "blockdetector");
    }

    public void update() {
        if (_canUpdate) {
            if ((sensorRange.getDistance(DistanceUnit.CM) < 3) && !grabberClosed && (lift.getCurrentPosition() < 20)) {
                frontGrabberOpen = false;
                backGrabberOpen = false;
                grabberClosed = true;
            }

            if (grabberClosed && (sensorRange.getDistance(DistanceUnit.CM) > 3))
            {
                grabberClosed = false;
            }

            if (frontGrabberOpen) {
                front.setPosition(1);
            } else {
                front.setPosition(0);
            }

            if (backGrabberOpen) {
                back.setPosition(1);
            } else {
                back.setPosition(0);
            }

            if (dropCapstone) {
                capstone.setPosition(.75);
            }
        }
    }

    public void turnOnUpdate() {
        _canUpdate = true;
    }

    public void toggleBothGrabbers() {
        if(frontGrabberOpen == false && backGrabberOpen == false) {
            frontGrabberOpen = true;
            backGrabberOpen = true;
        } else {
            frontGrabberOpen = false;
            backGrabberOpen = false;
        }
    }

    public void dropCapstone() {
        dropCapstone = true;
    }

    public void loadBlock() {
        frontGrabberOpen = false;
        backGrabberOpen = true;
    }
}
