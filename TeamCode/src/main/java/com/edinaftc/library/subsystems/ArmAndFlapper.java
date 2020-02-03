package com.edinaftc.library.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ArmAndFlapper extends Subsystem {
    private Servo _leftarm;
    private Servo _rightarm;
    private Servo _leftflap;
    private Servo _rightflap;
    private boolean _canUpdate = false;

    public ArmAndFlapper(HardwareMap map) {
        _rightarm = map.servo.get("rightArm");
        _leftarm = map.servo.get("leftArm");
        _leftflap = map.servo.get("leftFlap");
        _rightflap = map.servo.get("rightFlap");
    }

    public void turnOnUpdate() {
        _canUpdate = true;
    }

    @Override
    public void update() {
        if (_canUpdate) {
            _leftarm.setPosition(1);
            _leftflap.setPosition(0);
            _rightarm.setPosition(0);
            _rightflap.setPosition(1);
        }
    }

    public void displayTelemetry(Telemetry telemetry) {
        telemetry.addData("larm", "%d", _leftarm.getPosition());
        telemetry.addData("lfl", "%d", _leftflap.getPosition());
        telemetry.addData("rarm", "%d", _rightarm.getPosition());
        telemetry.addData("rfl", "%d", _rightflap.getPosition());
    }
}
