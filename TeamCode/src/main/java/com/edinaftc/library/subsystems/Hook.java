package com.edinaftc.library.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Hook extends Subsystem {
    private Servo _left;
    private Servo _right;
    private double _leftPosition = .7;
    private double _rightPosition = .17;
    private boolean _canUpdate = false;

    public Hook(HardwareMap map) {
        _left = map.servo.get("blhook");
        _right = map.servo.get("brhook");
    }

    @Override
    public void update() {
        if (_canUpdate) {
            _left.setPosition(_leftPosition);
            _right.setPosition(_rightPosition);
        }
    }

    public void dropHooks() {
        _leftPosition = .3;
        _rightPosition = .6;
    }


    public void liftHooks() {
        _leftPosition = .7;
        _rightPosition = .17;
    }

    public void turnOnUpdate() {
        _canUpdate = true;
    }
}
