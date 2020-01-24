package com.edinaftc.library;

import com.qualcomm.robotcore.util.Range;

public class PIDController {
    private double p;
    private double i;
    private double d;
    private double max;
    private double min;
    private double setPoint;
    private double integral;
    private double previousError;
    private long previousTime;

    public PIDController(double P, double I, double D, double max, double min) {
        p = P;
        i = I;
        d = D;
        this.max = max;
        this.min = min;
    }

    public void setSetPoint(double setPoint) {
        this.setPoint = setPoint;
    }

    public void start() throws InterruptedException{
        integral = 0;
        previousError = 0;
        previousTime = System.currentTimeMillis();
        Thread.sleep(100);
    }

    public double update(double currentValue) {
        double answer = 0;
        long currentTime = System.currentTimeMillis();
        long diffTime = currentTime - previousTime;

        double error = setPoint - currentValue;
        double derivative = d * (error - previousError) / diffTime;
        integral += i * error * diffTime;
        integral = Range.clip(integral, min, max);

        answer = p * error + integral + derivative;

        answer = Range.clip(answer, min, max);
        previousTime = currentTime;
        previousError = error;

        return answer;
    }
}
