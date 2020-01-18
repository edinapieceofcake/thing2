package com.edinaftc.library.subsystems;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class IMU extends Subsystem {
    private BNO055IMU imu = null;
    private Telemetry telemetry = null;
    private double previousTime;
    private double derivative = 0;
    private double integral = 0;
    private double previousError = 0;
    private double Kp = 0.01, Ki = 0.001, Kd = .5; // PID constant multipliers
    private double endAngle = 0;
    private double currentAngle = 0;
    private double previousOutput = 0;
    private double output = 0;
    private double error = 0;
    private double percentOff = 0;
    private double left = 0;
    private double right = 0;
    private double errorPercent;

    public IMU(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imu.initialize(parameters);
    }

    private double GetImuAngle() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double currentAngle = angles.firstAngle;

        return currentAngle;
    }

    public void updateOutput() {
        double currentTime = System.currentTimeMillis();
        double difference = (currentTime - previousTime);
        // basic pid code for getting the angle and computing the output
        // for motor speed
        currentAngle = GetImuAngle();

        error = endAngle - currentAngle;
        integral = integral + (error * difference);
        integral = Range.clip(integral, -1, 1);
        derivative = (error - previousError) / difference;

        previousOutput = output;
        output = (Kp * error) + (Ki * integral) + (Kd * derivative);

        previousError = error;
        previousTime = currentTime;
    }

    public void update() {
        if (percentOff > errorPercent) {
            updateOutput();

            left = -Range.clip(output, -1, 1) * .60;
            right = Range.clip(output, -1, 1) * .60;

            percentOff = Math.abs((currentAngle - endAngle) / endAngle);
        }

        telemetry.addData("C, E, V Angle", "%f, %f, %f", GetImuAngle(), endAngle,
                percentOff);
        telemetry.addData("Proportional", "%f %f",  error, Kp * error);
        telemetry.addData("Integral", "%f %f", integral, Ki * integral);
        telemetry.addData("Derivative", "%f %f", derivative,
                Kd * derivative);
        telemetry.addData("Left Power: ", "%f", left);
        telemetry.addData("Right Power: ", "%f", right);
    }

    public void setEndAngle(double endAngle) {
        this.endAngle = endAngle;
        currentAngle = GetImuAngle();
        percentOff = Math.abs((currentAngle - this.endAngle) / this.endAngle);
    }

    public void start(double errorPercent) {
        this.errorPercent = errorPercent;
    }
}
