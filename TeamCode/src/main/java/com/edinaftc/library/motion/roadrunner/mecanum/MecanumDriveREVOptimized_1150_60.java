package com.edinaftc.library.motion.roadrunner.mecanum;

import android.support.annotation.NonNull;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.edinaftc.library.motion.roadrunner.util.AxesSigns;
import com.edinaftc.library.motion.roadrunner.util.BNO055IMUUtil;
import com.edinaftc.library.motion.roadrunner.util.LynxModuleUtil;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.RevBulkData;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import static com.edinaftc.library.motion.roadrunner.mecanum.DriveConstants_1150_60.MOTOR_VELO_PID;
import static com.edinaftc.library.motion.roadrunner.mecanum.DriveConstants_1150_60.RUN_USING_ENCODER;
import static com.edinaftc.library.motion.roadrunner.mecanum.DriveConstants_1150_60.encoderTicksToInches;

/*
 * Optimized mecanum drive implementation for REV ExHs. The time savings may significantly improve
 * trajectory following performance with moderate additional complexity.
 */
public class MecanumDriveREVOptimized_1150_60 extends MecanumDriveBase_1150_60 {
    private ExpansionHubEx hub9;
    private ExpansionHubEx hub2;
    private ExpansionHubMotor leftFront, leftRear, rightRear, rightFront;
    private List<ExpansionHubMotor> motors;
    private BNO055IMU imu;

    public MecanumDriveREVOptimized_1150_60(HardwareMap hardwareMap) {
        super();

        LynxModuleUtil.ensureMinimumFirmwareVersion(hardwareMap);

        // TODO: adjust the names of the following hardware devices to match your configuration
        // for simplicity, we assume that the desired IMU and drive motors are on the same hub
        // if your motors are split between hubs, **you will need to add another bulk read**
        hub2 = hardwareMap.get(ExpansionHubEx.class, "hub2");
        hub9 = hardwareMap.get(ExpansionHubEx.class, "hub9");

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        // TODO: if your hub is mounted vertically, remap the IMU axes so that the z-axis points
        // upward (normal to the floor) using a command like the following:
        BNO055IMUUtil.remapAxes(imu, AxesOrder.XYZ, AxesSigns.NPN);

        leftFront = hardwareMap.get(ExpansionHubMotor.class, "fl");
        leftRear = hardwareMap.get(ExpansionHubMotor.class, "bl");
        rightRear = hardwareMap.get(ExpansionHubMotor.class, "br");
        rightFront = hardwareMap.get(ExpansionHubMotor.class, "fr");

        motors = Arrays.asList(leftFront, leftRear, rightRear, rightFront);

        for (ExpansionHubMotor motor : motors) {
            if (RUN_USING_ENCODER) {
                motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        if (RUN_USING_ENCODER && MOTOR_VELO_PID != null) {
            setPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, MOTOR_VELO_PID);
        }

        // TODO: reverse any motors using DcMotor.setDirection()
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);

        // TODO: if desired, use setLocalizer() to change the localization method
        //setLocalizer(new StandardTrackingWheelLocalizer(hardwareMap));
    }

    @Override
    public PIDCoefficients getPIDCoefficients(DcMotor.RunMode runMode) {
        PIDFCoefficients coefficients = leftFront.getPIDFCoefficients(runMode);
        return new PIDCoefficients(coefficients.p, coefficients.i, coefficients.d);
    }

    @Override
    public void setPIDCoefficients(DcMotor.RunMode runMode, PIDCoefficients coefficients) {
        for (ExpansionHubMotor motor : motors) {
            motor.setPIDFCoefficients(runMode, new PIDFCoefficients(
                    coefficients.kP, coefficients.kI, coefficients.kD, DriveConstants_1150_60.getMotorVelocityF()
            ));
        }
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        RevBulkData bulkData2 = hub2.getBulkInputData();
        RevBulkData bulkData9 = hub9.getBulkInputData();

        if (bulkData2 == null || bulkData9 == null) {
            return Arrays.asList(0.0, 0.0, 0.0, 0.0);
        }

        List<Double> wheelPositions = new ArrayList<>();
        wheelPositions.add(encoderTicksToInches(bulkData2.getMotorCurrentPosition(leftFront)));
        wheelPositions.add(encoderTicksToInches(bulkData2.getMotorCurrentPosition(leftRear)));
        wheelPositions.add(encoderTicksToInches(bulkData9.getMotorCurrentPosition(rightRear)));
        wheelPositions.add(encoderTicksToInches(bulkData9.getMotorCurrentPosition(rightFront)));
        return wheelPositions;
    }

    @Override
    public List<Double> getWheelVelocities() {
        RevBulkData bulkData2 = hub2.getBulkInputData();
        RevBulkData bulkData9 = hub9.getBulkInputData();

        if (bulkData2 == null || bulkData9 == null) {
            return Arrays.asList(0.0, 0.0, 0.0, 0.0);
        }

        List<Double> wheelVelocities = new ArrayList<>();
        wheelVelocities.add(encoderTicksToInches(bulkData2.getMotorVelocity(leftFront)));
        wheelVelocities.add(encoderTicksToInches(bulkData2.getMotorVelocity(leftRear)));
        wheelVelocities.add(encoderTicksToInches(bulkData9.getMotorVelocity(rightRear)));
        wheelVelocities.add(encoderTicksToInches(bulkData9.getMotorVelocity(rightFront)));
        return wheelVelocities;
    }

    @Override
    public void setMotorPowers(double v, double v1, double v2, double v3) {
        leftFront.setPower(v);
        leftRear.setPower(v1);
        rightRear.setPower(v2);
        rightFront.setPower(v3);
    }

    @Override
    public double getRawExternalHeading() {
        return imu.getAngularOrientation().firstAngle;
    }
}
