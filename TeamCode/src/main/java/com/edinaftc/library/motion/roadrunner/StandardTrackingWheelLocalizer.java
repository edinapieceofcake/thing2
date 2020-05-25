package com.edinaftc.library.motion.roadrunner;

import android.support.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.RevBulkData;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import static com.edinaftc.library.motion.roadrunner.mecanum.DriveConstants_435_60.encoderTicksToInches;

/*
 * Sample tracking wheel localizer implementation assuming the standard configuration:
 *
 *    /--------------\
 *    |     ____     |
 *    |     ----     |
 *    | ||        || |
 *    | ||        || |
 *    |              |
 *    |              |
 *    \--------------/
 *
 */
@Config
public class StandardTrackingWheelLocalizer extends ThreeTrackingWheelLocalizer {
    public static double TICKS_PER_REV = 8192;
    public static double WHEEL_RADIUS = 48/25.4/2; // in
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

    public static double LATERAL_DISTANCE = 15; // in; distance between the left and right wheels
    public static double LATERAL_X_DISTANCE = 3; // in; distance towards the front of the left and right wheels
    public static double CENTER_Y_OFFSET = 3.25; // in; offset of the lateral wheel from center
    public static double CENTER_X_OFFSET = -6.5; // in; offset

    private DcMotor leftEncoder, rightEncoder, frontEncoder;
    private ExpansionHubEx hub9;
    private ExpansionHubEx hub2;

    public StandardTrackingWheelLocalizer(HardwareMap hardwareMap) {
        super(Arrays.asList(
                new Pose2d(LATERAL_X_DISTANCE, LATERAL_DISTANCE / 2, 0), // left
                new Pose2d(LATERAL_X_DISTANCE, -LATERAL_DISTANCE / 2, 0), // right
                new Pose2d(CENTER_X_OFFSET, CENTER_Y_OFFSET, Math.toRadians(90)) // front
        ));

        hub2 = hardwareMap.get(ExpansionHubEx.class, "hub2");
        hub9 = hardwareMap.get(ExpansionHubEx.class, "hub9");
        frontEncoder = hardwareMap.dcMotor.get("leftLift");
        rightEncoder = hardwareMap.dcMotor.get("ir");
        leftEncoder = hardwareMap.dcMotor.get("il");
    }

    public static double encoderTicksToInches(int ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        RevBulkData bulkData2 = hub2.getBulkInputData();
        RevBulkData bulkData9 = hub9.getBulkInputData();

        if (bulkData2 == null || bulkData9 == null) {
            return Arrays.asList(0.0, 0.0, 0.0);
        }

        List<Double> wheelPositions = new ArrayList<>();
        wheelPositions.add(encoderTicksToInches(bulkData2.getMotorCurrentPosition(frontEncoder)));
        wheelPositions.add(encoderTicksToInches(bulkData2.getMotorCurrentPosition(leftEncoder)));
        wheelPositions.add(encoderTicksToInches(bulkData9.getMotorCurrentPosition(rightEncoder)));
        return wheelPositions;
/*
        return Arrays.asList(
                encoderTicksToInches(leftEncoder.getCurrentPosition()),
                encoderTicksToInches(rightEncoder.getCurrentPosition()),
                encoderTicksToInches(frontEncoder.getCurrentPosition())
        );

 */
    }
}
