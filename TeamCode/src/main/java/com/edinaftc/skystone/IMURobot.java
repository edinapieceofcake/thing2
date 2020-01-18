package com.edinaftc.skystone;

import com.edinaftc.library.subsystems.IMU;
import com.edinaftc.library.subsystems.Intake;
import com.edinaftc.library.subsystems.LiftandArm;
import com.edinaftc.library.subsystems.MecanumDrive2;
import com.edinaftc.library.subsystems.Subsystem;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ThreadPool;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.ExecutorService;

public class IMURobot {
    private ExecutorService subsystemUpdateExecutor;
    private boolean started;

    public IMU imu;
    public MecanumDrive2 drive;

    private List<Subsystem> subsystems;

    private Telemetry telemetry;

    private Runnable subsystemUpdateRunnable = () -> {
        while (!Thread.currentThread().isInterrupted()) {
            try {
                for (Subsystem subsystem : subsystems) {
                    if (subsystem == null) continue;
                    try {
                        subsystem.update();
                    } catch (Throwable t) {
                        this.telemetry.addData("Exception running thread 1", "");
                        this.telemetry.update();
                    }
                }
            } catch (Throwable t) {
                this.telemetry.addData("Exception running thread 2", "");
                this.telemetry.update();
            }
        }
    };

    public IMURobot(OpMode opMode, Telemetry telemetry) {
        this.telemetry = telemetry;

        subsystems = new ArrayList<>();

        try {
            imu = new IMU(opMode.hardwareMap, telemetry);
            subsystems.add(imu);
        } catch (IllegalArgumentException e) {

        }

        try {
            drive = new MecanumDrive2(opMode.hardwareMap);
            subsystems.add(drive);
        } catch (IllegalArgumentException e) {

        }

        subsystemUpdateExecutor = ThreadPool.newSingleThreadExecutor("subsystem update");
    }

    public void start() {
        if (!started) {
            subsystemUpdateExecutor.submit(subsystemUpdateRunnable);
            started = true;
        }
    }

    private void stop() {
        if (subsystemUpdateExecutor != null) {
            subsystemUpdateExecutor.shutdownNow();
            subsystemUpdateExecutor = null;
        }

    }
}
