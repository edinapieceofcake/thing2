package com.edinaftc.skystone;

import com.edinaftc.library.motion.TelemetryMounts;
import com.edinaftc.library.subsystems.ArmAndFlapper;
import com.edinaftc.library.subsystems.Grabber;
import com.edinaftc.library.subsystems.Hook;
import com.edinaftc.library.subsystems.Intake;
import com.edinaftc.library.subsystems.LiftandArm;
import com.edinaftc.library.subsystems.MecanumDrive;
import com.edinaftc.library.subsystems.MecanumDrive2;
import com.edinaftc.library.subsystems.Subsystem;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ThreadPool;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.ExecutorService;

public class Robot {
    private ExecutorService subsystemUpdateExecutor;
    private boolean started;

    public MecanumDrive2 drive;

    public Intake intake;

    public LiftandArm liftandarm;

    public Hook hook;

    public Grabber grabber;

    public ArmAndFlapper arm;

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

    public Robot(OpMode opMode, Telemetry telemetry) {
        this.telemetry = telemetry;

        subsystems = new ArrayList<>();
/*
        try {
            drive = new MecanumDrive2(opMode.hardwareMap);
            subsystems.add(drive);
        } catch (IllegalArgumentException e) {

        }
*/
        try {
            intake = new Intake(opMode.hardwareMap);
            subsystems.add(intake);
        } catch (IllegalArgumentException e) {

        }

        try {
            liftandarm = new LiftandArm(opMode.hardwareMap);
            subsystems.add(liftandarm);
        } catch (IllegalArgumentException e) {

        }
/*
        try {
            hook = new Hook(opMode.hardwareMap);
            subsystems.add(hook);
        } catch (IllegalArgumentException e) {

        }

        try {
            grabber = new Grabber(opMode.hardwareMap);
            subsystems.add(grabber);
        } catch (IllegalArgumentException e) {

        }

        try {
            arm = new ArmAndFlapper(opMode.hardwareMap);
            subsystems.add(arm);
        } catch (IllegalArgumentException e) {

        }
*/
        subsystemUpdateExecutor = ThreadPool.newSingleThreadExecutor("subsystem update");
    }

    public void start() {
        if (!started) {
            subsystemUpdateExecutor.submit(subsystemUpdateRunnable);
            started = true;
        }
    }

    public void stop() {
        if (subsystemUpdateExecutor != null) {
            subsystemUpdateExecutor.shutdownNow();
            subsystemUpdateExecutor = null;
            started = false;
        }
    }
}
