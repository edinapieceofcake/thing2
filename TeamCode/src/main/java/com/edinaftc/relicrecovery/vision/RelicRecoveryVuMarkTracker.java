package com.edinaftc.relicrecovery.vision;

import com.edinaftc.library.vision.Overlay;
import com.edinaftc.library.vision.Tracker;
import com.edinaftc.library.vision.VisionCamera;
import com.edinaftc.library.vision.VuforiaCamera;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.opencv.core.Mat;

public class RelicRecoveryVuMarkTracker extends Tracker {
    private boolean initialized = false;
    private VuforiaTrackables relicTrackables;
    private VuforiaTrackable relicVuMark;

    @Override
    public void enable() {
        super.enable();

        if (initialized) {
            relicTrackables.activate();
        }
    }

    @Override
    public void disable() {
        super.disable();

        if (initialized) {
            relicTrackables.activate();
        }
    }

    @Override
    public void init(VisionCamera camera) {
        VuforiaLocalizer vuforia = ((VuforiaCamera) camera).getVuforia();
        relicTrackables = vuforia.loadTrackablesFromAsset("RelicVuMark");
        relicVuMark = relicTrackables.get(0);
        relicVuMark.setName("relicVuMark");

        initialized = true;

        enable();
    }

    @Override
    public void processFrame(Mat frame, double timestamp) {
    }

    @Override
    public void drawOverlay(Overlay overlay, int imageWidth, int imageHeight, boolean debug) {

    }

    public RelicRecoveryVuMark getVuMark() {
        return RelicRecoveryVuMark.from(relicVuMark);
    }
}
