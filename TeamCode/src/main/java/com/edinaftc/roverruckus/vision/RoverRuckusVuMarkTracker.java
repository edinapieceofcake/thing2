package com.edinaftc.roverruckus.vision;

import com.edinaftc.library.vision.Overlay;
import com.edinaftc.library.vision.Tracker;
import com.edinaftc.library.vision.VisionCamera;
import com.edinaftc.library.vision.VuforiaCamera;
import com.edinaftc.library.vision.VuforiaCamera2;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.opencv.core.Mat;

import java.util.ArrayList;
import java.util.List;

public class RoverRuckusVuMarkTracker extends Tracker {
    private boolean initialized = false;
    private VuforiaTrackables targetsRoverRuckus;
    private List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
    @Override
    public void enable() {
        super.enable();

        if (initialized) {
            targetsRoverRuckus.activate();
        }
    }

    @Override
    public void disable() {
        super.disable();

        if (initialized) {
            targetsRoverRuckus.activate();
        }
    }

    @Override
    public void init(VisionCamera camera) {
        VuforiaLocalizer vuforia = ((VuforiaCamera) camera).getVuforia();
        targetsRoverRuckus = vuforia.loadTrackablesFromAsset("RoverRuckus");
        VuforiaTrackable blueRover = targetsRoverRuckus.get(0);
        blueRover.setName("Blue-Rover");
        VuforiaTrackable redFootprint = targetsRoverRuckus.get(1);
        redFootprint.setName("Red-Footprint");
        VuforiaTrackable frontCraters = targetsRoverRuckus.get(2);
        frontCraters.setName("Front-Craters");
        VuforiaTrackable backSpace = targetsRoverRuckus.get(3);
        backSpace.setName("Back-Space");
        allTrackables.addAll(targetsRoverRuckus);

        initialized = true;

        enable();
    }

    @Override
    public void processFrame(Mat frame, double timestamp) {
    }

    @Override
    public void drawOverlay(Overlay overlay, int imageWidth, int imageHeight, boolean debug) {

    }

    public String GetRoverRuckusMark() {
        for (VuforiaTrackable trackable : allTrackables) {
            if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                return trackable.getName();
            }
        }

        return "Nothing";
    }
}
