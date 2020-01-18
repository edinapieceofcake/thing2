package com.edinaftc.skystone.vision;

import com.edinaftc.library.vision.Overlay;
import com.edinaftc.library.vision.Tracker;
import com.edinaftc.library.vision.VisionCamera;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

public class SkyStoneDetector extends Tracker {
    private Mat mat0;
    private Mat mat1;
    private Mat mat2;

    private Mat mask0;
    private Mat mask1;
    private Mat mask2;

    private boolean madeMats = false;

    private Scalar BLACK = new Scalar(0,0,0);
    private Scalar WHITE = new Scalar(255,255,255);
    private Scalar RED = new Scalar(0, 0, 255);

    public double cx0 = 400;
    public double cy0 = 400;
    public double cx1 = 600;
    public double cy1 = 400;
    public double cx2 = 900;
    public double cy2 = 400;

    public double lineX = 110;

    private int r = 10;
    private int strokeWidth = 3;

    private SkystoneLocation location = SkystoneLocation.right;

    @Override
    public void init(VisionCamera camera) {

    }

    @Override
    public synchronized void processFrame(Mat frame, double timestamp) {
        int h = frame.height();
        int w = frame.width();

        int type = frame.type();
        if (!madeMats) {
            mask0 = new Mat(h, w, type);
            mask1 = new Mat(h, w, type);
            mask2 = new Mat(h, w, type);
            mat0 = new Mat();
            mat1 = new Mat();
            mat2 = new Mat();
            madeMats = true;
        }

        mask0.setTo(BLACK);
        mask1.setTo(BLACK);
        mask2.setTo(BLACK);

        Imgproc.circle(mask0, new Point(cx0, cy0), r, WHITE, Core.FILLED);
        Imgproc.circle(mask1, new Point(cx1, cy1), r, WHITE, Core.FILLED);
        Imgproc.circle(mask2, new Point(cx2, cy2), r, WHITE, Core.FILLED);

        Core.bitwise_and(mask0, frame, mat0);
        Core.bitwise_and(mask1, frame, mat1);
        Core.bitwise_and(mask2, frame, mat2);

        double val0 = Core.sumElems(mat0).val[0] + Core.sumElems(mat0).val[1] +
                Core.sumElems(mat0).val[2];
        double val1 = Core.sumElems(mat1).val[0] + Core.sumElems(mat1).val[1] +
                Core.sumElems(mat1).val[2];
        double val2 = Core.sumElems(mat2).val[0] + Core.sumElems(mat2).val[1] +
                Core.sumElems(mat2).val[2];

        if (val0 < val1 && val0 < val2) {
            location = SkystoneLocation.left;
        } else if (val1 < val0 && val1 < val2) {
            location = SkystoneLocation.middle;
        } else {
            location = SkystoneLocation.right;
        }


    }

    @Override
    public synchronized void drawOverlay(Overlay overlay, int imageWidth, int imageHeight, boolean debug) {
        Scalar s0 = WHITE;
        Scalar s1 = WHITE;
        Scalar s2 = WHITE;
        double r0 = 10;
        double r1 = 10;
        double r2 = 10;

        if (location == SkystoneLocation.right) {
            s2 = RED;
            r2= 20;
        } else if (location == SkystoneLocation.left) {
            s0 = RED;
            r0 = 20;
        } else {
            s1 = RED;
            r1 = 20;
        }

        overlay.fillCircle(new Point(cx0, cy0), r0, s0);
        overlay.fillCircle(new Point(cx1, cy1), r1, s1);
        overlay.fillCircle(new Point(cx2, cy2), r2, s2);

        overlay.strokeLine(new Point(lineX,0), new Point(lineX,725),  new Scalar(51, 255, 51), 25);
    }

    public SkystoneLocation getLocation() {
        return location;
    }
}
