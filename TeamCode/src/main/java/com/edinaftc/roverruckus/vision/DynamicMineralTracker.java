package com.edinaftc.roverruckus.vision;

import com.edinaftc.library.vision.Overlay;
import com.edinaftc.library.vision.Tracker;
import com.edinaftc.library.vision.VisionCamera;
import com.edinaftc.roverruckus.utils.Enums;
import com.edinaftc.roverruckus.vision.filter.ColorFilter;
import com.edinaftc.roverruckus.vision.filter.HSVColorFilter;
import com.edinaftc.roverruckus.vision.filter.LeviColorFilter;
import com.edinaftc.roverruckus.vision.scorer.IScorer;
import com.edinaftc.roverruckus.vision.scorer.MaxAreaScorer;
import com.edinaftc.roverruckus.vision.scorer.PerfectAreaScorer;
import com.edinaftc.roverruckus.vision.scorer.RatioScorer;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

public class DynamicMineralTracker extends Tracker {
    // Defining Mats to be used.
    private Mat workingMat = null; // Used for preprocessing and working with (blurring as an example)
    private Mat maskYellow = null; // Yellow Mask returned by color filter
    private Mat hierarchy  = null; // hierarchy used by coutnours

    // Results of the detector
    private boolean found    = false; // Is the gold mineral found
    private boolean aligned  = false; // Is the gold mineral aligned
    private double  goldXPos = 0;     // X Position (in pixels) of the gold element
    private double  goldYPos = 0;

    // Detector settings
    public double alignPosOffset  = 0;    // How far from center frame is aligned
    public double alignSize       = 100;  // How wide is the margin of error for alignment

    public Enums.AreaScoringMethod areaScoringMethod = Enums.AreaScoringMethod.MAX_AREA; // Setting to decide to use MaxAreaScorer or PerfectAreaScorer

    private List<IScorer> scorers = new ArrayList<>();

    public RatioScorer ratioScorer       = new RatioScorer(1.0, 3);          // Used to find perfect squares
    public MaxAreaScorer maxAreaScorer     = new MaxAreaScorer( 0.01);                    // Used to find largest objects
    public PerfectAreaScorer perfectAreaScorer = new PerfectAreaScorer(5000,0.05); // Used to find objects near a tuned area value

    public double downscale = 0.5;
    public Size   downscaleResolution = new Size(640, 480);
    public boolean useFixedDownscale = true;
    private Size initSize;

    //Create the default filters and scorers
    public ColorFilter yellowFilter = new LeviColorFilter(LeviColorFilter.ColorPreset.YELLOW); //Default Yellow filter

    Rect resultRect = null;

    @Override
    public void init(VisionCamera camera) {
        yellowFilter = new LeviColorFilter(LeviColorFilter.ColorPreset.YELLOW, 100);
        // Add diffrent scoreres depending on the selected mode
        if(areaScoringMethod == Enums.AreaScoringMethod.MAX_AREA){
            scorers.add(maxAreaScorer);
        }

        if (areaScoringMethod == Enums.AreaScoringMethod.PERFECT_AREA){
            scorers.add(perfectAreaScorer);
        }
    }

    @Override
    public void processFrame(Mat frame, double timestamp) {
        Rect bestRect = null;
        Size adjustedSize;
        double bestDiffrence = Double.MAX_VALUE; // MAX_VALUE since less diffrence = better

        initSize = frame.size();

        if(useFixedDownscale){
            adjustedSize = downscaleResolution;
        }else{
            adjustedSize = new Size(initSize.width * downscale, initSize.height * downscale);
        }


        if (workingMat == null) {
            workingMat = new Mat(); // Used for preprocessing and working with (blurring as an example)
            maskYellow = new Mat(); // Yellow Mask returned by color filter
            hierarchy  = new Mat(); // hierarchy used by coutnours
        }

        frame.copyTo(workingMat);

        Imgproc.resize(workingMat, workingMat,adjustedSize); // Downscale

        //Preprocess the working Mat (blur it then apply a yellow filter)
        Imgproc.GaussianBlur(workingMat,workingMat,new Size(5,5),0);
        yellowFilter.process(workingMat.clone(),maskYellow);

        //Find contours of the yellow mask and draw them to the display mat for viewing
        // Current result
        List<MatOfPoint> contoursYellow = new ArrayList<>();
        Imgproc.findContours(maskYellow, contoursYellow, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        // Loop through the contours and score them, searching for the best result
        for (MatOfPoint cont : contoursYellow) {
            double score = calculateScore(cont); // Get the diffrence score using the scoring API

            // Get bounding rect of contour
            Rect rect = Imgproc.boundingRect(cont);

            // If the result is better then the previously tracked one, set this rect as the new best
            if (score < bestDiffrence) {
                bestDiffrence = score;
                bestRect = rect;
            }
        }

        // Vars to calculate the alignment logic.
        double alignX    = (adjustedSize.width / 2) + alignPosOffset; // Center point in X Pixels
        double alignXMin = alignX - (alignSize / 2); // Min X Pos in pixels
        double alignXMax = alignX +(alignSize / 2); // Max X pos in pixels
        double xPos; // Current Gold X Pos

        if(bestRect != null){
            resultRect = bestRect;

            // Set align X pos
            xPos = bestRect.x + (bestRect.width / 2);
            goldXPos = xPos;
            goldYPos = bestRect.y + (bestRect.width /2 );

            // Check if the mineral is aligned
            if(xPos < alignXMax && xPos > alignXMin){
                aligned = true;
            }else{
                aligned = false;
            }
            found = true;
        }else{
            found = false;
            aligned = false;
            resultRect = null;
        }
    }

    @Override
    public void drawOverlay(Overlay overlay, int imageWidth, int imageHeight, boolean debug) {
        overlay.putText("Found: " + found, Overlay.TextAlign.LEFT, new Point(20, 700),
                new Scalar(255, 255, 0), 30);
        overlay.putText("Aligned: " + aligned, Overlay.TextAlign.LEFT, new Point(20, 675),
                new Scalar(255, 255, 0), 30);
        if (resultRect != null) {
            overlay.putText("Rect: " + resultRect, Overlay.TextAlign.LEFT, new Point(20, 650),
                    new Scalar(255, 255, 0), 30);
        } else {
            overlay.putText("Rect: not found", Overlay.TextAlign.LEFT, new Point(20, 650),
                    new Scalar(255, 255, 0), 30);
        }
    }

    private double calculateScore(Mat input){
        double totalScore = 0;

        for(IScorer scorer : scorers){
            totalScore += scorer.calculateScore(input);
        }

        return totalScore;
    }

    public boolean isFound()
    {
        return found;
    }

    public Rect getRect()
    {
        return resultRect;
    }

    public boolean isAligned()
    {
        return aligned;
    }
}
