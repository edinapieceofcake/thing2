package com.edinaftc.roverruckus.vision.filter;


import org.opencv.core.Mat;


public abstract class ColorFilter {
    public abstract void process(Mat input, Mat mask);

}
