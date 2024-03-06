package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.*;

import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
import java.util.ArrayList;
import java.util.List;
import org.opencv.imgproc.Moments;

class Detector extends OpenCvPipeline {

    private int quadrant = 2;
    double cX = 0;
    double cY = 0;

    public boolean isRed;

    public int numOfContours;

//    public int position;

    public Detector(boolean isRed) {
        this.isRed = isRed;
    }


    public int getQuadrant() {
        return quadrant;
    }

    @Override
    public Mat processFrame(Mat input) {
        // Preprocess the frame to detect colored regions
        Mat colorMask = preprocessFrame(input);
        // Find contours of the detected colored regions
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(colorMask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        numOfContours = contours.size();

        // Find the largest contour (blob)
        MatOfPoint largestContour = findLargestContour(contours);

        if (largestContour != null) {

            // Draw a green outline around the largest detected object
            Imgproc.drawContours(input, contours, contours.indexOf(largestContour), new Scalar(0, 255, 0), 2);
            // Calculate the centroid of the largest contour
            Moments moments = Imgproc.moments(largestContour);
            cX = moments.get_m10() / moments.get_m00();
            cY = moments.get_m01() / moments.get_m00();

            // Determine the correct quadrant
            if (cX <= input.cols()*0.33){
                quadrant = 1;
            } else if (cX >= input.cols()*0.66){
                quadrant = 3;
            } else {
                quadrant = 2;
            }

        }

        return colorMask;
    }

    public Mat preprocessFrame(Mat frame){
        Mat hsvFrame = new Mat();
        Imgproc.cvtColor(frame, hsvFrame, Imgproc.COLOR_RGB2HSV);

        Scalar lowerHSV;
        Scalar upperHSV;

        if (isRed){
            lowerHSV = new Scalar(155, 80, 220);
            upperHSV = new Scalar(180,255,255);
        } else {
            lowerHSV = new Scalar(100, 80, 220);
            upperHSV = new Scalar(140, 255, 255);
        }



        Mat colorMask = new Mat();
        Core.inRange(hsvFrame, lowerHSV, upperHSV, colorMask);

        Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));
        Imgproc.morphologyEx(colorMask, colorMask, Imgproc.MORPH_OPEN, kernel);
        Imgproc.morphologyEx(colorMask, colorMask, Imgproc.MORPH_CLOSE, kernel);

        return colorMask;
    }

    private MatOfPoint findLargestContour(List<MatOfPoint> contours) {
        double maxArea = 0;
        MatOfPoint largestContour = null;

        for (MatOfPoint contour : contours) {
            double area = Imgproc.contourArea(contour);
            if (area > maxArea) {
                maxArea = area;
                largestContour = contour;
            }
        }
        return largestContour;
    }
}



