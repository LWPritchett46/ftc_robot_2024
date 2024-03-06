package org.firstinspires.ftc.teamcode.resources;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

class ColorDetector extends OpenCvPipeline {

    private int quadrant = 2;
    private Mat workingMatrix = new Mat();
    public double[] colorMiddle = new double[]{0, 0, 0};
    public Point avgPositionOrange;
    public Point avgPositionGreen;
    public Mat maskOrange = new Mat();
    public Mat maskGreen = new Mat();

//    public int position;

    public ColorDetector() {

    }


    public int getQuadrant() {
        return quadrant;
    }

    @Override
    public Mat processFrame(Mat input) {

        Imgproc.cvtColor(input, workingMatrix, Imgproc.COLOR_RGB2HSV_FULL);
        colorMiddle = workingMatrix.get(workingMatrix.rows()/2, workingMatrix.cols()/2);

        //System.out.printf("HSV(%b, %b, %b)%n", colorMiddle[0], colorMiddle[1], colorMiddle[2]);

//        Scalar lowHSVOrange = new Scalar(12.5, 80, 80); // lower bound HSV for orange
//        Scalar highHSVOrange = new Scalar(17.5, 255, 255); // higher bound HSV for orange
        Scalar lowHSVGreen = new Scalar(45, 35, 35); // lower bound HSV for green
        Scalar highHSVGreen = new Scalar(80, 255, 255); // higher bound HSV for green

//        Core.inRange(workingMatrix, lowHSVOrange, highHSVOrange, maskOrange);
        Core.inRange(workingMatrix, lowHSVGreen, highHSVGreen, maskGreen);

        int rows = maskGreen.rows();
        int cols = maskGreen.cols();

//        double totalXOrange = 0;
//        double totalYOrange = 0;
//        double totalPixelsOrange = 0;
        double totalXGreen = 0;
        double totalYGreen = 0;
        double totalPixelsGreen = 0;

        for (int y = 0; y < rows; y++) {
            for (int x = 0; x < cols; x++) {
//                if (maskOrange.get(y, x)[0] > 0) {
//                    totalXOrange += x;
//                    totalYOrange += y;
//                    totalPixelsOrange++;
//                }

                if (maskGreen.get(y, x)[0] > 0) {
                    totalXGreen += x;
                    totalYGreen += y;
                    totalPixelsGreen++;
                }
            }
        }

//        if (totalPixelsOrange > 0) {
//            double averageXOrange = totalXOrange / totalPixelsOrange;
//            double averageYOrange = totalYOrange / totalPixelsOrange;
//            avgPositionOrange = new Point(averageXOrange, averageYOrange);
//        }

        if (totalPixelsGreen > 0) {
            double averageXGreen = totalXGreen / totalPixelsGreen;
            double averageYGreen = totalYGreen / totalPixelsGreen;
            avgPositionGreen = new Point(averageXGreen, averageYGreen);
        }

//        if (avgPositionOrange != null) {
//            Imgproc.rectangle(workingMatrix, new Point(avgPositionOrange.x - 20, avgPositionOrange.y - 20), new Point(avgPositionOrange.x + 20, avgPositionOrange.y + 20), new Scalar(0, 0, 255), 5);
//        }

        if (avgPositionGreen != null) {
            Imgproc.rectangle(workingMatrix, new Point(avgPositionGreen.x - 20, avgPositionGreen.y - 20), new Point(avgPositionGreen.x + 20, avgPositionGreen.y + 20), new Scalar(255, 0, 0), 5);
            if (avgPositionGreen.x > (2.0/3.0) * maskGreen.cols()) {
                quadrant = 3;
            } else if (avgPositionGreen.x < maskGreen.cols()/3.0) {
                quadrant = 1;
            }
        }



        return maskGreen;

    }


}
