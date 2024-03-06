package org.firstinspires.ftc.teamcode.resources;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

class detector extends OpenCvPipeline {

    private Mat workingMatrix = new Mat();
    public  String position;

    public detector(){

    }

    @Override
    public Mat processFrame(Mat input) {
       input.copyTo(workingMatrix);

       if(workingMatrix.empty()){
           return workingMatrix;
       }

       Imgproc.cvtColor(workingMatrix, workingMatrix, Imgproc.COLOR_RGB2YCrCb);

       Mat matBot = workingMatrix.submat(120, 150, 10, 50);
       Mat matMid = workingMatrix.submat(120, 150, 80, 120);
       Mat matTop = workingMatrix.submat(120, 150, 150, 190);

       //ADD TO THE STREAM

        Imgproc.rectangle(workingMatrix, new Rect(10, 120, 40, 30),new Scalar(255,0,0));
        Imgproc.rectangle(workingMatrix, new Rect(80, 120, 40, 30),new Scalar(255,0,0));
        Imgproc.rectangle(workingMatrix, new Rect(150, 120, 40, 30),new Scalar(255,0,0));

       double botTotal = Core.sumElems(matBot).val[2];
       double midTotal = Core.sumElems(matMid).val[2];
       double topTotal = Core.sumElems(matTop).val[2];

       if(botTotal > midTotal){
           if(botTotal > topTotal){
               //Duck is in position one
               position = "bottom";
           } else {
               //Duck is in position three
               position = "top";
           }
       } else {
           if(midTotal > topTotal) {
               //Duck is in position two
               position = "middle";
           } else {
               //Duck is in position three
               position = "top";
           }
       }

       return workingMatrix;

    }


}
