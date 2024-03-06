package org.firstinspires.ftc.teamcode.resources;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;



@Autonomous(name = "openCvDuck")
@Disabled
public class markerReader extends LinearOpMode {

   OpenCvWebcam webcam;
   private detector detect;
   private String position;

   @Override
   public void runOpMode(){


      int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
      webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
      webcam.setPipeline(detect);
      webcam.startStreaming(320,240, OpenCvCameraRotation.UPRIGHT);
      position = detect.position;

      while(!isStarted()){
         telemetry.addData("Level: ", position);
      }


   }

}
