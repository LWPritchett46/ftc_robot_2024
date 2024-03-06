package org.firstinspires.ftc.teamcode.resources;


import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.resources.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.HardwarePushbot;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous (name="Testing April Tags", group="Autonomous")
@Disabled

public class AutonomousLeft extends OpMode {

    HardwarePushbot robot = new HardwarePushbot();

    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;
    static final int FORWARD = 52;
    static final int SIDE = 55;
    static final int ROTATE = 53;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    private final int[] wanted_ids = new int[]{1, 2, 3};
    AprilTagDetection tagFound = null;
    //private AprilTagDetection[] detected_tags = new AprilTagDetection[3];

    @Override
    public void init() {
        robot.init(hardwareMap);
        telemetry.addData("Started", "FALSE");
        telemetry.update();


        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        FtcDashboard.getInstance().startCameraStream(camera, 0);
        telemetry.setMsTransmissionInterval(50);
    }

    @Override
    public void init_loop(){


    }


    public void start(){

        telemetry.addData("Started", "TRUE");
        if (tagFound != null) {
            telemetry.addData("Tag ID", tagFound.id);
        }
        else{
            telemetry.addData("Tag ID", "None");
        }

        robot.left_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.right_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.left_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.right_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    @Override
    public void loop() {

        ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

        AprilTagDetection[] detected_tags = new AprilTagDetection[3];

        if(currentDetections.size() != 0) {

            for (AprilTagDetection tag : currentDetections) {
                for (int i = 0; i < wanted_ids.length; i ++) {

                    if (wanted_ids[i] == tag.id) {
                        detected_tags[i] = tag;
                    }
                    else{
                        detected_tags[i] = null;
                    }
                }
            }
        }
        int driveSpeed = 800;

        if (detected_tags[1] != null) {
            if (detected_tags[1].center.x > 450){
                robot.move(0, driveSpeed, 0, 1);
            }
            else if (detected_tags[1].center.x < 350){
                robot.move(Math.PI, driveSpeed, 0, 1);
            }
            else{
                robot.move(Math.PI/2, driveSpeed, 0, 1);
            }
        }
        else {
            robot.brake();
        }
        telemetry.update();

    }
}
