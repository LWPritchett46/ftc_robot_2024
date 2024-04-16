package org.firstinspires.ftc.teamcode;

import static android.os.SystemClock.sleep;

import static org.firstinspires.ftc.teamcode.TeleOp360Mov.armLow;
import static org.firstinspires.ftc.teamcode.TeleOp360Mov.closeLeft;
import static org.firstinspires.ftc.teamcode.TeleOp360Mov.closeRight;
import static org.firstinspires.ftc.teamcode.TeleOp360Mov.openLeft;
import static org.firstinspires.ftc.teamcode.TeleOp360Mov.openRight;
import static org.firstinspires.ftc.teamcode.TeleOp360Mov.rWiperGrab;
import static org.firstinspires.ftc.teamcode.TeleOp360Mov.rWiperOpen;
import static org.firstinspires.ftc.teamcode.TeleOp360Mov.rWiperStow;
import static org.firstinspires.ftc.teamcode.TeleOp360Mov.rotGrab;
import static org.firstinspires.ftc.teamcode.TeleOp360Mov.rotHover;
import static org.firstinspires.ftc.teamcode.TeleOp360Mov.rotLow;
import static org.firstinspires.ftc.teamcode.util.Utility.*;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import org.openftc.apriltag.AprilTagDetection;

import java.util.List;

/**
 * To do:
 *      (X) Fix rotation one the first movement of all routines (most up to date is routineThree) X
 *      (X) Possibly code the logic discussed with Dr. Eva X
 *      ( ) Code the Right routines
 *      ( ) Code the Left routines
 *      ( ) Code a routine for just parking
 *      (X) Figure out a way to get detection to not be as sensitive to light (watch how other teams are doing it)
 *      ( ) Optimize routines to fit within 30s:
 *          ( ) Don't give a fuck about AprilTags
 *          ( ) Fix intake_rot Servo
 *          ( ) Can we start with claw grabbing pixels
 *          ( ) If so, can we unfold as we move
 *          ( ) Shave off unnecessary pauses
 *          ( ) What's the greatest speed we can do without sacrificing accuracy?
 *
 */

@Autonomous(name="AutoBR", group="Autonomous")
public class AutoBR extends OpMode {
    HardwarePushbot robot = new HardwarePushbot();

    double residualPower = 0;

    // A greater offset results in a shift to the left
    int offset = 100;
    boolean centeredTag = false;


    int centerX = 320;
    int quadrant = 3;

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    // TFOD_MODEL_ASSET points to a model file stored in the project Asset location,
    // this is only used for Android Studio when using models in Assets.
    private static final String TFOD_MODEL_ASSET = "model.tflite";
    private static final String[] LABELS = {"blue", "red"};


    private TfodProcessor tfod;
    private VisionPortal visionPortal;

    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.05;

    private final int[] wanted_ids = new int[]{1, 2, 3};
    AprilTagDetection tagFound = null;

    @Override
    public void init() {
        robot.init(hardwareMap);
        initTfod();
    }

    @Override
    public void init_loop() {

        telemetryTfod();

        // Push telemetry to the Driver Station.


        // Save CPU resources; can resume streaming when needed.
        if (gamepad1.dpad_down) {
            visionPortal.stopStreaming();
        } else if (gamepad1.dpad_up) {
            visionPortal.resumeStreaming();
        }

        //FtcDashboard.getInstance().startCameraStream((CameraStreamSource) visionPortal, 0);
        telemetry.setMsTransmissionInterval(50);

        List<Recognition> currentRecognitions = tfod.getRecognitions();
        float highestConfidence = 0;

        if (!currentRecognitions.isEmpty()) {
            for (Recognition recognition : currentRecognitions) {
                if (recognition.getConfidence() > highestConfidence) {
                    highestConfidence = recognition.getConfidence();
                    centerX = Math.round((recognition.getLeft() + recognition.getRight()) / 2);
                    quadrant = 1 + Math.floorDiv(centerX, (640 / 2));
                }
            }
        } else {
            quadrant = 3;
        }

        telemetry.addData("Quadrant", quadrant);
        telemetry.update();
    }

    @Override
    public void start() {
        visionPortal.stopStreaming();
        visionPortal.setProcessorEnabled(tfod, false);
        robot.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.imu.resetYaw();

        robot.imu.resetYaw();

        robot.claw_rot.setPosition(rotGrab);
        robot.claw_left.setPosition(closeLeft);
        robot.claw_right.setPosition(openRight);
        sleep(150);
        robot.right_wiper.setPosition(rWiperOpen);
//        robot.claw_rot.setPosition(rotHover);
//        robot.arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        robot.armUp(robot.armHover);
//        residualPower = 0.08;




        // Yo I fixed it
        switch (quadrant) {
            case 1:
                routineOne();
                break;
            case 2:
                routineTwo();
                break;
            case 3:
                routineThree();
                break;
        }
    }

    @Override
    public void loop() {

        telemetry.addLine("In Loop");
        telemetry.addData("wrapIMU", wrapIMU(robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)));
        telemetry.update();
    }

//    @SuppressLint("DefaultLocale")
//    public void centerWantedTag() {
//
//        robot.setModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();
//        AprilTagDetection[] detected_tags = new AprilTagDetection[3];
//
//
//        //
//        if (currentDetections.size() != 0) {
//            // The code works per iteration but after the iteration of the wanted tag
//            // the following iteration just erases the tag
//
//            detected_tags = new AprilTagDetection[3];
//
//            for (AprilTagDetection tag : currentDetections) {
//                telemetry.addData("Found Tag ID", tag.id);
//                for (int i = 0; i < wanted_ids.length; i++) {
//                    if (wanted_ids[i] == tag.id) {
//                        detected_tags[i] = tag;
//                    }
//                }
//            }
//        }
//
//
//        int wantedTag = quadrant - 1;
//        double driveSpeed = 0.04;
//        int unwantedDetected = -1;
//
//
//        telemetry.addData("Wanted Tag", quadrant);
//
//        if (detected_tags[wantedTag] != null) {
//            telemetry.addData("Wanted Tag X", detected_tags[wantedTag].center.x);
//        }
//
//        if (detected_tags[wantedTag] != null) {
//            if (detected_tags[wantedTag].center.x > (408 + offset)) {
//                robot.setVelocities(robot.move(0, driveSpeed, 0, 1));
//            } else if (detected_tags[wantedTag].center.x < (392 + offset)) {
//                robot.setVelocities(robot.move(Math.PI, driveSpeed, 0, 1));
//            } else {
//
//                double[] positions = new double[]{
//                        robot.left_front.getCurrentPosition(),
//                        -robot.right_front.getCurrentPosition(),
//                        -robot.left_back.getCurrentPosition(),
//                        robot.right_back.getCurrentPosition()
//                };
//
//                robot.brake();
//                sleep(100);
//                robot.rotateTo(180, 700, 0.15, 1.3);
//                residualPower = -0.1;
//                robot.armUp(armLow);
//                robot.arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//                robot.claw_rot.setPosition(rotLow);
//
//                sleep(500);
//
//                robot.moveTo(Math.PI, 450, 800, 1.25, telemetry);
//                robot.claw_right.setPosition(openRight);
//                robot.claw_right.setPosition(openRight);
//                sleep(200);
//
//                // Move Back
//                robot.moveTo(0, 450, 800, 1.25, telemetry);
//                sleep(200);
//
//                if (average(positions) < 0){
//                    robot.moveTo(Math.PI/2, (int)(Math.abs(average(positions))), 800, 1.3, telemetry);
//                }
//                else{
//                    robot.moveTo(-Math.PI/2, (int)(Math.abs(average(positions))), 800, 1.3, telemetry);
//                }
//
//                robot.claw_rot.setPosition(rotHover);
//                robot.arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//                robot.armUp(robot.armHover);
//                sleep(500);
//
//                // Move to parking location
//                robot.moveTo(215 * (Math.PI / 180), 1000, 1000, 1.3, telemetry);
//
//                telemetry.addLine("Finished MoveTo");
//                sleep(500);
//
//                centeredTag = true;
//                return;
//            }
//        } else {
//            for (int i = 0; i < detected_tags.length; i++) {
//                if (detected_tags[i] != null) {
//                    unwantedDetected = i;
//                    break;
//                }
//            }
//
//            if (unwantedDetected > wantedTag) {
//                if (detected_tags[unwantedDetected].center.x < 450) {
//                    robot.setVelocities(robot.move(Math.PI, driveSpeed, 0, 1));
//                } else if (detected_tags[unwantedDetected].center.x > 550) {
//                    robot.setVelocities(robot.move(0, driveSpeed, 0, 1));
//                }
//
//            } else if (unwantedDetected < wantedTag && unwantedDetected != -1) {
//                if (detected_tags[unwantedDetected].center.x > 350) {
//                    robot.setVelocities(robot.move(0, driveSpeed, 0, 1));
//                } else if (detected_tags[unwantedDetected].center.x < 250) {
//                    robot.setVelocities(robot.move(Math.PI, driveSpeed, 0, 1));
//                }
//            } else {
//                robot.brake();
//            }
//        }
//
//        telemetry.update();
//    }

    @Override
    public void stop() {
    }


    private void routineOne() {

        robot.moveRotateTo(270, 0 * (Math.PI/180), 0.15, true, 1.0, 0.52, telemetry);
        robot.moveTo(180 * (Math.PI/180), 400, 500, 0, telemetry);

        robot.fling.setPosition(0.1);
        sleep(50);

        robot.moveTo(5 * (Math.PI/180), 1150, 600, 0, telemetry);

        robot.right_wiper.setPosition(rWiperGrab);
        sleep(600);
        robot.claw_right.setPosition(closeRight);
        sleep(70);

        robot.moveTo(180 * (Math.PI/180), 100, 500, 0, telemetry);
        robot.brake();
        robot.moveTo(100 * (Math.PI/180), 1000, 500, 0, telemetry);
        robot.brake();


        //Common Part
        robot.claw_rot.setPosition(rotHover);
        robot.arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.armUp(robot.armHover);
        robot.setVelocities(new double[]{-400, 400, 400, 400});
        sleep(100);
        robot.claw_rot.setPosition(rotLow + 0.12);

        robot.moveTo(180 * (Math.PI/180), 3800, 3000, 0.8, telemetry);
        robot.brake();
        robot.rotateTo(92, 1500, 0.2, true, 1, telemetry);
        robot.brake();

        robot.moveTo(90 * (Math.PI/180), 1080, 1500, 0.8, telemetry);
        robot.brake();

        robot.arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.armUp(robot.armHover + 100);
        robot.arm.setPower(0.04);

        robot.moveTo(0 * (Math.PI/180), 750, 1000, 0.8, telemetry);
        robot.arm.setPower(0);
        robot.brake();

        robot.claw_right.setPosition(openRight);
        robot.claw_left.setPosition(openLeft);
        robot.right_wiper.setPosition(rWiperStow);
        sleep(70);

        robot.moveTo(245 * (Math.PI/180), 600, 1000, 0.8, telemetry);
        robot.claw_rot.setPosition(rotGrab);
        robot.moveTo(340 * (Math.PI/180), 750, 1000, 0.8, telemetry);
    }

    private void routineTwo() {

        robot.moveRotateTo(270, 350 * (Math.PI/180), 0.15, true, 1.0, 0.38, telemetry);

        robot.fling.setPosition(0.1);
        sleep(50);

        robot.moveTo(300 * (Math.PI/180), 500, 500, 0, telemetry);
        robot.moveTo(0 * (Math.PI/180), 200, 500, 0, telemetry);

        robot.right_wiper.setPosition(rWiperGrab);
        sleep(600);
        robot.claw_right.setPosition(closeRight);
        sleep(70);

        robot.moveTo(180 * (Math.PI/180), 100, 500, 0, telemetry);
        robot.brake();
        robot.moveTo(100 * (Math.PI/180), 1000, 500, 0, telemetry);
        robot.brake();


        // Common Part
        robot.claw_rot.setPosition(rotHover);
        robot.arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.armUp(robot.armHover);
        robot.setVelocities(new double[]{-400, 400, 400, 400});
        sleep(100);
        robot.claw_rot.setPosition(rotLow + 0.12);

        robot.moveTo(180 * (Math.PI/180), 3800, 3000, 0.8, telemetry);
        robot.brake();
        robot.rotateTo(92, 1500, 0.2, true, 1, telemetry);
        robot.brake();

        robot.moveTo(90 * (Math.PI/180), 1180, 1500, 0.8, telemetry);
        robot.brake();

        robot.arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.armUp(robot.armHover + 100);
        robot.arm.setPower(0.04);

        robot.moveTo(0 * (Math.PI/180), 750, 1000, 0.8, telemetry);
        robot.arm.setPower(0);
        robot.brake();

        robot.claw_right.setPosition(openRight);
        robot.claw_left.setPosition(openLeft);
        robot.right_wiper.setPosition(rWiperStow);
        sleep(70);

        robot.moveTo(245 * (Math.PI/180), 800, 1000, 0.8, telemetry);
        robot.claw_rot.setPosition(rotGrab);
        robot.moveTo(340 * (Math.PI/180), 950, 1000, 0.8, telemetry);
    }

    private void routineThree() {

        robot.moveRotateTo(270, 336 * (Math.PI/180), 0.15, true, 1.0, 0.41, telemetry);

        robot.fling.setPosition(0.1);
        sleep(50);

        robot.moveTo(0, 170, 500, 0, telemetry);

        robot.right_wiper.setPosition(rWiperGrab);
        sleep(600);
        robot.claw_right.setPosition(closeRight);
        sleep(70);

        robot.moveTo(180 * (Math.PI/180), 100, 500, 0, telemetry);
        robot.brake();
        robot.moveTo(100 * (Math.PI/180), 1000, 500, 0, telemetry);
        robot.brake();


        // Common Part
        robot.claw_rot.setPosition(rotHover);
        robot.arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.armUp(robot.armHover);
        robot.setVelocities(new double[]{-400, 400, 400, 400});
        sleep(100);
        robot.claw_rot.setPosition(rotLow + 0.12);

        robot.moveTo(180 * (Math.PI/180), 3800, 3000, 0.8, telemetry);
        robot.brake();
        robot.rotateTo(92, 1500, 0.2, true, 1, telemetry);
        robot.brake();

        robot.moveTo(90 * (Math.PI/180), 1280, 1500, 0.8, telemetry);
        robot.brake();

        robot.arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.armUp(robot.armHover + 100);
        robot.arm.setPower(0.04);

        robot.moveTo(0 * (Math.PI/180), 750, 1000, 0.8, telemetry);
        robot.arm.setPower(0);
        robot.brake();

        robot.claw_right.setPosition(openRight);
        robot.claw_left.setPosition(openLeft);
        robot.right_wiper.setPosition(rWiperStow);
        sleep(70);

        robot.moveTo(245 * (Math.PI/180), 1000, 1000, 0.8, telemetry);
        robot.claw_rot.setPosition(rotGrab);
        robot.moveTo(340 * (Math.PI/180), 1150, 1000, 0.8, telemetry);
    }



    private void initTfod() {

        // Create the TensorFlow processor by using a builder.
        tfod = new TfodProcessor.Builder()

                // With the following lines commented out, the default TfodProcessor Builder
                // will load the default model for the season. To define a custom model to load,
                // choose one of the following:
                //   Use setModelAssetName() if the custom TF Model is built in as an asset (AS only).
                //   Use setModelFileName() if you have downloaded a custom team model to the Robot Controller.
                .setModelAssetName(TFOD_MODEL_ASSET)
                //.setModelFileName(TFOD_MODEL_FILE)

                // The following default settings are available to un-comment and edit as needed to
                // set parameters for custom models.
                .setModelLabels(LABELS)
                .setIsModelTensorFlow2(true)
                //.setIsModelQuantized(true)
                .setModelInputSize(320)
                //.setModelAspectRatio(16.0 / 9.0)

                .build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        // Choose a camera resolution. Not all cameras support all resolutions.
        builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(tfod);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Set confidence threshold for TFOD recognitions, at any time.
        tfod.setMinResultConfidence(0.75f);

        // Disable or re-enable the TFOD processor at any time.
        //visionPortal.setProcessorEnabled(tfod, true);

    }

    private void telemetryTfod() {

        List<Recognition> currentRecognitions = tfod.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());

        // Step through the list of recognitions and display info for each one.
//        for (Recognition recognition : currentRecognitions) {
//            double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
//            double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;
//
//            telemetry.addData(""," ");
//            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
//            telemetry.addData("- Position", "%.0f / %.0f", x, y);
//            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
//        }
    }
}