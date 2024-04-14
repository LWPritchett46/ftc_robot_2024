package org.firstinspires.ftc.teamcode;

import static android.os.SystemClock.sleep;
import static org.firstinspires.ftc.teamcode.TeleOp360Mov.armLow;
import static org.firstinspires.ftc.teamcode.TeleOp360Mov.closeLeft;
import static org.firstinspires.ftc.teamcode.TeleOp360Mov.closeRight;
import static org.firstinspires.ftc.teamcode.TeleOp360Mov.intakeOpen;
import static org.firstinspires.ftc.teamcode.TeleOp360Mov.openLeft;
import static org.firstinspires.ftc.teamcode.TeleOp360Mov.openRight;
import static org.firstinspires.ftc.teamcode.TeleOp360Mov.rotHover;
import static org.firstinspires.ftc.teamcode.TeleOp360Mov.rotLow;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import org.openftc.apriltag.AprilTagDetection;

import java.util.List;

/**
 * To do:
 * Fix rotation one the first movement of all routines (most up to date is routineThree)
 * Possibly code the logic discussed with Dr. Eva
 * Code the Right routines
 * Code the Left routines
 * Code a routine for just parking
 * Ensure the red and blue detection work consistently in multiple lighting conditions
 * Time everything and make the robot wait as much as possible before crossing the field
 */

@Autonomous(name="AutoRL", group="Autonomous")
public class AutoRL extends OpMode {
    HardwarePushbot robot = new HardwarePushbot();

    int centerX = 320;
    int quadrant = 2;

    double residualPower = 0;

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    // TFOD_MODEL_ASSET points to a model file stored in the project Asset location,
    // this is only used for Android Studio when using models in Assets.
    private static final String TFOD_MODEL_ASSET = "model.tflite";
    private static final String[] LABELS = {"blue", "red"};


    private TfodProcessor tfod;
    private VisionPortal visionPortal;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
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
        for (Recognition recognition : currentRecognitions){
            if (recognition.getConfidence() > highestConfidence){
                highestConfidence = recognition.getConfidence();
                centerX = Math.round((recognition.getLeft() + recognition.getRight())/2);
                quadrant = 1 + Math.floorDiv(centerX, (640/3));
            }
        }

        telemetry.addData("Quadrant", quadrant);
        telemetry.addData("Confidence", highestConfidence);

        telemetry.update();
    }

    @Override
    public void start() {
        visionPortal.stopStreaming();
        visionPortal.setProcessorEnabled(tfod, false);
        robot.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Grab Preloaded Pixels
        robot.claw_right.setPosition(closeRight);
        robot.claw_left.setPosition(closeLeft);
        sleep(100);

        // Unfold Intake
        robot.intake_rot.setPosition(TeleOp360Mov.intakeOpen);
        sleep(500);
        robot.intake_rot.setPosition(TeleOp360Mov.intakeOpen);
        sleep(500);

        // Hover
        residualPower = 0.05;
        robot.claw_rot.setPosition(rotHover);
        robot.arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.armUp(robot.armHover + 50);

        robot.imu.resetYaw();


        //quadrant = 1;

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



        telemetry.update();
    }



    @Override
    public void stop() {
    }

    private void routineOne() {

        robot.moveRotateTo(92, 280 * (Math.PI / 180), 0.1, false, 1.62, 0.35, telemetry);
        robot.brake();

        robot.claw_rot.setPosition(rotHover);
        robot.arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.armUp(robot.armHover - 170);

        residualPower = 0.05;
        sleep(200);
        robot.claw_left.setPosition(openLeft);

        robot.moveRotateTo(315, 210 * (Math.PI / 180), 0.15, true, 1.0, 0.40, telemetry);
        robot.brake();
        robot.setModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.setModes(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.moveTo(180 * (Math.PI/180), 1550, 1800, 1.3, telemetry);
        robot.brake();
        sleep(100);
        robot.rotateTo(0, 1000, 0.15,false, 1.3, telemetry);
        robot.brake();

        robot.armUp(armLow);
        robot.claw_rot.setPosition(rotLow - 0.05);
        robot.isHolding = false;
        robot.arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        sleep(100);

        robot.moveTo(180 * (Math.PI / 180), 500, 700, 1.25, telemetry);
        robot.claw_right.setPosition(openRight);
        sleep(350);

        // Move Back
        robot.moveTo(0 , 150, 1000, 1.25, telemetry);
        robot.claw_rot.setPosition(rotHover);
        robot.armUp(robot.armHover);

        ;
        robot.moveTo(180 * (Math.PI / 180), 550, 1000, 1.25, telemetry);
        robot.brake();
    }

    private void routineTwo() {

        robot.moveRotateTo(92, 272 * (Math.PI / 180), 0.1, false, 1.62, 0.32, telemetry);
        robot.brake();

        robot.claw_rot.setPosition(rotHover);
        robot.arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.armUp(robot.armHover - 170);

        residualPower = 0.05;
        sleep(200);
        robot.claw_left.setPosition(openLeft);

        robot.moveRotateTo(315, 210 * (Math.PI / 180), 0.15, true, 1.0, 0.42, telemetry);
        robot.brake();
        robot.setModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.setModes(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.moveTo(180 * (Math.PI/180), 1800, 1500, 1.3, telemetry);
        robot.brake();
        sleep(100);
        robot.rotateTo(0, 800, 0.15,false, 1.3, telemetry);
        robot.brake();

        robot.armUp(armLow);
        robot.claw_rot.setPosition(rotLow - 0.05);
        robot.isHolding = false;
        robot.arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        sleep(100);

        robot.moveTo(265 * (Math.PI / 180), 80, 700, 1.25, telemetry);
        robot.claw_right.setPosition(openRight);
        sleep(350);

        // Move Back
        robot.moveTo(0 , 200, 800, 1.25, telemetry);
        robot.claw_rot.setPosition(rotHover);
        robot.armUp(robot.armHover);

        ;
        robot.moveTo(180 * (Math.PI / 180), 650, 900, 1.25, telemetry);
        robot.brake();

    }

    private void routineThree() {
        robot.moveTo(274 * (Math.PI / 180), 1400, 1000, 1.25, telemetry);
        robot.brake();
        robot.moveTo(Math.PI, 890, 1000, 1.25, telemetry);

        robot.claw_rot.setPosition(rotHover);
        robot.arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.armUp(robot.armHover + 50);

        residualPower = 0.05;
        robot.claw_left.setPosition(openLeft);

        robot.intake_rot.setPosition(intakeOpen);
        robot.moveTo(100 * (Math.PI / 180), 500, 1000, 1.25, telemetry);
        this.stop();
//        sleep(100);
//        robot.moveTo(Math.PI, 3100, 1400, 1.8, telemetry);
//        sleep(100);
//        robot.setModes(DcMotor.RunMode.RUN_USING_ENCODER);
//        robot.rotateTo(93, 1000, 0.15, 1.3);
//        sleep(100);
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