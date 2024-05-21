package org.firstinspires.ftc.teamcode;

import static android.os.SystemClock.sleep;

import static org.firstinspires.ftc.teamcode.util.Utility.clamp;
import static org.firstinspires.ftc.teamcode.util.Utility.expo;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.TriggerReader;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@TeleOp(name="TeleOp Program", group="TeleOp")

public class TeleOp360Mov extends OpMode {

    // Declare OpMode members.
    HardwarePushbot robot = new HardwarePushbot();
//    AsyncRoutines routines;

    GamepadEx gamepad1Ex;
    GamepadEx gamepad2Ex;

    TriggerReader rTriggerReader1;
    TriggerReader lTriggerReader1;

    TriggerReader rTriggerReader2;
    TriggerReader lTriggerReader2;

    /* 'grab' is helpful to determine whether or not the claws are at a "grabbing" position.
    Depending on the position, the claw will open or close. */
    boolean wiperOpenLeft, wiperOpenRight = true;
    boolean intaking = false;

    static float intakeFold = 0.80f;
    static float intakeOpen = 0.30f;

    public static double closeRight = 0.45;
    public static double openRight = 0.57;

    public static double closeLeft = 0.37;
    public static double openLeft = 0.26;

//    public static double rotGrab = 0.97;
//    public static double rotHover = rotGrab;
//    public static double rotHigh = 0.25;
//    public static double rotLow = 0.65;

    public static float rWiperStow = 1.0f;
    public static float rWiperOpen = 0.1f;
    public static float rWiperGrab = 0.58f;

    public static float lWiperStow = 0.1f;
    public static float lWiperOpen = 1.0f;
    public static float lWiperGrab = 0.55f;

    public static double rotGrab = 0.80;
    public static double rotHover = rotGrab;
    public static double rotHigh = 0.15;
    public static double rotLow = 0.45;


    public static int armHigh = 3550 + HardwarePushbot.playOffset;
    public static int armLow = 3800 + HardwarePushbot.playOffset;

    public double residualPower = 0;

    public static int upperLimit = 4500 + HardwarePushbot.playOffset;
    public static int lowerLimit = 2600 + HardwarePushbot.playOffset;
    public boolean armDeployed = false;


    public enum MovingState{
        MOVE_UP,
        MOVE_DOWN,
        NO_MOVE
    }
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        robot.init(hardwareMap);
        armHigh = robot.armHover + 2300;

//        routines = new AsyncRoutines(0, robot, this);

        gamepad1Ex = new GamepadEx(gamepad1);
        gamepad2Ex = new GamepadEx(gamepad2);

        rTriggerReader1 = new TriggerReader(gamepad1Ex, GamepadKeys.Trigger.RIGHT_TRIGGER);
        lTriggerReader1 = new TriggerReader(gamepad1Ex, GamepadKeys.Trigger.LEFT_TRIGGER);

        rTriggerReader2 = new TriggerReader(gamepad2Ex, GamepadKeys.Trigger.RIGHT_TRIGGER);
        lTriggerReader2 = new TriggerReader(gamepad2Ex, GamepadKeys.Trigger.LEFT_TRIGGER);
    }


    @Override
    public void init_loop() {

//        telemetry.addData("Arm Pos", robot.arm.getCurrentPosition());

        telemetry.update();

    }


    @Override
    public void start() {
//        robot.unfoldingRoutine();
//        routines.run = true;
//        routines.setRoutine(4);
//        routines.start();

        robot.claw_rot.setPosition(rotGrab);

        robot.grabRight = false;
        robot.grabLeft = false;
        robot.claw_right.setPosition(closeRight);
        robot.claw_left.setPosition(closeLeft);
        sleep(100);
        robot.claw_right.setPosition(openRight);
        robot.claw_left.setPosition(openLeft);

    }


    @Override
    public void loop() {
/*
        if(gamepad1.a){
            robot.testMotorPower(1);
        }
        else{
            robot.testMotorPower(0);
        }
*/


        //GamePad 2 ================================================================================


        // Right Claw Controls ---------------------------------------------------------------------
        if (gamepad2Ex.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)){
            if (robot.grabRight){
                // Open Position
                robot.claw_right.setPosition(openRight);
                robot.autoOnUndetectRight = !armDeployed && robot.autoClose && !robot.hardShut;
                robot.autoClose = false;
                if (robot.hardShut){
                    robot.grabRight = !robot.grabRight;
                }
                //robot.claw_rot.setPosition(robot.claw_rot.getPosition() - 0.05);

            }
            else {
                // Close position
                robot.claw_right.setPosition(closeRight);
            }
            robot.grabRight = !robot.grabRight;
        }

        // Left Claw Controls
        if (gamepad2Ex.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)){
            if (robot.grabLeft){
                // Open position
                robot.claw_left.setPosition(openLeft);
                robot.autoOnUndetectLeft = !armDeployed && robot.autoClose && !robot.hardShut;
                robot.autoClose = false;
                if (robot.hardShut){
                    robot.grabLeft = !robot.grabLeft;
                }
                //robot.claw_rot.setPosition(robot.claw_rot.getPosition() + 0.05);

            }
            else {
                // Close position
                robot.claw_left.setPosition(closeLeft);
            }
            robot.grabLeft = !robot.grabLeft;
        }

        // Intake Controls -------------------------------------------------------------------------
//        if (gamepad2.dpad_down /* && intakeTimer == 0 */){
//            //intakeTimer = 5;
//            //if (intaking){
//                robot.intake.setPower(1.0);
//            }
//
//            else{
//                robot.intake.setPower(0);
//            }
        //}



        if (rTriggerReader2.wasJustPressed()){
            //robot.left_wiper.setPosition(0.9);
            //robot.intake_rot.getController().pwmEnable();
//            sleep(100);
//            robot.intake_rot.setPosition(intakeFold);

            //robot.claw_rot.setPosition(robot.claw_rot.getPosition() + 0.1);
            //robot.claw_rot.setPosition(rotLow);
        }
        if (lTriggerReader2.wasJustPressed()){
            //robot.left_wiper.setPosition(0.35);
//            robot.intake_rot.setPosition(intakeOpen);
//            sleep(500);
//            robot.intake_rot.getController().pwmDisable();

            //robot.claw_rot.setPosition(robot.claw_rot.getPosition() - 0.1);
            //robot.claw_rot.setPosition(rotGrab);

        }


//        if (gamepad2Ex.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)){
//            // Release Direction
//            robot.launcher.setPosition(1);
//        }
//        if (gamepad2Ex.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)){
//            // Release Direction
//            robot.launcher.setPosition(0);
//        }

//        if (gamepad2.right_trigger >= 0.5 && rTriggerTimer == 0){
//            robot.launcher.setPosition(robot.launcher.getPosition() + 0.05);
//            rTriggerTimer = 7;
//        }
//        if (gamepad2.left_trigger >= 0.5 && lTriggerTimer == 0){
//            robot.launcher.setPosition(robot.launcher.getPosition() - 0.05);
//            lTriggerTimer = 7;
//        }

        // Arm Control -----------------------------------------------------------------------------
        if (gamepad2Ex.wasJustPressed(GamepadKeys.Button.B)){
            // Low Position

            if(!armDeployed) {

                robot.grabRight = true;
                robot.grabLeft = true;
                robot.claw_right.setPosition(closeRight);
                robot.claw_left.setPosition(closeLeft);
                //robot.brake();

                robot.movingState = MovingState.MOVE_UP;
                robot.arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.isHolding = false;
                residualPower = 0;
                robot.arm.setPower(0.8);

                //robot.claw_rot.setPosition(rotLow);
                robot.arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                armDeployed=true;
                robot.autoClose = false;
            }
            else {


                armDeployed=false;
                robot.claw_rot.setPosition(rotGrab);
                robot.claw_rot.setPosition(rotGrab);
                robot.movingState = MovingState.MOVE_DOWN;
                sleep(200);


                //robot.brake();

                robot.isHolding = false;
                robot.arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.arm.setPower(-0.85);

//                robot.claw_right.setPosition(openRight);
//                robot.claw_left.setPosition(openLeft);
//                robot.claw_right.setPosition(closeRight);
//                robot.claw_left.setPosition(closeLeft);
            }

            //residualPower = 0.07;


//            routines.setRoutine(3);
//            routines.start();
        }

        if (gamepad2.y && robot.arm.getCurrentPosition() > lowerLimit + 200){
            // Move up

            robot.arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.arm.setVelocity(-1000);
            robot.arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//            robot.claw_rot.setPosition(
//                    ((((upperLimit - lowerLimit)/85.0) * robot.arm.getCurrentPosition()) + 180)
//                    * (1.0/180.0)
//            );
            robot.claw_rot.setPosition(
                    (rotHigh + ((clamp(robot.arm.getCurrentPosition(), upperLimit, lowerLimit) - lowerLimit)/
                            (upperLimit-lowerLimit)) * (rotLow-rotHigh)));

            residualPower = 0;

//            routines.setRoutine(0);
//            routines.start();
        }
        else if (gamepad2.a && robot.arm.getCurrentPosition() < upperLimit - 200) {
            // Move down

            robot.arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.arm.setVelocity(1000);
            robot.arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.claw_rot.setPosition(
                    (rotHigh + ((clamp(robot.arm.getCurrentPosition(), upperLimit, lowerLimit) - lowerLimit)/
                            (upperLimit-lowerLimit)) * (rotLow-rotHigh)));
            residualPower = 0;

//            routines.setRoutine(2);
//            routines.start();
        }
        else{

            if (armDeployed && robot.movingState == MovingState.NO_MOVE) {
                robot.arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.arm.setPower(0);
//                robot.arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                robot.claw_rot.setPosition(
                        (rotHigh + ((clamp(robot.arm.getCurrentPosition(), upperLimit, lowerLimit) - lowerLimit)/
                                (upperLimit-lowerLimit)) * (rotLow-rotHigh)));
                residualPower = 0;
            }
            else{
                robot.arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            }
        }

        if (gamepad2Ex.wasJustPressed(GamepadKeys.Button.X)) {
            // Grabbing Position

            if (robot.arm.getCurrentPosition() < robot.armHover/2) {
                robot.autoClose = false;
                robot.grabRight = true;
                robot.grabLeft = true;
                robot.claw_right.setPosition(closeRight);
                robot.claw_left.setPosition(closeLeft);
                robot.claw_rot.setPosition(rotGrab);
                robot.brake();
                sleep(70);
                robot.claw_rot.setPosition(rotHover);
                robot.arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                robot.armUp(robot.armHover);
                residualPower = 0.08;
            }
            else{
                robot.grabRight = true;
                robot.grabLeft = true;
                robot.claw_right.setPosition(closeRight);
                robot.claw_left.setPosition(closeLeft);
                robot.claw_rot.setPosition(rotGrab);
                sleep(50);
                //sleep(50);
                robot.brake();
                //sleep(50);
                robot.armDown();
                residualPower = 0.05;
                robot.grabRight = false;
                robot.grabLeft = false;

                robot.autoOnUndetectRight = true;
                robot.autoOnUndetectLeft = true;

                robot.claw_right.setPosition(openRight);
                robot.claw_left.setPosition(openLeft);

            }

//            routines.setRoutine(1);
//            routines.start();
        }

        if (gamepad2Ex.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
            robot.hardShut = false;
            robot.autoClose = true;
        }

        if (gamepad2Ex.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
            robot.hardShut = true;
            robot.autoClose = false;
        }


        // GamePad 1 ===============================================================================

        double[] powers =
                robot.drive(expo(gamepad1.left_stick_x, 1.5)/1.5,
                        expo(gamepad1.left_stick_y, 1.5)/1.5,
                        expo(gamepad1.right_stick_x, 1.5)/1.5);

        if (gamepad1.right_bumper) {
            for (int i = 0; i < powers.length; i++) {
                powers[i] /= 3.7;
            }
        }
        robot.setVelocities(powers);

        // Arm to lift Position --------------------------------------------------------------------
        if (gamepad1.a){
            robot.autoClose = false;
            robot.brake();
            robot.armUp(armHigh - 1000);
            robot.claw_rot.setPosition(rotHigh);
        }

        // Purge Intake ----------------------------------------------------------------------------
//        if (gamepad1.left_bumper){
//            robot.intake.setPower(-0.8);
//        }


        // Lift Controls ---------------------------------------------------------------------------
        if (gamepad1.dpad_up){
            robot.arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.lift.setVelocity(-1000);
        }
        else if (gamepad1.dpad_down){
            robot.lift.setVelocity(1000000);
            robot.isHolding = false;
            residualPower = 0;
            robot.arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.arm.setPower(0.0);
        }
        else if (gamepad1.dpad_left) {
            robot.lift.setVelocity(0);
        }

        if (gamepad1.x) {
            robot.holdPos += 25;
        } else if (gamepad1.b) {
            robot.holdPos -= 25;
        }

//        if (robot.left_sensor.getDistance(DistanceUnit.MM) < 20) {
//            robot.grabLeft = true;
//            robot.claw_left.setPosition(closeLeft);
//        }
//
//        if (robot.right_sensor.getDistance(DistanceUnit.MM) < 20) {
//            robot.grabRight = true;
//            robot.claw_right.setPosition(closeRight);
//        }



        telemetry.addData("Left sensor distance", robot.left_sensor.getDistance(DistanceUnit.MM));
        telemetry.addData("Right sensor distance", robot.right_sensor.getDistance(DistanceUnit.MM));

        telemetry.addLine();

        telemetry.addData("Left sensor col", robot.left_sensor.getNormalizedColors());
        telemetry.addData("Right sensor col", robot.right_sensor.getNormalizedColors());

        telemetry.addLine();



//        telemetry.addData("Left claw", robot.grabLeft);
//        telemetry.addData("Right claw", robot.grabRight);
//
        telemetry.addData("Claw Rot", robot.claw_rot.getPosition());
//
//
        telemetry.addData("Launcher Pos", robot.launcher.getPosition());
//
//        telemetry.addLine();
//
//        telemetry.addData("Left Wiper", robot.left_wiper.getPosition());
//        telemetry.addData("Right Wiper", robot.right_wiper.getPosition());



        telemetry.addLine();

        telemetry.addData("Arm Pos", robot.arm.getCurrentPosition());
        telemetry.addData("Arm State", robot.movingState);


        float[] lDetectedHSV = new float[3];
        float[] rDetectedHSV = new float[3];
        Color.colorToHSV(robot.left_sensor.getNormalizedColors().toColor(), lDetectedHSV);
        Color.colorToHSV(robot.right_sensor.getNormalizedColors().toColor(), rDetectedHSV);

        telemetry.addLine();
        telemetry.addData("hard shut", robot.hardShut);
        telemetry.addData("Left Sensor value", lDetectedHSV[2]);
        telemetry.addData("Right Sensor value", rDetectedHSV[2]);


        robot.detectedRight = robot.pixelInClaw(robot.right_sensor);
        robot.detectedLeft = robot.pixelInClaw(robot.left_sensor);

        telemetry.addData("Left Sensor detect", robot.detectedLeft);
        telemetry.addData("Right Sensor detect", robot.detectedRight);

        if (robot.autoOnUndetectRight && !robot.detectedRight){
            robot.autoClose = true;
            robot.autoOnUndetectRight = false;
        }

        if (robot.autoOnUndetectLeft && !robot.detectedLeft){
            robot.autoClose = true;
            robot.autoOnUndetectLeft = false;
        }

        if (robot.autoClose && !robot.hardShut) {
            robot.grabRight = robot.detectedRight;
            robot.grabLeft = robot.detectedLeft;
        }

        telemetry.addData("Grab Right", robot.grabRight);
        telemetry.addData("Left Right", robot.grabLeft);


        telemetry.update();

        robot.tick();

        if (robot.grabLeft && robot.autoClose){
            robot.claw_left.setPosition(closeLeft);
        }
        else if (!robot.grabLeft && robot.autoClose){
            robot.claw_left.setPosition(openLeft);
        }


        if (robot.grabRight && robot.autoClose){
            robot.claw_right.setPosition(closeRight);
        }
        else if (!robot.grabRight && robot.autoClose){
            robot.claw_right.setPosition(openRight);
        }

        if (robot.autoClose && robot.detectedRight && robot.detectedLeft && robot.grabRight && robot.grabLeft){
            robot.autoClose = false;
            robot.brake();
            robot.claw_rot.setPosition(rotHover);
            robot.arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.armUp(robot.armHover);
            residualPower = 0.08;
        }

//        telemetry.addData("Arm Pos", robot.arm.getCurrentPosition());
//        telemetry.addData("Arm Vel", robot.arm.getVelocity());


        if (rTriggerReader1.wasJustPressed()) {

//            if (wiperOpenRight) {
//                robot.right_wiper.setPosition(rWiperGrab);
//            } else {
//                robot.right_wiper.setPosition(rWiperOpen);
//            }
//            wiperOpenRight = !wiperOpenRight;
            robot.launcher.setPosition(1);
        }
//
        if (lTriggerReader1.wasJustPressed()) {
//
//            if (wiperOpenLeft) {
//                robot.left_wiper.setPosition(lWiperGrab);
//            } else {
//                robot.left_wiper.setPosition(lWiperOpen);
//            }
//            wiperOpenLeft = !wiperOpenLeft;
        robot.launcher.setPosition(1);
        }


        // Toggle Timers & HoldPos -----------------------------------------------------------------

        if(robot.isHolding){
            robot.holdPos(residualPower);
        }

        gamepad1Ex.readButtons();
        gamepad2Ex.readButtons();

        rTriggerReader1.readValue();
        lTriggerReader1.readValue();

        rTriggerReader2.readValue();
        lTriggerReader2.readValue();
    }


    @Override
    public void stop() {
    }

}