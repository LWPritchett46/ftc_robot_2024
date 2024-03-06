package org.firstinspires.ftc.teamcode;

import static android.os.SystemClock.sleep;

import static org.firstinspires.ftc.teamcode.Utility.expo;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.TriggerReader;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@TeleOp(name="TeleOp Program", group="TeleOp")

public class TeleOp360Mov extends OpMode {

    // Declare OpMode members.
    HardwarePushbot robot = new HardwarePushbot();
    AsyncRoutines routines;

    GamepadEx gamepad1Ex;
    GamepadEx gamepad2Ex;

    TriggerReader rTriggerReader;
    TriggerReader lTriggerReader;

    /* 'grab' is helpful to determine whether or not the claws are at a "grabbing" position.
    Depending on the position, the claw will open or close. */
    boolean grabLeft, grabRight = true;
    boolean intaking = false;

    static float intakeFold = 0.80f;
    static float intakeOpen = 0.30f;

    public static double closeRight = 0;
    public static double openRight = 1;

    public static double closeLeft = 0.25;
    public static double openLeft = 0;

    public static double rotGrab = 0.75;
    public static double rotHover = rotGrab + 0.02;
    public static double rotHigh = 0;
    public static double rotLow = 0.35;

    public static int armHigh = 3550 + HardwarePushbot.playOffset;
    public static int armLow = 4200 + HardwarePushbot.playOffset;

    public double residualPower = 0;



    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        robot.init(hardwareMap);
        armHigh = robot.armHover + 2300;

        routines = new AsyncRoutines(0, robot, this);

        gamepad1Ex = new GamepadEx(gamepad1);
        gamepad2Ex = new GamepadEx(gamepad2);

        rTriggerReader = new TriggerReader(gamepad2Ex, GamepadKeys.Trigger.RIGHT_TRIGGER);
        lTriggerReader = new TriggerReader(gamepad2Ex, GamepadKeys.Trigger.LEFT_TRIGGER);
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
            if (grabRight){
                // Open Position
                robot.claw_right.setPosition(openRight);
                //robot.claw_rot.setPosition(robot.claw_rot.getPosition() - 0.05);

            }
            else {
                // Close position
                robot.claw_right.setPosition(closeRight);
            }
            grabRight = !grabRight;
        }

        // Left Claw Controls
        if (gamepad2Ex.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)){
            if (grabLeft){
                // Open position
                robot.claw_left.setPosition(openLeft);
                //robot.claw_rot.setPosition(robot.claw_rot.getPosition() + 0.05);

            }
            else {
                // Close position
                robot.claw_left.setPosition(closeLeft);
            }
            grabLeft = !grabLeft;
        }

        // Intake Controls -------------------------------------------------------------------------
        if (gamepad2.dpad_down /* && intakeTimer == 0 */){
            //intakeTimer = 5;
            //if (intaking){
                robot.intake.setPower(1.0);
            }

            else{
                robot.intake.setPower(0);
            }
        //}



        if (rTriggerReader.wasJustPressed()){
            //robot.intake_rot.getController().pwmEnable();
            sleep(100);
            robot.intake_rot.setPosition(intakeFold);
        }
        if (lTriggerReader.wasJustPressed()){
            robot.intake_rot.setPosition(intakeOpen);
            sleep(500);
            robot.intake_rot.getController().pwmDisable();
        }


        if (gamepad2.dpad_left){
            // Release Direction
            robot.launcher.setPosition(1);
        }
        if (gamepad2.dpad_right){
            // Release Direction
            robot.launcher.setPosition(-1);
        }

//        if (gamepad2.right_trigger >= 0.5 && rTriggerTimer == 0){
//            robot.launcher.setPosition(robot.launcher.getPosition() + 0.05);
//            rTriggerTimer = 7;
//        }
//        if (gamepad2.left_trigger >= 0.5 && lTriggerTimer == 0){
//            robot.launcher.setPosition(robot.launcher.getPosition() - 0.05);
//            lTriggerTimer = 7;
//        }

        // Arm Control -----------------------------------------------------------------------------
        if (gamepad2.a) {
            // High Position

            robot.brake();

            robot.armUp(armHigh);

            robot.arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.claw_rot.setPosition(rotHigh);

            residualPower = 0;

//            routines.setRoutine(2);
//            routines.start();
        }
        if (gamepad2.b){
            // Low Position

            robot.brake();

            robot.armUp(armLow);
            robot.isHolding = false;
            robot.arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            robot.claw_rot.setPosition(rotLow);

            //residualPower = 0.07;


//            routines.setRoutine(3);
//            routines.start();
        }

        if (gamepad2.y){
            // Hover Position

            robot.brake();

            robot.claw_rot.setPosition(rotHover);
            sleep(100);
            robot.arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.armUp(robot.armHover);

            residualPower = 0.05;

//            routines.setRoutine(0);
//            routines.start();
        }

        if (gamepad2.x) {
            // Grabbing Position

            robot.brake();

            robot.claw_rot.setPosition(rotGrab);
            robot.armDown();
            robot.claw_right.setPosition(closeRight);
            robot.claw_left.setPosition(closeLeft);

            sleep(80);

            robot.claw_rot.setPosition(rotHover);
            robot.arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.armUp(robot.armHover);

            residualPower = 0.05;

//            routines.setRoutine(1);
//            routines.start();
        }


        // Unfold Routine --------------------------------------------------------------------------
        if (gamepad2Ex.wasJustPressed(GamepadKeys.Button.RIGHT_STICK_BUTTON)){

            robot.brake();

            robot.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.unfoldingRoutine();
            residualPower = 0.05;
        }

        // GamePad 1 ===============================================================================

        double[] powers =
                robot.drive(expo(gamepad1.left_stick_x, 1.5)/1.5,
                        expo(gamepad1.left_stick_y, 1.5)/1.5,
                        expo(gamepad1.right_stick_x, 1.5)/1.5);

        if (gamepad1.right_bumper) {
            for (int i = 0; i < powers.length; i++) {
                powers[i] /= 3;
            }
        }
        robot.setVelocities(powers);

        // Arm to lift Position --------------------------------------------------------------------
        if (gamepad1.a){
            robot.brake();
            robot.armUp(armHigh - 700);
        }

        // Purge Intake ----------------------------------------------------------------------------
        if (gamepad1.left_bumper){
            robot.intake.setPower(-0.8);
        }


        // Lift Controls ---------------------------------------------------------------------------
        if (gamepad1.dpad_up){
            robot.arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.lift.setVelocity(-1000);
        }
        else if (gamepad1.dpad_down){
            robot.lift.setVelocity(1000);
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

//        if (gamepad1.y){
//            robot.rotateTo(0, 8);
//        }
//        if (gamepad1.x){
//            robot.rotateTo(90, 8);
//        }
//        if (gamepad1.a){
//            robot.rotateTo(180, 8);
//        }
//        if (gamepad1.b){
//            robot.rotateTo(-90, 8);
//
//        }
        if(gamepad2.dpad_down) {
            telemetry.addLine("d-pad down");
        }

        telemetry.addData("Claw Rot", robot.claw_rot.getPosition());

        if (intaking) {
            telemetry.addLine("Intaking");
        } else {
            telemetry.addLine("Not Intaking");
        }


        telemetry.addData("Launcher Pos", robot.launcher.getPosition());

        //telemetry.addLine();

//        telemetry.addData("Front_Left", robot.left_front.getVelocity());
//        telemetry.addData("Front_Right", robot.right_front.getVelocity());
//        telemetry.addData("Back_Left", robot.left_back.getVelocity());
//        telemetry.addData("Back_Right", robot.right_back.getVelocity());

        //telemetry.addLine();

        //telemetry.addData("Claw Right", grabRight);
        //telemetry.addData("Claw Left", grabLeft);
        //telemetry.addData("Launch", robot.launcher.getPosition());

        //telemetry.addLine();

//        telemetry.addData("Front_Left", powers[0]);
//        telemetry.addData("Front_Right", powers[1]);
//        telemetry.addData("Back_Left", powers[2]);
//        telemetry.addData("Back_Right", powers[3]);

        telemetry.update();

//        telemetry.addData("Arm Pos", robot.arm.getCurrentPosition());
//        telemetry.addData("Arm Vel", robot.arm.getVelocity());



        // Toggle Timers & HoldPos -----------------------------------------------------------------

        if(robot.isHolding){
            robot.holdPos(residualPower);
        }

        gamepad1Ex.readButtons();
        gamepad2Ex.readButtons();

        rTriggerReader.readValue();
        lTriggerReader.readValue();
    }


    @Override
    public void stop() {
        routines.run = false;
    }

}