package org.firstinspires.ftc.teamcode.resources;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.HardwarePushbot;

@TeleOp(name="Testing Motors", group="Autonomous")
@Disabled
public class TestMotors extends OpMode {

    private boolean rightBumper, leftBumper = false;

    HardwarePushbot robot = new HardwarePushbot();

    @Override
    public void init() {
        robot.init(hardwareMap);
    }

    @Override
    public void init_loop() {
    }


    public void start() {

    }

    @Override
    public void loop() {



        // Test DriveTrain Motors
        if (gamepad1.x){
            robot.claw_rot.setPosition(0);
        }
        else if (gamepad1.b){
            robot.claw_rot.setPosition(1);
        }
        else if (gamepad1.y){
            robot.claw_rot.setPosition(.5);
        }
        else if (gamepad1.a){
            robot.claw_rot.setPosition(.7);
        }
        else{
            robot.brake();
        }

        //Test Arm and Claw


        //telemetry.addData("Velocity", robot.left_front.getVelocity());
//        telemetry.addData("Arm Position,", robot.arm.getCurrentPosition());
//        telemetry.addData("Claw Position", robot.claw.getPosition());
//        telemetry.addData("Claw Rotation", robot.claw_rot.getPosition());
//        telemetry.update();
    }
}