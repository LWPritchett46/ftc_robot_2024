package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.TriggerReader;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp (name="cmd TeleOp 😈")
public class CMDTeleOp extends CommandOpMode {
    
    HardwarePushbot robot = new HardwarePushbot();

    GamepadEx gamepad1Ex, gamepad2Ex;

    TriggerReader rTriggerReader1, lTriggerReader1, rTriggerReader2, lTriggerReader2;

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();

        robot.init(hardwareMap);

        gamepad1Ex = new GamepadEx(gamepad1);
        gamepad2Ex = new GamepadEx(gamepad2);

        rTriggerReader1 = new TriggerReader(gamepad1Ex, GamepadKeys.Trigger.RIGHT_TRIGGER);
        lTriggerReader1 = new TriggerReader(gamepad1Ex, GamepadKeys.Trigger.LEFT_TRIGGER);
        rTriggerReader2 = new TriggerReader(gamepad2Ex, GamepadKeys.Trigger.RIGHT_TRIGGER);
        lTriggerReader2 = new TriggerReader(gamepad2Ex, GamepadKeys.Trigger.LEFT_TRIGGER);
    }
}
