// =============================================================================
// FILE:    AdelitasTeleopV1.java
// PACKAGE: org.firstinspires.ftc.teamcode.teleops
// =============================================================================
// DESCRIPTION:
//   First TeleOp version for the Adelitas robot.
//   Currently DISABLED (@Disabled) — use AdelitasTeleopV2 for active matches.
//   Kept as a reference. All hardware is accessed through Chassis and Robot.
//
// RULE: OpModes must NEVER declare or initialize hardware directly.
//       All motors and servos belong to their robot/ class.
//       This file only calls methods on Chassis and Robot.
//
// GAMEPAD 1 — Driver (movement):
//   Left  stick Y         → forward / backward
//   Left  stick X         → strafe left / right
//   Right stick X         → rotate left / right
//   Y button              → spin up launcher to target velocity
//   B button              → stop launcher flywheel
//   Right bumper (press)  → fire one shot (triggers full launch sequence)
//
// GAMEPAD 2 — Operator (mechanisms):
//   Right trigger (≥0.1)  → intake forward (collect ball)
//   Left  trigger (≥0.1)  → intake reverse (eject ball)
//   Right bumper          → feeders forward (manual feed)
//   Left  bumper          → feeders reverse (manual eject)
//   A button (hold)       → raise assist arm
//   A button (release)    → lower assist arm
//   D-pad up              → launcher angle up
//   D-pad down            → launcher angle down
//
// DEPENDENCIES:
//   - Chassis.java  (drive motors)
//   - Robot.java    (launcher, intake, feeders, servos)
// =============================================================================
package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.Chassis;
import org.firstinspires.ftc.teamcode.robot.Robot;

@Disabled
@TeleOp(name = "AdelitasTeleopV1", group = "Adelitas")
public class AdelitasTeleopV1 extends OpMode {

    private Chassis chassis;
    private Robot   robot;

    @Override
    public void init() {
        chassis = new Chassis(hardwareMap, telemetry);
        robot   = new Robot(hardwareMap, telemetry);
        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void loop() {
        if (gamepad2.a) {
            robot.upwardAssist();
        } else {
            robot.downwardsAssist();
        }

        if (gamepad2.right_trigger >= 0.1) {
            robot.setIntakePower(1);
        } else if (gamepad2.left_trigger >= 0.1) {
            robot.setIntakePower(-1);
        } else {
            robot.setIntakePower(0);
        }

        if (gamepad2.right_bumper) {
            robot.setFeedersPower(0.5);
        } else if (gamepad2.left_bumper) {
            robot.setFeedersPower(-0.5);
        }

        if (gamepad2.dpad_up) {
            robot.setAngleUp();
        } else if (gamepad2.dpad_down) {
            robot.setAngleDown();
        }

        chassis.mecanumDrive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, false);

        if (gamepad1.y) {
            robot.setLauncherVelocity(robot.LAUNCHER_TARGET_VELOCITY);
        } else if (gamepad1.b) {
            robot.setLauncherVelocity(robot.STOP_SPEED);
        }

        robot.launch(gamepad1.rightBumperWasPressed());

        telemetry.addData("State",      robot.getLaunchState());
        telemetry.addData("motorSpeed", robot.getLauncherVelocity());
        telemetry.update();
    }
}
