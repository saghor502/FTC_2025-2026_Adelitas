// =============================================================================
// FILE:    AdelitasTeleopV2.java
// PACKAGE: org.firstinspires.ftc.teamcode.teleops
// =============================================================================
// DESCRIPTION:
//   Current active TeleOp for the Adelitas robot.
//   Uses the Chassis and Robot classes instead of direct hardware access.
//   This is the file to modify for new driver-controlled features.
//
// GAMEPAD 1 — Driver (movement):
//   Left  stick Y         → forward / backward
//   Left  stick X         → strafe left / right
//   Right stick X         → rotate left / right
//   Right bumper (hold)   → slow mode (50% power) while driving
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

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.Chassis;
import org.firstinspires.ftc.teamcode.robot.Robot;

@TeleOp(name = "AdelitasTeleopV2", group = "StarterBot")
public class AdelitasTeleopV2 extends OpMode {
    private Chassis chassis;
    private Robot robot;
    @Override
    public void init(){
        chassis = new Chassis(hardwareMap, telemetry);
        robot = new Robot(hardwareMap, telemetry);
    }

    @Override
    public void loop() {
        if(gamepad2.a){
            robot.upwardAssist();
        } else {
            robot.downwardsAssist();
        }

        if (gamepad2.right_trigger >= 0.1){
            robot.setIntakePower(1);
        } else if (gamepad2.left_trigger >= 0.1) {
            robot.setIntakePower(-1);
        }else {
            robot.setIntakePower(0);
        }

        if(gamepad2.right_bumper){
            robot.setFeedersPower(0.5);
        }else if(gamepad2.left_bumper){
            robot.setFeedersPower(-0.5);
        }

        chassis.mecanumDrive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, gamepad1.right_bumper);

        if (gamepad1.y) {
            robot.setLauncherVelocity(robot.LAUNCHER_TARGET_VELOCITY);
        } else if (gamepad1.b) { // stop flywheel
            robot.setLauncherVelocity(robot.STOP_SPEED);
        }
        if(gamepad2.dpad_up){
            robot.setAngleUp();
        } else if (gamepad2.dpad_down) {
            robot.setAngleDown();
        }

        robot.launch(gamepad1.rightBumperWasPressed());
        
        telemetry.addData("State", robot.getLaunchState());
        telemetry.addData("motorSpeed", robot.getLauncherVelocity());
        telemetry.update();
    }
}
