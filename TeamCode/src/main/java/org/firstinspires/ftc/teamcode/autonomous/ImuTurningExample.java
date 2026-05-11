package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.robot.Chassis;
import org.firstinspires.ftc.teamcode.robot.ImuTurning;

// DELETE THIS FILE when the real autonomous is written.
// This is only a turning example using ImuTurning PID.
//
// FIELD LAYOUT (top-down view, robot starts at exact center of the 144" x 144" FTC field):
//
//           ↑ 90°
//           |
//   180° ←─[S]─→ 0°   [S] = robot start (field center), initially facing 0° (forward/right)
//           |
//           ↓ -90°
//
//   Turn sequence:
//     0°  → face right       (starting heading, no turn needed)
//     90° → face up/forward  (quarter turn left)
//    -90° → face down/back   (half turn from 90°, or 3/4 turn right from start)
//    180° → face left        (half turn from start)
//      0° → back to start    (full rotation complete)

@Autonomous(name = "IMU Turning Example", group = "Adelitas Examples")
public class ImuTurningExample extends OpMode {

    private Chassis    chassis;
    private ImuTurning imuTurning;

    private enum State {
        TURN_TO_90,     // from 0° → face up (quarter turn left)
        TURNING_TO_90,
        TURN_TO_NEG90,  // from 90° → face down (half turn)
        TURNING_TO_NEG90,
        TURN_TO_180,    // from -90° → face left (quarter turn left)
        TURNING_TO_180,
        TURN_TO_0,      // from 180° → back to start (half turn, full rotation complete)
        TURNING_TO_0,
        DONE
    }

    private State state;

    @Override
    public void init() {
        chassis    = new Chassis(hardwareMap, telemetry);
        imuTurning = new ImuTurning(hardwareMap, telemetry, chassis);

        state = State.TURN_TO_90;
        telemetry.addLine("IMU Turning Example — ready");
    }

    @Override
    public void loop() {
        switch (state) {

            case TURN_TO_90:
                // Robot is facing 0°. Turn left 90° to face up the field.
                imuTurning.startTurn(90);
                state = State.TURNING_TO_90;
                break;

            case TURNING_TO_90:
                // update() returns false when within tolerance and the robot has stopped.
                if (!imuTurning.update()) {
                    state = State.TURN_TO_NEG90;
                }
                break;

            case TURN_TO_NEG90:
                // Robot is facing 90°. Turn right 180° to face down the field (-90°).
                imuTurning.startTurn(-90);
                state = State.TURNING_TO_NEG90;
                break;

            case TURNING_TO_NEG90:
                if (!imuTurning.update()) {
                    state = State.TURN_TO_180;
                }
                break;

            case TURN_TO_180:
                // Robot is facing -90°. Turn left 90° to face left (180°).
                imuTurning.startTurn(180);
                state = State.TURNING_TO_180;
                break;

            case TURNING_TO_180:
                if (!imuTurning.update()) {
                    state = State.TURN_TO_0;
                }
                break;

            case TURN_TO_0:
                // Robot is facing 180°. Turn back to 0° — full rotation complete.
                imuTurning.startTurn(0);
                state = State.TURNING_TO_0;
                break;

            case TURNING_TO_0:
                if (!imuTurning.update()) {
                    state = State.DONE;
                }
                break;

            case DONE:
                chassis.stopChasis();
                telemetry.addLine("Turning complete — back at 0°");
                break;
        }

        telemetry.addData("State",   state);
        telemetry.addData("Heading", "%.1f°", imuTurning.getHeading());
        telemetry.update();
    }
}
