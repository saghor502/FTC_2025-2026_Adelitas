package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.robot.Chassis;
import org.firstinspires.ftc.teamcode.robot.ImuTurning;
import org.firstinspires.ftc.teamcode.robot.Odometry;

// DELETE THIS FILE when the real autonomous is written.
// This is only a navigation example using Odometry + BFS pathfinding.
//
// FIELD LAYOUT (top-down view, robot starts at exact center of the 144" x 144" FTC field):
//
//   (0,144) ──────────────────── (144,144)
//      |          [C]                |       [C] = corner target  (24" forward, 24" left)
//      |           ↑                 |       [F] = forward target (24" straight forward)
//      |    [C]←──[F]               |       [S] = start position (field center)
//      |           ↑                 |
//      |          [S]  ← robot starts here, facing UP (+X direction)
//      |                             |
//   (0,0)  ──────────────────── (144,0)
//
//   Path:  [S](72,72) → [F](96,72) → [C](96,96) → back to [S](72,72) facing 90°

@Autonomous(name = "Odometry Navigation Example", group = "Adelitas Examples")
public class OdometryNavigationExample extends OpMode {

    private Chassis    chassis;
    private ImuTurning imuTurning;
    private Odometry   odometry;

    // The FTC field is 144"x144". Robot starts at the center: (72", 72").
    // X = forward/backward axis. Y = left/right axis.
    private static final double START_X   = 72; // center of field
    private static final double START_Y   = 72; // center of field
    private static final double START_HDG = 0;  // facing forward (0°)

    private enum State {
        PLAN_TO_FORWARD,    // planning path: center → 24" forward
        DRIVE_TO_FORWARD,   // executing that path
        PLAN_TO_CORNER,     // planning path: forward point → 24" to the left
        DRIVE_TO_CORNER,    // executing that path
        PLAN_BACK_TO_START, // planning path: corner → back to center
        DRIVE_BACK_TO_START,// executing that path
        DONE
    }

    private State state;

    @Override
    public void init() {
        chassis    = new Chassis(hardwareMap, telemetry);
        // ImuTurning is created first — Odometry reads heading through it,
        // so the IMU is only initialized once here.
        imuTurning = new ImuTurning(hardwareMap, telemetry, chassis);
        odometry   = new Odometry(hardwareMap, telemetry, chassis, imuTurning);

        // Tell odometry where the robot physically is on the field right now
        odometry.resetPose(START_X, START_Y, START_HDG);

        state = State.PLAN_TO_FORWARD;
        telemetry.addLine("Odometry Navigation Example — ready");
    }

    @Override
    public void loop() {
        // update() must be called every loop so pose tracking stays accurate
        odometry.update();

        switch (state) {

            case PLAN_TO_FORWARD:
                // Robot is at center (72", 72"). Drive 24" straight forward → (96", 72").
                // Arrive still facing 0° (forward).
                odometry.planPath(96, 72, 0);
                state = State.DRIVE_TO_FORWARD;
                break;

            case DRIVE_TO_FORWARD:
                // BFS is executing. navigatePath() returns false when the waypoint is reached.
                if (!odometry.navigatePath()) {
                    state = State.PLAN_TO_CORNER;
                }
                break;

            case PLAN_TO_CORNER:
                // Robot is now at (96", 72"). Strafe 24" to the left → (96", 96").
                // Arrive facing 90° (turned left).
                odometry.planPath(96, 96, 90);
                state = State.DRIVE_TO_CORNER;
                break;

            case DRIVE_TO_CORNER:
                if (!odometry.navigatePath()) {
                    state = State.PLAN_BACK_TO_START;
                }
                break;

            case PLAN_BACK_TO_START:
                // Robot is at (96", 96"). Navigate back to field center (72", 72").
                // Arrive facing 180° (now pointing backward from original start).
                odometry.planPath(72, 72, 180);
                state = State.DRIVE_BACK_TO_START;
                break;

            case DRIVE_BACK_TO_START:
                if (!odometry.navigatePath()) {
                    state = State.DONE;
                }
                break;

            case DONE:
                chassis.stopChasis();
                telemetry.addLine("Navigation complete — back at field center");
                break;
        }

        telemetry.addData("State",   state);
        telemetry.addData("Pose X",  "%.2f\"", odometry.getX());
        telemetry.addData("Pose Y",  "%.2f\"", odometry.getY());
        telemetry.addData("Heading", "%.1f°",  odometry.getHeadingDeg());
        telemetry.update();
    }
}
