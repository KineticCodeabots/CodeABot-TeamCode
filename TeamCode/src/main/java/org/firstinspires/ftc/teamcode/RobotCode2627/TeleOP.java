package org.firstinspires.ftc.teamcode.RobotCode2627;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * BIO-BUZZ TeleOp. Driver controls only; hardware and aiming in init_class.
 * Before START: centre turret physically, press gamepad2 A, choose alliance X/B.
 */
@TeleOp(name = "TeleOp", group = "Competition")
public class TeleOP extends OpMode {

    private final init_class drive = new init_class();

    // Manual edge detection, compatible with older FTC SDKs.
    private boolean aWasDown = false;
    private boolean leftWasDown = false;
    private boolean rightWasDown = false;
    private boolean upWasDown = false;

    @Override
    public void init() {
        drive.init(hardwareMap);
        telemetry.addLine("Set alliance and centre turret before START.");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        // Select alliance before the match.
        if (gamepad2.b) drive.setAlliance(init_class.Alliance.RED);
        if (gamepad2.x) drive.setAlliance(init_class.Alliance.BLUE);

        // A must ONLY be pressed with turret physically centred.
        if (gamepad2.a && !aWasDown) {
            drive.zeroTurretAtCenter();
        }
        aWasDown = gamepad2.a;

        telemetry.addData("Alliance (G2 B red / X blue)",
                drive.isAllianceSelected() ? drive.getAlliance() : "NOT SELECTED");
        telemetry.addData("Turret zeroed (G2 A)", drive.isTurretReady());
        telemetry.addData("Encoder ticks", drive.getTurretPosition());
        telemetry.addData("Soft limits (ticks)", "+/-%d", drive.getTurretLimitTicks());
        telemetry.addLine("Do not press A unless physically at centre.");
        telemetry.addLine("Verify gearbox, encoder direction and stops before turret use.");
        telemetry.update();
    }

    @Override
    public void start() {
        // Prevent the INIT A hold from triggering anything after START.
        aWasDown = gamepad2.a;
    }

    @Override
    public void loop() {
        // Gamepad 1: mecanum drive.
        double forward = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;
        drive.driveMotors(forward, strafe, turn);

        // Gamepad 2: intake; 1000 encoder ticks/s (from your original code).
        drive.setIntakeVelocity(gamepad2.left_bumper ? 1000.0 : 0.0);

        // Gamepad 2 D-pad: manually choose scoring/audience group or AUTO.
        boolean left = gamepad2.dpad_left;
        boolean right = gamepad2.dpad_right;
        boolean up = gamepad2.dpad_up;
        if (left && !leftWasDown) {
            drive.setTargetMode(init_class.TargetMode.AUDIENCE);
        } else if (right && !rightWasDown) {
            drive.setTargetMode(init_class.TargetMode.SCORING);
        } else if (up && !upWasDown) {
            drive.setTargetMode(init_class.TargetMode.AUTO);
        }
        leftWasDown = left;
        rightWasDown = right;
        upWasDown = up;

        // Read camera ONCE each loop, share result with turret + shooter.
        drive.updateVision();

        // Hold gamepad2 right bumper: auto-aim. Otherwise manual right stick X.
        drive.controlTurret(gamepad2.right_bumper, gamepad2.right_stick_x);

        // G2 LT: low-speed RPM TEST. G2 RT: distance-based RPM (after calibration).
        // Gate servo remains disabled; nothing feeds game pieces automatically.
        drive.controlFlywheels(gamepad2.left_trigger, gamepad2.right_trigger);

        telemetry.addData("Alliance",
                drive.isAllianceSelected() ? drive.getAlliance() : "NOT SELECTED");
        telemetry.addData("Target mode", drive.getTargetMode());
        telemetry.addData("Active cell", drive.getActiveCell());
        telemetry.addData("Tag ID", drive.getTargetId());
        telemetry.addData("Tag visible", drive.isTargetVisible());
        telemetry.addData("Tag tx (deg)", "%.2f", drive.getTargetXDegrees());
        telemetry.addData("Camera-to-tag horizontal (m)", "%.2f", drive.getTargetDistanceMeters());
        telemetry.addData("Turret zeroed", drive.isTurretReady());
        telemetry.addData("Turret ticks", drive.getTurretPosition());
        telemetry.addData("Turret angle (deg)", "%.1f", drive.getTurretAngleDegrees());
        telemetry.addData("Turret power", "%.3f", drive.getTurretPower());
        telemetry.addData("Shooter status", drive.getShooterStatus());
        telemetry.addData("Flywheel target RPM", "%.0f", drive.getFlywheelTargetRPM());
        telemetry.addData("Left RPM", "%.0f", drive.getLeftFlywheelRPM());
        telemetry.addData("Right RPM", "%.0f", drive.getRightFlywheelRPM());
        telemetry.addData("At speed", drive.isFlywheelAtSpeed());
        telemetry.update();
    }

    @Override
    public void stop() {
        drive.stopAll();
    }
}
