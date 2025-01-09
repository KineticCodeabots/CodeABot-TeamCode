package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp(name = "TeleOp")
@Config
public class TeleOpMode extends GamepadOpMode {
    public static double ARM_MAX_POWER = 0.6;
    public static double MAX_DRIVE_SPEED = 0.5;
    public static double MAX_TURN_SPEED = 0.4;
    public static double CROUCH_SPEED = 0.2;
    public static int MAX_LIFT_POSITION = 1900;
    public static int MAX_LIFT_POSITION_HORIZONTAL = 1600;
    public static int LIFT_SLOWDOWN_DISTANCE = 50;
    public static double ARM_ANGLE_OFFSET = -0.5;

    final double ARM_TICKS_PER_REV =
            ((20.0) // HD Hex Motor 20:1 Planetary Gearbox
                    * (90.0 / 45.0) // Gear chain
                    * (125.0 / 60.0)
                    * (125.0 / 60.0)
            ) * 28;

    private final Robot robot = new Robot(this);
    private final PID armSlowdownPID = new PID(0, 0.002, 0.00001);
    private final PID liftPositionPID = new PIDAW(0.01, 0.1, 0);

    private boolean precisionDriving = false;
    private boolean fieldCentric = true;
    private boolean secondDriverLimitsDisabled = false;

    private boolean liftMoveToPosition = false;
    private int liftTargetPosition = 0;

    @Override
    public void init() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        telemetry.addData("Status", "Initializing");
        telemetry.update();
        robot.init();
        robot.armMotor.setCurrentAlert(5, CurrentUnit.AMPS);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        telemetry.addData("Status", "Running");
        updateGamepads();

        firstDriverLoop();
        secondDriverLoop();
    }

    private void firstDriverLoop() {
        if (gamepad1.right_stick_button) {
            robot.imu.resetYaw();
        }

        // Toggle precision driving
        if (currentGamepad1.left_bumper && !previousGamepad1.left_bumper) {
            precisionDriving = !precisionDriving;
        }

        double driveFactor = MAX_DRIVE_SPEED;
        double turnFactor = MAX_TURN_SPEED;
        if (currentGamepad1.right_bumper) {
            driveFactor = 1;
            turnFactor = 1;
        } else if (precisionDriving) {
            driveFactor = CROUCH_SPEED;
            turnFactor = turnFactor * 0.5;
        }
        if (currentGamepad1.share && !previousGamepad1.share) {
            fieldCentric = !fieldCentric;
        }
        if (fieldCentric) {
            robot.updateMecanumFieldDrive(-gamepad1.left_stick_y * driveFactor, gamepad1.left_stick_x * driveFactor, gamepad1.right_stick_x * turnFactor);
        } else {
            robot.updateMecanumRobotDrive(-gamepad1.left_stick_y * driveFactor, gamepad1.left_stick_x * driveFactor, gamepad1.right_stick_x * turnFactor);
        }

        telemetry.addData("Field Centric Mode", fieldCentric);
    }

    private void secondDriverLoop() {
        if (currentGamepad2.x && !previousGamepad2.x) {
            robot.toggleClaw();
        }

        if (currentGamepad2.back && !previousGamepad2.back) {
            secondDriverLimitsDisabled = !secondDriverLimitsDisabled;
        }

        armLoop();
        liftLoop();
    }

    private void armLoop() {
        double armCommand = -gamepad2.left_stick_y * ARM_MAX_POWER;
        if (armCommand < 0) {
            armCommand = armCommand * 0.3;
        }
        telemetry.addData("Arm Command", armCommand);

        // TODO: should be replaced with anti gravity compensation
        if (armCommand == 0) {
            // Prevent arm from moving when it should not be moving, and limiting the force applied to hopefully not get shock loads idk.
            double armSlowdown;
            if (Math.abs(robot.armMotor.getVelocity()) > 500) {
                armSlowdown = 0;
            } else {
                armSlowdown = armSlowdownPID.update(0, Range.clip(robot.armMotor.getVelocity(), -200, 50));
            }
            telemetry.addData("Arm Slowdown", armSlowdown);

            robot.armMotor.setPower(armSlowdown);
        } else {
            telemetry.addData("Arm Slowdown", 0);
            armSlowdownPID.reset();
            robot.armMotor.setPower(armCommand);
        }
        if (robot.armMotor.isOverCurrent()) {
            telemetry.addLine("ARM MOTOR EXCEEDED CURRENT LIMIT!");
        }
    }

    private void liftLoop() {
        double armAngleRadians = (robot.armMotor.getCurrentPosition() / ARM_TICKS_PER_REV * 2 * Math.PI) + ARM_ANGLE_OFFSET;  // TODO: angle estimation
        double cosine = Math.cos(armAngleRadians);
        double maxLiftLength = Math.max(MAX_LIFT_POSITION_HORIZONTAL / cosine, 0);

        if (!currentGamepad2.start) {
            if (currentGamepad2.y && !previousGamepad1.y) {
                liftMoveToPosition = true;
                liftTargetPosition = MAX_LIFT_POSITION;
            } else if (currentGamepad2.b && !previousGamepad2.b) {
                liftMoveToPosition = true;
                liftTargetPosition = 1000;
            } else if (currentGamepad2.a && !previousGamepad1.a) {
                liftMoveToPosition = true;
                liftTargetPosition = 0;
            }
        }

        double liftCommand = -gamepad2.right_stick_y;
        if (!secondDriverLimitsDisabled) {
            // Slow down the lift when it is near the top or bottom
            if (liftCommand > 0 && robot.liftMotor.getCurrentPosition() >= MAX_LIFT_POSITION - LIFT_SLOWDOWN_DISTANCE) {
                double slowDownFactor = Math.max((double) (MAX_LIFT_POSITION - robot.liftMotor.getCurrentPosition()) / LIFT_SLOWDOWN_DISTANCE, 0);
                liftCommand = Math.min(liftCommand, slowDownFactor);
            } else if (liftCommand < 0 && robot.liftMotor.getCurrentPosition() <= LIFT_SLOWDOWN_DISTANCE) {
                double slowDownFactor = -Math.max((double) robot.liftMotor.getCurrentPosition() / LIFT_SLOWDOWN_DISTANCE, 0);
                liftCommand = Math.max(liftCommand, slowDownFactor);
            }
        }

        // TODO: add lift current limit

        if (!secondDriverLimitsDisabled && robot.liftMotor.getCurrentPosition() > maxLiftLength && liftCommand >= 0) {
            liftMoveToPosition = false;
            robot.liftMotor.setPower(liftPositionPID.update(maxLiftLength, robot.liftMotor.getCurrentPosition()));
        } else {
            if (liftMoveToPosition && liftCommand == 0) {
                int limitedLiftTargetPosition;
                if (!secondDriverLimitsDisabled) {
                    limitedLiftTargetPosition = Math.min(liftTargetPosition, (int) maxLiftLength);
                } else {
                    limitedLiftTargetPosition = liftTargetPosition;
                }
                robot.liftMotor.setPower(liftPositionPID.update(limitedLiftTargetPosition, robot.liftMotor.getCurrentPosition()));
                int error = Math.abs(limitedLiftTargetPosition - robot.liftMotor.getCurrentPosition());
                if (error < 20) {
                    liftMoveToPosition = false;
                }
            } else {
                liftMoveToPosition = false;
                robot.liftMotor.setPower(liftCommand);
            }
        }

        telemetry.addData("Max Lift Length", maxLiftLength);
        telemetry.addData("Lift Position", robot.liftMotor.getCurrentPosition());
    }
}
