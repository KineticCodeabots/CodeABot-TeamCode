package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp(name = "TeleOp")
@Config
public class TeleOpMode extends GamepadOpMode {
    public static double ARM_MAX_POWER = 0.6;
    public static double MAX_DRIVE_SPEED = 0.5;
    public static double MAX_TURN_SPEED = 0.4;
    public static double CROUCH_SPEED = 0.2;
    public static int MAX_LIFT_POSITION = 1000;
    public static int MAX_LIFT_POSITION_HORIZONTAL = 700;
    //    public static int LIFT_SLOWDOWN_DISTANCE = 100;
    public static double ARM_ANGLE_OFFSET = 0;

    final double ARM_TICKS_PER_REV =
            28 // number of encoder ticks per rotation of the bare motor
                    / (20.0 // HD Hex Motor 20:1 Planetary Gearbox
                    * (45.0 / 90.0) // Gear chain
                    * (60.0 / 125.0)
                    * (60.0 / 125.0)
            );

    private final Robot robot = new Robot(this);
    private final PID armSlowdownPID = new PID(0, 0.002, 0.00001);
    private final PID liftPositionPID = new PIDAW(0.000001, 0.00001, 0);
    private boolean liftMoveToPosition = false;
    private int liftTargetPosition = 0;

    private boolean precisionDriving = false;
    private boolean fieldCentric = true;

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
        double armCommand = Range.scale(-gamepad2.left_stick_y * ARM_MAX_POWER, 0.3, 1, 0, 1);
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

        if (currentGamepad2.right_stick_button && !previousGamepad2.right_stick_button) {
            robot.liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }


        if (currentGamepad2.x && !previousGamepad2.x) {
            robot.toggleClaw();
        }

        if (currentGamepad2.y && !previousGamepad1.y) {
            liftMoveToPosition = true;
            liftTargetPosition = MAX_LIFT_POSITION;
        } else if (currentGamepad2.a && !previousGamepad1.a) {
            liftMoveToPosition = true;
            liftTargetPosition = 0;
        }

        double armAngleRadians = (robot.armMotor.getCurrentPosition() / ARM_TICKS_PER_REV * 2 * Math.PI) + ARM_ANGLE_OFFSET;  // TODO: angle estimation
        double cosine = Math.cos(armAngleRadians);
        double maxLiftLength = MAX_LIFT_POSITION_HORIZONTAL / cosine;

        double liftCommand = -gamepad2.right_stick_y;
        if (liftCommand > 0 && robot.liftMotor.getCurrentPosition() >= MAX_LIFT_POSITION) {
            liftCommand = Math.min(liftCommand, 0);
        } else if (liftCommand < 0 && robot.liftMotor.getCurrentPosition() >= 0) {
            liftCommand = Math.max(liftCommand, 0);
        }
        // TODO: implement slow down

        if (robot.liftMotor.getCurrentPosition() > maxLiftLength && liftCommand >= 0) {
            robot.liftMotor.setPower(liftPositionPID.update(maxLiftLength, robot.liftMotor.getCurrentPosition()));
        } else {
            if (liftMoveToPosition && liftCommand == 0) {
                robot.liftMotor.setPower(liftPositionPID.update(liftTargetPosition, robot.liftMotor.getCurrentPosition()));
                int error = Math.abs(liftTargetPosition - robot.liftMotor.getCurrentPosition());
                if (error < 10) {
                    liftMoveToPosition = false;
                }
            } else {
                liftMoveToPosition = false;

                robot.liftMotor.setPower(liftCommand);
            }

        }

        telemetry.addData("Max Lift Length", maxLiftLength);
        telemetry.addData("Arm Angle", armAngleRadians);
        telemetry.addData("Arm Angle Degrees", Math.toDegrees(armAngleRadians));
        telemetry.addData("Claw State", robot.clawState);
        telemetry.addData("Lift Position", robot.liftMotor.getCurrentPosition());
        telemetry.addData("Lift Power", robot.liftMotor.getPower());
        telemetry.addData("Arm Position", robot.armMotor.getCurrentPosition());
        telemetry.addData("Arm Velocity", robot.armMotor.getVelocity());
        telemetry.addData("Arm Power", robot.armMotor.getPower());
        telemetry.addData("Yaw", robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        // TODO: telemetry
    }
}
