package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "TeleOp")
@Config
public class TeleOpMode extends GamepadOpMode {
    public static double ARM_MAX_POWER = 0.6;
    public static double MAX_DRIVE_SPEED = 0.5;
    public static double MAX_TURN_SPEED = 0.4;
    public static double CROUCH_SPEED = 0.2;

    private final Robot robot = new Robot(this);
    private final PID armSlowdownPID = new PID(0, 0.00005, 0.000001);

    private boolean crouching = false;

    @Override
    public void init() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        telemetry.addData("Status", "Initializing");
        telemetry.update();
        robot.init();
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        telemetry.addData("Status", "Running");
        updateGamepads();

        if (gamepad1.options) {
            robot.imu.resetYaw();
        }

        if (currentGamepad1.left_bumper && !previousGamepad1.left_bumper) {
            crouching = !crouching;
        }

        double driveFactor = MAX_DRIVE_SPEED;
        double turnFactor = MAX_TURN_SPEED;
        if (crouching) {
            driveFactor = CROUCH_SPEED;
            turnFactor = turnFactor * 0.5;
        } else if (currentGamepad1.right_bumper) {
            driveFactor = 1;
            turnFactor = 1;
        }
        robot.updateMecanumFieldDrive(-gamepad1.left_stick_y * driveFactor, gamepad1.left_stick_x * driveFactor, gamepad1.right_stick_x * turnFactor);  // TODO: should I add option to switch between field and robot centric?
        double armCommand = Range.scale(-gamepad2.left_stick_y * ARM_MAX_POWER, 0.3, 1, 0, 1);
        if (armCommand < 0) {
            armCommand = armCommand * 0.2;
        }
        telemetry.addData("Arm Command", armCommand);

        if (armCommand == 0) {
            // Prevent arm from moving when it should not be moving, and limiting the force applied to hopefully not get shock loads idk.
            double armSlowdown;
//            if (Math.abs(robot.armMotor.getVelocity()) > 300) {
//                armSlowdown = 0;
//            } else {
//                armSlowdown = armSlowdownPID.update(0, Range.clip(robot.armMotor.getVelocity(), -100, 100));
//            }
            armSlowdown = armSlowdownPID.update(0, Range.clip(robot.armMotor.getVelocity(), -100, 100));
            telemetry.addData("Arm Slowdown", armSlowdown);

            robot.armMotor.setPower(armSlowdown);
        } else {
            armSlowdownPID.reset();
            robot.armMotor.setPower(armCommand);
        }

        // TODO: arm current warning
        // TODO: lift limits
        robot.liftMotor.setPower(-gamepad2.right_stick_y * Robot.LIFT_MAX_POWER);

        if (currentGamepad2.cross && !previousGamepad2.cross) {
            robot.toggleClaw();
        }
        telemetry.addData("Claw State", robot.clawState);
        telemetry.addData("Lift Position", robot.liftMotor.getCurrentPosition());
        telemetry.addData("Arm Position", robot.armMotor.getCurrentPosition());
        telemetry.addData("Arm Velocity", robot.armMotor.getVelocity());
        telemetry.addData("Arm Power", robot.armMotor.getPower());
        telemetry.addData("Yaw", robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        // TODO: telemetry
    }
}
