package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "TeleOp")
@Config
public class TeleOpMode extends GamepadOpMode {
    public static double MAX_DRIVE_SPEED = 0.5;
    public static double MAX_TURN_SPEED = 0.4;
    public static double CROUCH_SPEED = 0.2;

    private final Robot robot = new Robot(this);
    private final PID armSlowdownPID = new PID(0, 0.00001, 0.000001);

    private boolean crouching = false;

    @Override
    public void init() {
        telemetry.addData("Status", "Initializing");
        telemetry.update();
        robot.init();
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        updateGamepads();

//        robot.updateMecanumRobotDrive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
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
        } else if (currentGamepad1.right_bumper && !previousGamepad1.right_bumper) {
            driveFactor = 1;
        }
        robot.updateMecanumFieldDrive(-gamepad1.left_stick_y * driveFactor, gamepad1.left_stick_x * driveFactor, gamepad1.right_stick_x * turnFactor);  // TODO: should I add option to switch between field and robot centric?
        double armCommand = gamepad2.left_stick_y * Robot.ARM_MAX_POWER;
//        robot.armMotor.setPower(armCommand);
        if (armCommand == 0) {
            // Prevent arm from moving when it should not be moving, and limiting the force applied to hopefully not get shock loads idk.
            double armSlowdown = armSlowdownPID.update(0, robot.armMotor.getVelocity());
            robot.armMotor.setPower(armSlowdown);
        } else {
            armSlowdownPID.reset();
            robot.armMotor.setPower(armCommand);
        }

        // TODO: arm current warning
        // TODO: lift limits
        robot.liftMotor.setPower(gamepad2.right_stick_y * Robot.LIFT_MAX_POWER);
        telemetry.addData("Status", "Running");
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
