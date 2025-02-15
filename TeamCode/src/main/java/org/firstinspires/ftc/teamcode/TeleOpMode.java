package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp(name = "TeleOp")
@Config
public class TeleOpMode extends GamepadOpMode {
    // Constants
    public static double ARM_MAX_POWER = 0.6;
    public static double ARM_DOWN_POWER = 0.2;
    public static double ARM_PRECISION_SPEED = 0.2;
    public static double MAX_DRIVE_SPEED = 0.5;
    public static double MAX_TURN_SPEED = 0.4;
    public static double PRECISION_SPEED = 0.2;
    public static int MAX_LIFT_POSITION = 1200;

    public static int MAX_LIFT_POSITION_HORIZONTAL = 1000;
    public static int LIFT_SLOWDOWN_DISTANCE = 50;
    public static double ARM_ANGLE_OFFSET = -0.5;
    public static int ARM_MAX_POSITION = 1200;
    public static int ARM_SLOWDOWN_DISTANCE = 150;

    final double ARM_TICKS_PER_REV =
            ((20.0) // HD Hex Motor 20:1 Planetary Gearbox
                    * (90.0 / 45.0) // Gear chain
                    * (125.0 / 60.0)
                    * (125.0 / 60.0)
            ) * 28; // HD Hex Motor 28 Pulses per Revolution

    // Robot and PID controllers
    private final Robot robot = new Robot(this);
    private final PID armAntiGravityPID = new ArmAGPID(0, 0.0007, 0);
    private final PID liftPositionPID = new PIDAW(0.1, 0.001, 0);

    // State variables
    private static boolean debugMode = false;

    private boolean armPrecisionMode = false;
    private boolean fieldCentric = true;
    private boolean operatorLimitsDisabled = false;
    private boolean liftMoveToPosition = false;
    private int liftTargetPosition = 0;

    @Override
    public void init() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        telemetry.addData("<b>Status</b>", "<font color='orange'>Initializing</font>");
        telemetry.update();
        robot.init();
        robot.armMotor.setCurrentAlert(5, CurrentUnit.AMPS);
        robot.liftMotor.setCurrentAlert(5, CurrentUnit.AMPS);
        telemetry.addData("<b>Status</b>", "<font color='green'>Initialized</font>");
        telemetry.update();
    }

    @Override
    public void start() {
        robot.toggleClaw();
    }

    @Override
    public void loop() {
        telemetry.addData("<b>Status</b>", "<font color='blue'>Running</font>");
        updateGamepads();

        if ((currentGamepad1.start && currentGamepad1.y && !previousGamepad1.y) || (currentGamepad2.start && currentGamepad2.y && !previousGamepad2.y)) {
            debugMode = !debugMode;
        }
        if (debugMode) telemetry.addLine("<font color='red'>DEBUG MODE</font>");

        driverLoop();
        operatorLoop();
    }

    private void driverLoop() {
        if (gamepad1.right_stick_button) {
            robot.imu.resetYaw();
        }

        double driveFactor = MAX_DRIVE_SPEED;
        double turnFactor = MAX_TURN_SPEED;
        if (currentGamepad1.right_bumper) {
            driveFactor = 1;
            turnFactor = 1;
        } else if (currentGamepad1.left_bumper) {
            driveFactor = PRECISION_SPEED;
            turnFactor = turnFactor * 0.5;
        }
        if (currentGamepad1.share && !previousGamepad1.share) {
            fieldCentric = !fieldCentric;
        }
        if (fieldCentric) {
            robot.updateMecanumFieldDrive(-stickDeadZoneComp(gamepad1.left_stick_y, 0.1) * driveFactor, stickDeadZoneComp(gamepad1.left_stick_x, 0.1) * driveFactor, stickDeadZoneComp(gamepad1.right_stick_x, 0.1) * turnFactor);
        } else {
            robot.updateMecanumRobotDrive(-gamepad1.left_stick_y * driveFactor, gamepad1.left_stick_x * driveFactor, gamepad1.right_stick_x * turnFactor);
        }

        if (fieldCentric) {
            telemetry.addLine("<font color='orange'>Field Centric Mode</font>");
        } else {
            telemetry.addLine("<font color='red'>Robot Centric Mode</font>");
        }
    }

    private void operatorLoop() {
        if ((currentGamepad2.x && !previousGamepad2.x) || (currentGamepad1.x && !previousGamepad1.x)) {
            robot.toggleClaw();
        }

        if (currentGamepad2.back && !previousGamepad2.back) {
            operatorLimitsDisabled = !operatorLimitsDisabled;
        }
        if (operatorLimitsDisabled) {
            telemetry.addLine("<font color='red'>Operator Limits Disabled</font>");
        }

        double armAngleRads = (robot.armMotor.getCurrentPosition() / ARM_TICKS_PER_REV * 2 * Math.PI) + ARM_ANGLE_OFFSET;
        double cosine = Math.cos(Range.clip(armAngleRads, -Math.PI / 2, Math.PI / 2));
        if (debugMode) telemetry.addData("Arm Angle", armAngleRads);
        if (debugMode) telemetry.addData("Cosine", cosine);

        armLoop(armAngleRads);
        liftLoop(cosine);
    }

    private void armLoop(double armAngleRads) {
        if (currentGamepad2.left_stick_button && !previousGamepad2.left_stick_button) {
            robot.armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        if (currentGamepad2.dpad_up && !previousGamepad2.dpad_up) {
            armAntiGravityPID.reset();
        }
        armPrecisionMode = currentGamepad2.left_bumper;
        if (armPrecisionMode) {
            telemetry.addLine("<font color='orange'>Arm Precision Mode</font>");
        }

        double armOperatorInput = -stickDeadZoneComp(gamepad2.left_stick_y, 0.1);
        if (debugMode) telemetry.addData("Arm Operator Input", armOperatorInput);

        double armAntiGravityCommand = 0;
        if (!currentGamepad2.right_bumper) {
            // Prevent arm from moving when it should not be moving, and limiting the force applied to hopefully not get shock loads idk.
            if (armOperatorInput == 0) {
                if (Math.abs(robot.armMotor.getVelocity()) > 500) {
                    armAntiGravityCommand = 0;
                } else {
                    armAntiGravityCommand = armAntiGravityPID.update(0, Range.clip(robot.armMotor.getVelocity(), -100, 150));
                }
            } else {
                armAntiGravityCommand = armAntiGravityPID.update(0, 0);
            }
        }

        double armCommand;
        armCommand = armOperatorInput;
        if (armCommand < 0) {
            if (armPrecisionMode) {
                armCommand *= Math.max(armAntiGravityCommand * 1.5, 0.01);
            } else {
                armCommand *= ARM_DOWN_POWER;
            }
        } else {
            if (armPrecisionMode) {
                armCommand *= Math.max(armAntiGravityCommand * 2, ARM_PRECISION_SPEED);
            } else {
                armCommand *= ARM_MAX_POWER;
            }
        }

        armCommand += armAntiGravityCommand;
        if (!operatorLimitsDisabled && robot.armMotor.getCurrentPosition() > ARM_MAX_POSITION - ARM_SLOWDOWN_DISTANCE) {
            armCommand = Math.min(armCommand, Math.max((double) (ARM_MAX_POSITION - robot.armMotor.getCurrentPosition()) / ARM_SLOWDOWN_DISTANCE, 0));
        }

        if (debugMode) telemetry.addData("Arm Anti Gravity", armAntiGravityCommand);

        robot.armMotor.setPower(armCommand);

        if (debugMode) telemetry.addData("Arm Power", robot.armMotor.getPower());
//        telemetry.addData("")
        if (debugMode) telemetry.addData("Arm Position", robot.armMotor.getCurrentPosition());
        if (robot.armMotor.isOverCurrent()) {
            telemetry.addLine("<font color='red'>ARM MOTOR EXCEEDED CURRENT LIMIT!</font>");
        }
    }

    private void liftLoop(double cosine) {
        if (currentGamepad2.right_stick_button && !previousGamepad2.right_stick_button) {
            robot.liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }


        double maxLiftLength = Math.max(MAX_LIFT_POSITION_HORIZONTAL / cosine, 0);

        if (!currentGamepad2.start) {
            if (currentGamepad2.y && !previousGamepad1.y) {
                liftMoveToPosition = true;
                liftTargetPosition = MAX_LIFT_POSITION;
                liftPositionPID.reset();
            } else if (currentGamepad2.b && !previousGamepad2.b) {
                liftMoveToPosition = true;
                liftTargetPosition = 400;
                liftPositionPID.reset();
            } else if (currentGamepad2.a && !previousGamepad1.a) {
                liftMoveToPosition = true;
                liftTargetPosition = 0;
                liftPositionPID.reset();
            }
        }

        double liftCommand = -gamepad2.right_stick_y;

        if (!operatorLimitsDisabled) {
            // Slow down the lift when it is near the top or bottom
            if (liftCommand > 0 && robot.liftMotor.getCurrentPosition() >= MAX_LIFT_POSITION - LIFT_SLOWDOWN_DISTANCE) {
                double slowDownFactor = Math.max((double) (MAX_LIFT_POSITION - robot.liftMotor.getCurrentPosition()) / LIFT_SLOWDOWN_DISTANCE, 0);
                liftCommand = Math.min(liftCommand, slowDownFactor);
            } else if (liftCommand < 0 && robot.liftMotor.getCurrentPosition() <= LIFT_SLOWDOWN_DISTANCE) {
                double slowDownFactor = -Math.max((double) robot.liftMotor.getCurrentPosition() / LIFT_SLOWDOWN_DISTANCE, 0);
                liftCommand = Math.max(liftCommand, slowDownFactor);
            }
        }

        if (!operatorLimitsDisabled && robot.liftMotor.getCurrentPosition() > maxLiftLength && liftCommand >= 0) {
            liftMoveToPosition = false;
            robot.liftMotor.setPower(liftPositionPID.update(maxLiftLength, robot.liftMotor.getCurrentPosition()));
        } else {
            if (liftMoveToPosition && liftCommand == 0) {
                int limitedLiftTargetPosition;
                if (!operatorLimitsDisabled) {
                    limitedLiftTargetPosition = Math.min(liftTargetPosition, (int) maxLiftLength);
                } else {
                    limitedLiftTargetPosition = liftTargetPosition;
                }
                robot.liftMotor.setPower(liftPositionPID.update(limitedLiftTargetPosition, robot.liftMotor.getCurrentPosition()));
                int error = Math.abs(limitedLiftTargetPosition - robot.liftMotor.getCurrentPosition());
                if (error < 30) {
                    liftMoveToPosition = false;
                }
            } else {
                liftMoveToPosition = false;
                // slow down the lift when getting close to maxLiftLength
                if (!operatorLimitsDisabled && robot.liftMotor.getCurrentPosition() > maxLiftLength - LIFT_SLOWDOWN_DISTANCE) {
                    liftCommand *= (maxLiftLength - robot.liftMotor.getCurrentPosition()) / LIFT_SLOWDOWN_DISTANCE;
                }
                robot.liftMotor.setPower(liftCommand);
            }
        }

        telemetry.addData("Max Lift Length", maxLiftLength);
        telemetry.addData("Lift Position", robot.liftMotor.getCurrentPosition());

        if (robot.liftMotor.isOverCurrent()) {
            telemetry.addLine("<font color='red'>LIFT MOTOR EXCEEDED CURRENT LIMIT!</font>");
        }
    }

    public static double stickDeadZoneComp(double number, double min) {
        if (number <= -1) {
            return -1.0;
        }
        if (number >= 1) {
            return 1.0;
        }
        if (Math.abs(number) < min) {
            return 0.0;
        }
        if (number > 0) {
            return (number - min) / (1 - min);
        } else {
            return (number + min) / (1 - min);
        }
    }
}
