package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManager;
import com.qualcomm.robotcore.eventloop.opmode.OpModeRegistrar;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.internal.opmode.OpModeMeta;

public class Auto extends LinearOpMode {
    public static int DRIVE_FORWARD_DISTANCE = 1600;
    public static int SPECIMEN_ARM_POSITION = 1070;
    public static double DRIVE_SPEED = 0.3;
    public static int PARK_STRAFE_DISTANCE = 3000;
    public static int SAMPLE_PICKUP_STRAFE_DISTANCE = 2800;
    public static long START_DELAY = 0;

    public enum AutoStart {
        SPECIMEN,
        NONE
    }

    public enum AutoEnding {
        PARK,
        SAMPLE_PICKUP,
        LEVEL_1_PARK
    }

    public AutoStart autoStart;
    public AutoEnding autoEnding;

    private final Robot robot = new Robot(this);
    private final AutoRobot autoRobot = new AutoRobot(this, robot);

    public Auto(AutoStart autoStart, AutoEnding autoEnding) {
        this.autoStart = autoStart;
        this.autoEnding = autoEnding;
    }

    @Override
    public void runOpMode() {
        telemetry.addData("Auto Mode", autoStart);
        telemetry.update();
        robot.init();
        robot.reset();
//        telemetry.addLine(String.valueOf(robot.armMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION)));
//        robot.claw.getController().pwmEnable();
        if (autoStart == AutoStart.SPECIMEN) {
            robot.claw.setPosition(Robot.CLAW_CLOSED_POSITION);
        }
        waitForStart();
        // drive: 45.5
        // strafe: 42

        sleep(START_DELAY);

        if (autoStart == AutoStart.SPECIMEN) {

            robot.claw.setPosition(Robot.CLAW_CLOSED_POSITION);
//            sleep(3000);
            robot.armMotor.setTargetPosition(SPECIMEN_ARM_POSITION);
            robot.armMotor.setPower(0.3);
            robot.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            autoRobot.drive(DRIVE_FORWARD_DISTANCE, DRIVE_SPEED);
            robot.armMotor.setTargetPosition(700);
            robot.armMotor.setPower(0.4);
            autoRobot.drive((1000 - DRIVE_FORWARD_DISTANCE), 0.2);

            robot.claw.setPosition(Robot.CLAW_OPEN_POSITION);
            robot.armMotor.setTargetPosition(100);
            robot.armMotor.setPower(0.2);
            autoRobot.drive(-700, DRIVE_SPEED);
        } else {
            autoRobot.drive(300, DRIVE_SPEED);
        }
        if (autoEnding == AutoEnding.PARK) {

            autoRobot.strafe(PARK_STRAFE_DISTANCE, DRIVE_SPEED);
        } else if (autoEnding == AutoEnding.LEVEL_1_PARK) {
            robot.armMotor.setTargetPosition(800);
            robot.armMotor.setPower(0.3);
            autoRobot.strafe(-2000, DRIVE_SPEED);
            autoRobot.drive(2800, DRIVE_SPEED);
            robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            robot.frontLeftMotor.setPower(0.3);
            robot.backLeftMotor.setPower(0.3);
            robot.frontRightMotor.setPower(-0.3);
            robot.backRightMotor.setPower(-0.3);

            while (true) {
                double botHeading = robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
                telemetry.addData("Bot Heading", botHeading);
                telemetry.update();
                // right 90 degree turn
                if (botHeading < -90) {
                    break;
                }

            }
            robot.frontLeftMotor.setPower(0);
            robot.backLeftMotor.setPower(0);
            robot.frontRightMotor.setPower(0);
            robot.backRightMotor.setPower(0);

            autoRobot.drive(1000, DRIVE_SPEED);

            robot.armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.armMotor.setPower(0);

        } else if (autoEnding == AutoEnding.SAMPLE_PICKUP) {
            robot.claw.setPosition(Robot.CLAW_OPEN_POSITION);
            robot.armMotor.setTargetPosition(100);
            robot.armMotor.setPower(0.2);
            robot.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            autoRobot.strafe(SAMPLE_PICKUP_STRAFE_DISTANCE, 0.25);
            autoRobot.drive(750, 0.2);

            robot.liftMotor.setTargetPosition(400);
            robot.liftMotor.setPower(0.8);
            robot.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            while (robot.liftMotor.isBusy()) {

            }
            robot.armMotor.setTargetPosition(0);
            robot.armMotor.setPower(0.1);
            sleep(500);
            robot.claw.setPosition(Robot.CLAW_CLOSED_POSITION);
            sleep(500);
            robot.armMotor.setTargetPosition(200);
            robot.armMotor.setPower(0.2);
            sleep(200);
            autoRobot.drive(-750, DRIVE_SPEED);
        }


        while (opModeIsActive()) {
            telemetry.addLine("Auto Complete");
            telemetry.update();
        }
    }

    @OpModeRegistrar
    public static void register(OpModeManager manager) {
        for (AutoStart autoStart : AutoStart.values()) {
            for (AutoEnding autoEnding : AutoEnding.values()) {
                String name = (autoStart == AutoStart.NONE)
                        ? autoEnding.toString()
                        : String.format("%s -> %s", autoStart, autoEnding);

                OpModeMeta meta = new OpModeMeta.Builder()
                        .setName(name)
                        .setFlavor(OpModeMeta.Flavor.AUTONOMOUS)
                        .setTransitionTarget("TeleOp")
                        .build();

                manager.register(meta, new Auto(autoStart, autoEnding));
            }
        }
    }
}
