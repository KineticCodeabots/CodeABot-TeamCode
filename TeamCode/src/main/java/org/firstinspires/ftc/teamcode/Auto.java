package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManager;
import com.qualcomm.robotcore.eventloop.opmode.OpModeRegistrar;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.internal.opmode.OpModeMeta;

public class Auto extends LinearOpMode {
    public static int DRIVE_FORWARD_DISTANCE = 1600;
    public static int ARM_POSITION = 1070;
    public static double DRIVE_SPEED = 0.3;
    public static int PARK_STRAFE_DISTANCE = 2000;
    public static int SPECIMEN_PARK_DISTANCE = 3000;

    public enum AutoStart {
        SPECIMEN,
        NONE
    }

    public enum AutoEnding {
        PARK,
        SAMPLE_PICKUP,
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
        waitForStart();

        robot.claw.setPosition(Robot.CLAW_CLOSED_POSITION);
        sleep(1000);

        if (autoStart == AutoStart.SPECIMEN) {
            robot.armMotor.setTargetPosition(ARM_POSITION);
            robot.armMotor.setPower(0.3);
            robot.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            autoRobot.drive(DRIVE_FORWARD_DISTANCE, DRIVE_SPEED);
            robot.armMotor.setTargetPosition(700);
            robot.armMotor.setPower(0.4);
            autoRobot.drive((1000 - DRIVE_FORWARD_DISTANCE), 0.2);

            robot.claw.setPosition(Robot.CLAW_OPEN_POSITION);
            robot.armMotor.setTargetPosition(100);
            robot.armMotor.setPower(0.1);
            autoRobot.drive(-700, DRIVE_SPEED);


        } else {
            autoRobot.drive(200, DRIVE_SPEED);

        }
        if (autoEnding == AutoEnding.PARK) {
            autoRobot.strafe(SPECIMEN_PARK_DISTANCE, DRIVE_SPEED);
        } else if (autoEnding == AutoEnding.SAMPLE_PICKUP) {
            autoRobot.strafe(2800, DRIVE_SPEED);
            autoRobot.drive(900, DRIVE_SPEED);

            robot.armMotor.setTargetPosition(100);
            robot.armMotor.setPower(0.1);
            robot.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.liftMotor.setTargetPosition(400);
            robot.liftMotor.setPower(0.8);
            robot.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            while (robot.liftMotor.isBusy()) {

            }
            robot.claw.setPosition(Robot.CLAW_CLOSED_POSITION);
            sleep(500);
            robot.armMotor.setTargetPosition(200);
            robot.armMotor.setPower(0.2);
            sleep(200);
            autoRobot.drive(-900, DRIVE_SPEED);
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
                OpModeMeta meta = new OpModeMeta.Builder()
                        .setName(String.format("Auto - %s - %s", autoStart, autoEnding))
                        .setFlavor(OpModeMeta.Flavor.AUTONOMOUS)
                        .setTransitionTarget("TeleOp")
                        .build();

                manager.register(meta, new Auto(autoStart, autoEnding));

            }
        }
    }
}
