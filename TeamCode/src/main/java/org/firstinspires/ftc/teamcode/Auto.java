package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManager;
import com.qualcomm.robotcore.eventloop.opmode.OpModeRegistrar;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.internal.opmode.OpModeMeta;

public class Auto extends LinearOpMode {
    public static int DRIVE_FORWARD_DISTANCE = 1600;
    public static int ARM_POSITION = 1070;
    public static double DRiVE_SPEED = 0.2;
    public static int PARK_STRAFE_DISTANCE = 2000;
    public static int SPECIMEN_PARK_DISTANCE = 3000;
    public static boolean DO_PARK = true;

    public enum AutoMode {
        SPECIMEN,
        PARK
    }

    public AutoMode autoMode;

    private final Robot robot = new Robot(this);
    private final AutoRobot autoRobot = new AutoRobot(this, robot);

    public Auto(AutoMode autoMode) {
        this.autoMode = autoMode;
    }

    @Override
    public void runOpMode() {
        telemetry.addData("Auto Mode", autoMode);
        telemetry.update();
        robot.init();
        waitForStart();

        robot.claw.setPosition(Robot.CLAW_CLOSED_POSITION);
        sleep(1000);

        if (autoMode == AutoMode.SPECIMEN) {
            robot.armMotor.setTargetPosition(ARM_POSITION);
            robot.armMotor.setPower(0.2);
            robot.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            autoRobot.drive(DRIVE_FORWARD_DISTANCE, DRiVE_SPEED);
            robot.armMotor.setTargetPosition(700);
            robot.armMotor.setPower(0.3);
            autoRobot.drive((1000 - DRIVE_FORWARD_DISTANCE), DRiVE_SPEED);

            robot.claw.setPosition(Robot.CLAW_OPEN_POSITION);
            robot.armMotor.setTargetPosition(100);
            robot.armMotor.setPower(0.05);
            autoRobot.drive(-700, DRiVE_SPEED);
            if (DO_PARK) {
                autoRobot.strafe(SPECIMEN_PARK_DISTANCE, DRiVE_SPEED);
            }
        } else if (autoMode == AutoMode.PARK) {
            autoRobot.drive(200, DRiVE_SPEED);
            autoRobot.strafe(PARK_STRAFE_DISTANCE, DRiVE_SPEED);
        }


        while (opModeIsActive()) {
            telemetry.addLine("Auto Complete");
            telemetry.update();
        }
    }

    @OpModeRegistrar
    public static void register(OpModeManager manager) {
        for (AutoMode autoMode : AutoMode.values()) {
            OpModeMeta meta = new OpModeMeta.Builder()
                    .setName(String.format("Auto - %s", autoMode))
                    .setFlavor(OpModeMeta.Flavor.AUTONOMOUS)
                    .setTransitionTarget("TeleOp")
                    .build();

            manager.register(meta, new Auto(autoMode));
        }
    }
}
