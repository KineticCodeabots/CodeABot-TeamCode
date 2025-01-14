package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManager;
import com.qualcomm.robotcore.eventloop.opmode.OpModeRegistrar;

import org.firstinspires.ftc.robotcore.internal.opmode.OpModeMeta;

public class Auto extends LinearOpMode {
    public enum Alliance {
        BLUE,
        RED;

        @NonNull
        @Override
        public String toString() {
            switch (this) {
                case BLUE:
                    return "Blue";
                case RED:
                    return "Red";
                default:
                    throw new IllegalArgumentException();
            }
        }
    }

    public enum StartingLocation {
        LEFT,
        RIGHT;

        @NonNull
        @Override
        public String toString() {
            switch (this) {
                case LEFT:
                    return "Left";
                case RIGHT:
                    return "Right";
                default:
                    throw new IllegalArgumentException();
            }
        }
    }

    public Alliance alliance;
    public StartingLocation startingLocation;

    private final Robot robot = new Robot(this);
    private final AutoRobot autoRobot = new AutoRobot(this, robot);

    public Auto(Alliance alliance, StartingLocation startingLocation) {
        this.alliance = alliance;
        this.startingLocation = startingLocation;
    }

    @Override
    public void runOpMode() {
        telemetry.addData("Alliance", alliance);
        telemetry.addData("Starting Location", startingLocation);
        telemetry.update();
        robot.init();
        waitForStart();
        if (alliance == Alliance.RED) {
            // flip direction
        }
        if (startingLocation == StartingLocation.LEFT) {
            // left auto
        } else {
            // right auto
        }

        autoRobot.drive(250, 0.1, 0.3);
        autoRobot.strafe(1000);
    }

    @OpModeRegistrar
    public static void register(OpModeManager manager) {
        for (Alliance selectedAlliance : Alliance.values()) {
            for (StartingLocation selectedStartingLocation : StartingLocation.values()) {
                OpModeMeta meta = new OpModeMeta.Builder()
                        .setName(String.format("%s / %s", selectedAlliance, selectedStartingLocation))
                        .setGroup(selectedAlliance.toString())
                        .setFlavor(OpModeMeta.Flavor.AUTONOMOUS)
                        .setTransitionTarget("TeleOp")
                        .build();

                manager.register(meta, new Auto(selectedAlliance, selectedStartingLocation));
            }
        }
    }
}
