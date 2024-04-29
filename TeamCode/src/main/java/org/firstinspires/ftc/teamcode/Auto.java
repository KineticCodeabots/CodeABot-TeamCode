package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManager;
import com.qualcomm.robotcore.eventloop.opmode.OpModeRegistrar;

import org.firstinspires.ftc.robotcore.internal.opmode.OpModeMeta;

public class Auto extends LinearOpMode {
    public enum ALLIANCE {
        BLUE,
        RED
    }

    public enum STARTING_LOCATION {
        LEFT,
        RIGHT
    }

    public ALLIANCE alliance;
    public STARTING_LOCATION startingLocation;

    public Auto(ALLIANCE alliance, STARTING_LOCATION startingLocation) {
        this.alliance = alliance;
        this.startingLocation = startingLocation;
    }

    @Override
    public void runOpMode() {
        telemetry.addData("Alliance", alliance);
        telemetry.addData("Starting Location", startingLocation);
        telemetry.update();
        waitForStart();
        if (alliance == ALLIANCE.RED) {
            // flip direction
        }
        if (startingLocation == STARTING_LOCATION.LEFT) {
            // left auto
        } else {
            // right auto
        }
    }

    @OpModeRegistrar
    public static void register(OpModeManager manager) {
        for (ALLIANCE selectedAlliance : ALLIANCE.values()) {
            for (STARTING_LOCATION selectedStartingLocation : STARTING_LOCATION.values()) {
                OpModeMeta meta = new OpModeMeta.Builder()
                        .setName(String.format("%s:%s", selectedAlliance, selectedStartingLocation))
                        .setGroup(selectedAlliance.toString())
                        .setFlavor(OpModeMeta.Flavor.AUTONOMOUS)
                        .setTransitionTarget("TeleOp")
                        .build();

                manager.register(meta, new Auto(selectedAlliance, selectedStartingLocation));
            }
        }
    }
}
