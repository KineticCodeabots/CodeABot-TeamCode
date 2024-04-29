package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManager;
import com.qualcomm.robotcore.eventloop.opmode.OpModeRegistrar;

import org.firstinspires.ftc.robotcore.internal.opmode.OpModeMeta;

public class Auto extends LinearOpMode {
    public enum Alliance {
        BLUE,
        RED
    }

    public enum StartingLocation {
        LEFT,
        RIGHT
    }

    public Alliance alliance;
    public StartingLocation startingLocation;

    public Auto(Alliance alliance, StartingLocation startingLocation) {
        this.alliance = alliance;
        this.startingLocation = startingLocation;
    }

    @Override
    public void runOpMode() {
        if (alliance == Alliance.RED) {
            // flip direction
        }
        if (startingLocation == StartingLocation.LEFT) {
            // left auto
        } else {
            // right auto
        }
    }

    @OpModeRegistrar
    public static void register(OpModeManager manager) {
        for (Alliance selectedAlliance : Alliance.values()) {
            for (StartingLocation selectedStartingLocation : StartingLocation.values()) {
                OpModeMeta meta = new OpModeMeta.Builder()
                        .setName(String.format("%s:%s", selectedAlliance, selectedStartingLocation))
                        .setGroup(selectedAlliance.toString())
                        .setFlavor(OpModeMeta.Flavor.AUTONOMOUS)
                        .setTransitionTarget("TeleOp")
                        .build();

                class ConfiguredAuto extends Auto {
                    public ConfiguredAuto() {
                        super(selectedAlliance, selectedStartingLocation);
                    }
                }
                manager.register(meta, ConfiguredAuto.class);
            }
        }
    }
}
