package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;

@TeleOp(name = "Auto Configurator", group = "Auto")
public class AutoConfigurator extends LinearOpMode {
    private static int selectedConfigurationIndex = 0;

    private Gamepad previousGamepad = new Gamepad();
    private Gamepad currentGamepad = new Gamepad();

    @Override
    public void runOpMode() {
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);
        // large bold text saying Auto Configurator
        telemetry.addLine("<font size=\"5\"><b>Auto Configurator</b></font>");
        while (opModeIsActive()) {
            byte[] gamepad1Data = gamepad1.toByteArray();
            byte[] gamepad2Data = gamepad2.toByteArray();

            byte[] gamepadData = new byte[gamepad1Data.length];

            for (int i = 0; i < gamepad1Data.length; i++) {
                gamepadData[i] = (byte) (gamepad1Data[i] | gamepad2Data[i]);
            }

            Gamepad gamepad = new Gamepad();
            gamepad.fromByteArray(gamepadData);

            previousGamepad = currentGamepad;
            currentGamepad = gamepad;


            if (gamepad.x && !previousGamepad.x) {
                Auto.DRiVE_SPEED = 0.3;
                Auto.PARK_STRAFE_DISTANCE = 2000;
                Auto.SPECIMEN_PARK_DISTANCE = 3000;
            }

            if (gamepad.dpad_up && !previousGamepad.dpad_up) {
                selectedConfigurationIndex = Math.max(0, selectedConfigurationIndex - 1);
            } else if (gamepad.dpad_down && !previousGamepad.dpad_down) {
                selectedConfigurationIndex = Math.min(selectedConfigurationIndex + 1, 2);
            }

            Map<String, Object> configurations = new LinkedHashMap<>();
            configurations.put("Speed", Auto.DRiVE_SPEED);
            configurations.put("Park Strafe Distance", Auto.PARK_STRAFE_DISTANCE);
            configurations.put("Specimen Park Distance", Auto.SPECIMEN_PARK_DISTANCE);

            List<Map.Entry<String, Object>> entryList = new ArrayList<>(configurations.entrySet());

            Map.Entry<String, Object> selectedEntry = entryList.get(selectedConfigurationIndex);

            if (Objects.equals(selectedEntry.getKey(), "Speed")) {
                if (gamepad.dpad_right && !previousGamepad.dpad_right) {
                    Auto.DRiVE_SPEED = Math.min(1.0, Auto.DRiVE_SPEED + 0.1);
                } else if (gamepad.dpad_left && !previousGamepad.dpad_left) {
                    Auto.DRiVE_SPEED = Math.max(0.0, Auto.DRiVE_SPEED - 0.1);
                }
            } else if (Objects.equals(selectedEntry.getKey(), "Park Strafe Distance")) {
                if (gamepad.dpad_right && !previousGamepad.dpad_right) {
                    Auto.PARK_STRAFE_DISTANCE += 100;
                } else if (gamepad.dpad_left && !previousGamepad.dpad_left) {
                    Auto.PARK_STRAFE_DISTANCE = Auto.PARK_STRAFE_DISTANCE - 100;
                }
            } else if (Objects.equals(selectedEntry.getKey(), "Specimen Park Distance")) {
                if (gamepad.dpad_right && !previousGamepad.dpad_right) {
                    Auto.SPECIMEN_PARK_DISTANCE += 100;
                } else if (gamepad.dpad_left && !previousGamepad.dpad_left) {
                    Auto.SPECIMEN_PARK_DISTANCE = Auto.SPECIMEN_PARK_DISTANCE - 100;
                }
            }

            int index = 0;
            for (Map.Entry<String, Object> entry : entryList) {
                if (index == selectedConfigurationIndex) {
                    telemetry.addData("<font color=\"blue\">" + entry.getKey() + "</font>", entry.getValue());
                } else {
                    telemetry.addData(entry.getKey(), entry.getValue());
                }
            }

            telemetry.addLine("<font color=\"red\">(X) to reset</font>");


            telemetry.update();
        }
    }
}
