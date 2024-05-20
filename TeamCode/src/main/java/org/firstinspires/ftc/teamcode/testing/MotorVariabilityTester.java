package org.firstinspires.ftc.teamcode.testing;

import android.annotation.SuppressLint;
import android.os.Environment;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.io.FileWriter;
import java.io.IOException;
import java.time.LocalDateTime;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.HashMap;
import java.util.Map;

@TeleOp(name = "Motor Variability Tester", group = "Testing")
public class MotorVariabilityTester extends OpMode {
    private static final Comparator<Map.Entry<String, DcMotor>> motorComparator = Comparator.comparing(
            entry -> entry.getValue().getPortNumber()
    );
    private ArrayList<Map.Entry<String, DcMotor>> motors = null;
    private int selectedIndex = 0;
    private final Gamepad currentGamepad = new Gamepad();
    private final Gamepad previousGamepad = new Gamepad();

    private String selectedMotorName = null;
    private DcMotorEx selectedMotor = null;
    private final ElapsedTime motorRuntime = new ElapsedTime();
    private double motorPower = 0.05;
    private static final int SAMPLES = 500;
    private final double[] motorVelocitySamples = new double[SAMPLES];
    private final double[] motorCurrentSamples = new double[SAMPLES];
    private int motorSampleIndex = 0;

    private final HashMap<Double, MotorPowerResults> motorPowerResults = new HashMap<>();

    private boolean complete = false;
    private boolean savedResults = false;

    private static class MotorPowerResults {
        double velocityMean;
        double velocityStdDev;
        double currentMean;
        double currentStdDev;

        public MotorPowerResults(double velocityMean, double velocityStdDev, double currentMean, double currentStdDev) {
            this.velocityMean = velocityMean;
            this.velocityStdDev = velocityStdDev;
            this.currentMean = currentMean;
            this.currentStdDev = currentStdDev;
        }
    }

    @Override
    public void init() {
        motors = new ArrayList<>(hardwareMap.dcMotor.entrySet());
        motors.sort(MotorVariabilityTester.motorComparator);
        if (motors.isEmpty()) {
            requestOpModeStop();
        }
    }

    @SuppressLint("DefaultLocale")
    @Override
    public void init_loop() {
        updateGamepad();
        for (int i = 0; i < motors.size(); i++) {
            Map.Entry<String, DcMotor> motorEntry = motors.get(i);
            telemetry.addLine(String.format("%s%d: %s", i == selectedIndex ? "> " : "", motorEntry.getValue().getPortNumber(), motorEntry.getKey()));
        }
        if (currentGamepad.dpad_up && !previousGamepad.dpad_up) {
            selectedIndex = Math.floorMod(selectedIndex - 1, motors.size());
        } else if (currentGamepad.dpad_down && !previousGamepad.dpad_down) {
            selectedIndex = Math.floorMod(selectedIndex + 1, motors.size());
        }
    }

    @Override
    public void start() {
        Map.Entry<String, DcMotor> selectedMotorEntry = motors.get(selectedIndex);
        selectedMotorName = selectedMotorEntry.getKey();
        selectedMotor = (DcMotorEx) selectedMotorEntry.getValue();
        selectedMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRuntime.reset();
    }

    @Override
    public void loop() {
        if (!complete) {
            telemetry.addLine(selectedMotorName);
            telemetry.addData("Motor Power", motorPower);
            telemetry.addData("Motor Runtime", motorRuntime.seconds());

            selectedMotor.setPower(motorPower);
            if (motorSampleIndex == SAMPLES) {
                if (motorPower == 1) {
                    selectedMotor.setPower(0);
                    complete = true;
                } else {
                    double velocityMean = mean(motorVelocitySamples);
                    double velocityStdDev = standardDeviation(motorVelocitySamples);
                    double currentMean = mean(motorCurrentSamples);
                    double currentStdDev = standardDeviation(motorCurrentSamples);
                    motorPowerResults.put(motorPower, new MotorPowerResults(velocityMean, velocityStdDev, currentMean, currentStdDev));

                    motorPower = Math.min(motorPower * 2, 1);
                    motorRuntime.reset();
                    motorSampleIndex = 0;
                    Arrays.fill(motorVelocitySamples, 0.0);
                    Arrays.fill(motorCurrentSamples, 0.0);
                }
            } else if (motorRuntime.seconds() > 1) {
                motorVelocitySamples[motorSampleIndex] = selectedMotor.getVelocity();
                motorCurrentSamples[motorSampleIndex] = selectedMotor.getCurrent(CurrentUnit.AMPS);
                motorSampleIndex++;
            }
        } else {
            if (motors.isEmpty()) {
                telemetry.addLine("No motors found");
            } else {
                if (!savedResults) {
                    savedResults = true;
                    saveResults();
                }
                telemetry.addData("Results saved", System.getProperty("user.dir"));
//                telemetry.addLine("Results");
//                for (Map.Entry<Double, MotorPowerResults> entry : motorPowerResults.entrySet()) {
//                    MotorPowerResults motorPowerResult = entry.getValue();
//                    telemetry.addLine(entry.getKey().toString() + ": ")
//                            .addData("velMean", motorPowerResult.velocityMean)
//                            .addData("velStdDev", motorPowerResult.velocityStdDev)
//                            .addData("curMean", motorPowerResult.currentMean)
//                            .addData("curStdDev", motorPowerResult.currentStdDev);
//                }
            }
            telemetry.update();
        }
    }

    @SuppressLint("DefaultLocale")
    private void saveResults() {
        try (FileWriter writer = new FileWriter(String.format(Environment.getDataDirectory() + "/motor_results_%tF_%<tT\".csv", java.util.Calendar.getInstance()))) {
            writer.write("Power,VelMean,VelStdDev,CurMean,CurStdDev\n");
            for (Map.Entry<Double, MotorPowerResults> entry : motorPowerResults.entrySet()) {
                MotorPowerResults motorPowerResult = entry.getValue();
                writer.write(String.format("%f,%f,%f,%f,%f\n",
                        entry.getKey(),
                        motorPowerResult.velocityMean,
                        motorPowerResult.velocityStdDev,
                        motorPowerResult.currentMean,
                        motorPowerResult.currentStdDev));
            }
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    private void updateGamepad() {
        previousGamepad.copy(currentGamepad);
        currentGamepad.copy(gamepad1);
    }

    private static double sum(double[] numbers) {
        double sum = 0;
        for (double number : numbers) {
            sum += number;
        }
        return sum;
    }

    private static double mean(double[] numbers) {
        return sum(numbers) / numbers.length;
    }

    private static double standardDeviation(double[] numbers) {
        double mean = mean(numbers);
        double sum = 0;
        for (double number : numbers) {
            double difference = number - mean;
            sum += difference * difference;
        }
        return Math.sqrt(sum / numbers.length);
    }
}
