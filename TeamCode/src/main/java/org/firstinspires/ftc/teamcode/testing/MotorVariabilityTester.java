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
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.Date;
import java.util.Locale;
import java.util.Map;

@TeleOp(name = "Motor Variability Tester", group = "Testing")
public class MotorVariabilityTester extends OpMode {
    private static final Comparator<Map.Entry<String, DcMotor>> motorComparator = Comparator.comparing(
            entry -> entry.getValue().getPortNumber()
    );
    protected ArrayList<Map.Entry<String, DcMotor>> motors = null;
    private int selectedIndex = 0;

    private final Gamepad currentGamepad = new Gamepad();
    private final Gamepad previousGamepad = new Gamepad();

    protected String selectedMotorName = null;
    protected DcMotorEx selectedMotor = null;

    protected final ElapsedTime motorRuntime = new ElapsedTime();
    protected double motorPower = 0.5;

    protected static final int SAMPLES = 5000;
    protected final double[] motorVelocitySamples = new double[SAMPLES];
    protected final double[] motorCurrentSamples = new double[SAMPLES];
    protected int motorSampleIndex = 0;

    private MotorPowerResults results = null;

    protected boolean complete = false;
    protected boolean savedResults = false;

    public static class MotorPowerResults {
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
            runtime_loop();
        } else {
            complete_loop();
        }
    }

    protected void runtime_loop() {
        telemetry.addLine(selectedMotorName);
        telemetry.addData("Motor Power", motorPower);
        telemetry.addData("Motor Runtime", motorRuntime.seconds());

        selectedMotor.setPower(motorPower);
        if (motorSampleIndex == SAMPLES) {
            double velocityMean = mean(motorVelocitySamples);
            double velocityStdDev = standardDeviation(motorVelocitySamples);
            double currentMean = mean(motorCurrentSamples);
            double currentStdDev = standardDeviation(motorCurrentSamples);
            results = new MotorPowerResults(velocityMean, velocityStdDev, currentMean, currentStdDev);

            selectedMotor.setPower(0);
            complete = true;
            motorRuntime.reset();
            motorSampleIndex = 0;
            Arrays.fill(motorVelocitySamples, 0.0);
            Arrays.fill(motorCurrentSamples, 0.0);
        } else if (motorRuntime.seconds() > 1) {
            double velocity = selectedMotor.getVelocity();
            double current = selectedMotor.getCurrent(CurrentUnit.AMPS);
            telemetry.addData("Velocity", velocity);
            telemetry.addData("Current", current);
            motorVelocitySamples[motorSampleIndex] = velocity;
            motorCurrentSamples[motorSampleIndex] = current;
            motorSampleIndex++;
        }
    }

    protected void complete_loop() {
        if (motors.isEmpty()) {
            telemetry.addLine("No motors found");
        } else {
            if (!savedResults) {
                savedResults = true;
                try {
                    String resultsFilepath = saveResults();
                    telemetry.log().add("Results saved: %s", resultsFilepath);
                } catch (IOException e) {
                    telemetry.log().add("An error occurred while saving results: " + e.getMessage() + ", Stack Trace: " + Arrays.toString(e.getStackTrace()));
                }
            }
            telemetry.addLine("Results:");
            telemetry.addData("Velocity Mean", results.velocityMean);
            telemetry.addData("Velocity StdDev", results.velocityStdDev);
            telemetry.addData("Current Mean", results.currentMean);
            telemetry.addData("Current StdDev", results.currentStdDev);
        }
    }

    @SuppressLint("DefaultLocale")
    protected String saveResults() throws IOException {
        Date now = new Date();
        SimpleDateFormat formatter = new SimpleDateFormat("yyyyMMdd_HHmm", Locale.US);
        String formattedDateTime = formatter.format(now);
        String filePath = Environment.getExternalStorageDirectory() + String.format("/motor_results_%s.csv", formattedDateTime);
        try (FileWriter writer = new FileWriter(filePath)) {
            writer.write(String.format("%f\n%f\n%f\n%f\n%f\n", motorPower, results.velocityMean, results.velocityStdDev, results.currentMean, results.currentStdDev));
        }
        return filePath;
    }

    protected void updateGamepad() {
        previousGamepad.copy(currentGamepad);
        currentGamepad.copy(gamepad1);
    }

    protected static double sum(double[] numbers) {
        double sum = 0;
        for (double number : numbers) {
            sum += number;
        }
        return sum;
    }

    protected static double mean(double[] numbers) {
        return sum(numbers) / numbers.length;
    }

    protected static double standardDeviation(double[] numbers) {
        double mean = mean(numbers);
        double sum = 0;
        for (double number : numbers) {
            double difference = Math.abs(number - mean);
            sum += difference;
        }
        return sum / numbers.length;
    }
}
