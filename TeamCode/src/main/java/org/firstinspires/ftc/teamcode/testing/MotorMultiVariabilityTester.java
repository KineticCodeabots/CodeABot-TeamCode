package org.firstinspires.ftc.teamcode.testing;

import android.annotation.SuppressLint;
import android.os.Environment;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.Arrays;
import java.util.Date;
import java.util.LinkedHashMap;
import java.util.Locale;
import java.util.Map;

@TeleOp(name = "Motor Multi Variability Tester", group = "Testing")
public class MotorMultiVariabilityTester extends MotorVariabilityTester {
    private final Map<Double, MotorPowerResults> motorPowerResults = new LinkedHashMap<>();

    @Override
    protected void runtime_loop() {
        telemetry.addLine(selectedMotorName);
        telemetry.addData("Motor Power", motorPower);
        telemetry.addData("Motor Runtime", motorRuntime.seconds());

        selectedMotor.setPower(motorPower);
        if (motorSampleIndex == SAMPLES) {
            if (motorPower == 1) {
                selectedMotor.setPower(0);
                complete = true;
            }

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
        } else if (motorRuntime.seconds() > 1) {
            motorVelocitySamples[motorSampleIndex] = selectedMotor.getVelocity();
            motorCurrentSamples[motorSampleIndex] = selectedMotor.getCurrent(CurrentUnit.AMPS);
            motorSampleIndex++;
        }
    }

    @Override
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
            telemetry.addLine("Results\n");
            for (Map.Entry<Double, MotorPowerResults> entry : motorPowerResults.entrySet()) {
                MotorPowerResults motorPowerResult = entry.getValue();
                telemetry.addData(entry.getKey().toString(), motorPowerResult.velocityStdDev);
            }
        }
    }

    @SuppressLint("DefaultLocale")
    @Override
    protected String saveResults() throws IOException {
        Date now = new Date();
        SimpleDateFormat formatter = new SimpleDateFormat("yyyyMMdd_HHmm", Locale.US);
        String formattedDateTime = formatter.format(now);
        String filePath = Environment.getExternalStorageDirectory() + String.format("/motor_results_%s.csv", formattedDateTime);
        try (FileWriter writer = new FileWriter(filePath)) {
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
        }
        return filePath;
    }
}
