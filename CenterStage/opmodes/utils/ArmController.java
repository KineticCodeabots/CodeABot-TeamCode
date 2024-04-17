package org.firstinspires.ftc.teamcode.opmodes.utils;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

public class ArmController {
    private DcMotorEx motor = null;
    private final double MAX_DOWN_VELOCITY = -500;
    private final double MAX_POWER = 0.5; // For safety reasons
    private final int VIRTUAL_STOP_UPPER_POSITION = 85;
    private final int VIRTUAL_STOP_LOWER_POSITION = 75;

    private final PIDF velocityPIDF = new PIDF(0.0005, 0.0000, 0.000, 0.00045);
    private final PID positionPID = new PID(0.003, 0.005, 0.001);

    private boolean previousZeroing = false;
    private boolean zeroed = false;

    public int encoderOffset = 0;

    public ArmController(DcMotorEx motor) {
        this.motor = motor;
    }

    public void init() {
        motor.setDirection(DcMotor.Direction.REVERSE);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /**
     * Updates the state of the arm
     */
    public void updateArmState(double power, boolean virtualStop, boolean zeroing, boolean limitPower) {
        int encoderPosition = getCurrentPosition();

        double powerCommand = 0;
        if (zeroed && !zeroing && virtualStop && encoderPosition < VIRTUAL_STOP_UPPER_POSITION) {
            // Only allow upward power
            powerCommand = Math.max(power, 0);

            if (encoderPosition < VIRTUAL_STOP_LOWER_POSITION) {
                powerCommand += positionPID.update(VIRTUAL_STOP_LOWER_POSITION, encoderPosition);
            } else {
                motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }
        } else {
            // Reset position PID if not in virtual stop range
            positionPID.reset();
            powerCommand = power;

            if (!virtualStop) {
                // When bypassing virtual stop allow motor to float and turn off when close to ground.
                if (limitPower) motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

                if (zeroed && power <= 0 && motor.getCurrentPosition() < 10)
                    powerCommand = 0;
            } else if (powerCommand == 0) {
                // Brake, and run velocity PID if power is zero
                motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                powerCommand = velocityPIDF.update(powerCommand, motor.getVelocity());
            } else if (motor.getVelocity() < MAX_DOWN_VELOCITY) {
                // If moving too fast downwards turn off power and float.
                motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                powerCommand = 0;
            }
        }

        if (previousZeroing && !zeroing) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            encoderOffset = 0;
            zeroed = true;
        }

        previousZeroing = zeroing;

        if (limitPower) {
            setLimitedPower(powerCommand);
        } else {
            motor.setPower(powerCommand);
        }
    }

    /**
     * Encoder position plus offset
     */
    public int getCurrentPosition() {
        return motor.getCurrentPosition() + encoderOffset;
    }

    /**
     * Limits power sent to motors
     */
    void setLimitedPower(double power) {
        motor.setPower(Range.clip(power, -MAX_POWER, MAX_POWER));
    }
}
