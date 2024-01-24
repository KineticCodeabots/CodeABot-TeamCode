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

    public void updateArmState(double power, boolean virtualStop, boolean zeroing) {
        int encoderPosition = getCurrentPosition();
        double powerCommand = 0;
        if (zeroed && !zeroing && virtualStop && encoderPosition < VIRTUAL_STOP_UPPER_POSITION) {
            powerCommand = Math.max(power, 0);
            if (encoderPosition < VIRTUAL_STOP_LOWER_POSITION) {
                powerCommand = positionPID.update(VIRTUAL_STOP_LOWER_POSITION, encoderPosition) + powerCommand;
            } else {
                motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }
        } else {
            powerCommand = power;
            if (!virtualStop) {
                motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                if (zeroed && power <= 0 && motor.getCurrentPosition() < 10)
                    powerCommand = 0;
            } else if (powerCommand == 0) {
                motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                powerCommand = velocityPIDF.update(powerCommand, motor.getVelocity());
            } else if (motor.getVelocity() < MAX_DOWN_VELOCITY) {
                motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                powerCommand = 0;
            }
        }

        if (previousZeroing && !zeroing) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            zeroed = true;
        }
        previousZeroing = zeroing;
        setLimitedPower(powerCommand);
    }

    public int getCurrentPosition() {
        return motor.getCurrentPosition() + encoderOffset;
    }

    void setLimitedPower(double power) {
        motor.setPower(Range.clip(power, -MAX_POWER, MAX_POWER));
    }
}
