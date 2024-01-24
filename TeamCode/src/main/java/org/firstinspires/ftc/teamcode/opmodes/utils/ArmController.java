package org.firstinspires.ftc.teamcode.opmodes.utils;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

public class ArmController {
    private DcMotorEx motor = null;
    private final double MAX_DOWN_VELOCITY = -500;
    private final double MAX_POWER = 0.5; // For safety reasons
    private final int VIRTUAL_STOP_POSITION = 80;
    private final int VIRTUAL_STOP_THRESSHOLD = 5;

    private final PIDF velocityPIDF = new PIDF(0.0001, 0.0000, 0.000, 0.00045);
    private final PID positionPID = new PID(0.01, 0.01
            , 0);

    private boolean onVirtualStop = false;
    private boolean previousZeroing = false;
    private boolean zeroed = false;

    public double slowDown = 0;

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
        double powerCommand = power;
        if (zeroed && !zeroing && virtualStop) {
            if (motor.getCurrentPosition() < VIRTUAL_STOP_POSITION) {
                powerCommand = positionPID.update(VIRTUAL_STOP_POSITION, motor.getCurrentPosition()) + Math.max(powerCommand, 0);
            } else if (Math.abs(motor.getCurrentPosition() - VIRTUAL_STOP_POSITION) < VIRTUAL_STOP_THRESSHOLD) {
                powerCommand = Math.max(powerCommand, 0);
            }
        } else {
            if (powerCommand == 0) {
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

    void setLimitedPower(double power) {
        motor.setPower(Range.clip(power, -MAX_POWER, MAX_POWER));
    }
}
