package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.opmodes.utils.PID;
import org.firstinspires.ftc.teamcode.opmodes.utils.PIDF;

public class ArmController {
    private DcMotorEx motor = null;
    private final int TARGET_VELOCITY = 600;
    private final double MAX_POWER = 0.3; // For safety reasons
    private final int VIRTUAL_STOP_POSITION = 80;
    private final int VIRTUAL_STOP_THRESSHOLD = 5;
    private final int VIRTUAL_STOP_SLOWDOWN = 100;
    private final int VIRTUAL_STOP_SLOWDOWN_VELOCITY = 250;

    private final PIDF velocityPIDF = new PIDF(0.0005, 0.0000, 0.000, 0.00045);
    private final PID positionPID = new PID(0.05, 0.002, 0);

    private boolean onVirtualStop = false;
    private boolean previousZeroing = false;

    public ArmController(DcMotorEx motor) {
        this.motor = motor;
    }

    public void init() {
        motor.setDirection(DcMotor.Direction.REVERSE);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void update(double power, boolean pushDown, boolean zeroing) {
        if ((motor.getCurrentPosition() > VIRTUAL_STOP_POSITION + 10 && !onVirtualStop) || power > 0 || pushDown) {
            onVirtualStop = false;
            boolean inSlowdown = motor.getCurrentPosition() < VIRTUAL_STOP_POSITION + VIRTUAL_STOP_SLOWDOWN;
            if (pushDown || (power != 0 && !inSlowdown)) {
                motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            } else {
                motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }

            if (pushDown && motor.getCurrentPosition() < 20) {
                setPower(0);
            } else {
                double reference = power * TARGET_VELOCITY;
                if (inSlowdown) reference = Math.max(reference, -VIRTUAL_STOP_SLOWDOWN_VELOCITY);
                if (pushDown) reference = -100;

                double powerCommand = velocityPIDF.update(reference, motor.getVelocity());
                powerCommand = (reference < 0) ? Math.min(powerCommand, 0) : Math.max(powerCommand, 0);

                setPower(powerCommand);
            }

            positionPID.reset();
        } else {
            onVirtualStop = true;
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            if (Math.abs(motor.getCurrentPosition() - VIRTUAL_STOP_POSITION) > VIRTUAL_STOP_THRESSHOLD)
                setPower(positionPID.update(VIRTUAL_STOP_POSITION, motor.getCurrentPosition()));
            else {
                setPower(0);
            }
        }
        if (previousZeroing && !zeroing) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        previousZeroing = zeroing;
    }

    void setPower(double power) {
        motor.setPower(Range.clip(power, -MAX_POWER, MAX_POWER));
    }
}
