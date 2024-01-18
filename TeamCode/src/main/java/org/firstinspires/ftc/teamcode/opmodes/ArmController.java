package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.PID;
import org.firstinspires.ftc.teamcode.PIDF;

public class ArmController {
    private DcMotorEx motor = null;
    private final int MAX_VELOCITY = 600;
    private final double MAX_POWER = 0.3; // For safety reasons
    private final int VIRTUAL_STOP_POSITION = 90;
    private final int VIRTUAL_STOP_THRESSHOLD = 10;

    private final ArmPIDF velocityPID = new ArmPIDF(0.0005, 0.0000, 0.0001, 0.0005);
    private final PID positionPID = new PID(0.05, 0.001, 0);

    private int previousPosition = 0;
    private boolean previousZeroing = false;

    static class ArmPIDF extends PIDF {
        public ArmPIDF(double Kp, double Ki, double Kd, double Kv) {
            super(Kp, Ki, Kd, Kv);
        }

        @Override
        public double update(double reference, double state) {
            double powerCommand = super.update(reference, state);
            return (reference < 0) ? Math.min(powerCommand, 0) : Math.max(powerCommand, 0);
        }
    }

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
        if (motor.getCurrentPosition() > VIRTUAL_STOP_POSITION + VIRTUAL_STOP_THRESSHOLD || power > 0 || zeroing || pushDown) {
            if (power == 0 && !pushDown)
                motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            else motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

            if (power == 0) velocityPID.reset();

            if (pushDown && power == 0) {
                if (motor.getCurrentPosition() > 20) setPower(-0.1);
                else setPower(0);
            } else setPower(velocityPID.update(power * MAX_VELOCITY, motor.getVelocity()));
            positionPID.reset();
        } else {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            if (motor.getCurrentPosition() < VIRTUAL_STOP_POSITION)
                setPower(positionPID.update(VIRTUAL_STOP_POSITION, motor.getCurrentPosition()));
            else {
                setPower(0);
            }
        }
        if (previousZeroing && !zeroing) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        previousPosition = motor.getCurrentPosition();
        previousZeroing = zeroing;
    }

    void setPower(double power) {
        motor.setPower(Range.clip(power, -MAX_POWER, MAX_POWER));
    }
}
