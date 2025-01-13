package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.Range;

public class ArmAGPID extends PID {
    public ArmAGPID(double Kp, double Ki, double Kd) {
        super(Kp, Ki, Kd);
    }

    @Override
    public double update(double reference, double state) {
        double error = reference - state;
        double derivative = (error - lastError) / timer.seconds();
        double integralLimit = 1 / Ki;
        integralSum = Range.clip(integralSum + (error * timer.seconds()), -integralLimit, integralLimit); // anti-windup
        lastReference = reference;
        timer.reset();
        return (Kp * error) + (Ki * integralSum) + (Kd * derivative);
    }
}
