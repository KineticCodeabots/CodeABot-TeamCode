package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDF extends PID {
    private double Kv;

    ElapsedTime timer = new ElapsedTime();

    public PIDF(double Kp, double Ki, double Kd, double Kv) {
        super(Kp, Ki, Kd);
        this.Kv = Kv;
    }

    public double update(double reference, double state) {
        double error = reference - state;

        double derivative = (error - lastError) / timer.seconds();
        integralSum = integralSum + (error * timer.seconds());

        return (Kp * error) + (Ki * integralSum) + (Kd * derivative) + (Kv * reference);
    }
}
