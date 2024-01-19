package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDF extends PID {
    private double Kv;

    public PIDF(double Kp, double Ki, double Kd, double Kv) {
        super(Kp, Ki, Kd);
        this.Kv = Kv;
    }

    public double update(double reference, double state) {
        return super.update(reference, state) + (Kv * reference);
    }
}
