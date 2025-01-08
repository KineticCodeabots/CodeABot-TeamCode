package org.firstinspires.ftc.teamcode;


/**
 * A PID controller with anti-windup
 */
public class PIDAW extends PID {

    public PIDAW(double Kp, double Ki, double Kd) {
        super(Kp, Ki, Kd);
    }

    public double update(double reference, double state) {
        if (lastReference != reference) {
            integralSum = 0;
        }
        return super.update(reference, state);
    }
}
