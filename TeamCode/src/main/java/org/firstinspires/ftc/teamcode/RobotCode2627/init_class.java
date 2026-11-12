package org.firstinspires.ftc.teamcode.RobotCode2627;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class init_class {
    DcMotorEx frontLeft, backLeft, frontRight, backRight, intake, FlywheelLeft, FlywheelRight, Turret;
    Servo GateServo;

    public void init(HardwareMap hwMap) { //initializes Motors to use in a later stage of the code.
        frontLeft = hwMap.get(DcMotorEx.class, "frontLeft");
        backLeft = hwMap.get(DcMotorEx.class, "backLeft");
        frontRight = hwMap.get(DcMotorEx.class, "frontRight");
        backRight = hwMap.get(DcMotorEx.class, "backRight");
        intake = hwMap.get(DcMotorEx.class, "intakeMotor");
        FlywheelLeft = hwMap.get(DcMotorEx.class,"FlywheelLeft");
        FlywheelRight = hwMap.get(DcMotorEx.class,"FlywheelRight");
        Turret = hwMap.get(DcMotorEx.class,"Turret");
        GateServo = hwMap.get(Servo.class, "GateServo");

        frontLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        intake.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        FlywheelRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        FlywheelLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        Turret.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        frontRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);  //Sets Motors RunMode to using a encoder for high control over the robots driving and sets the motors brake mode to BRAKE
        backRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        Turret.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        FlywheelLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        FlywheelRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);


        frontLeft.setDirection(DcMotorEx.Direction.REVERSE); // setting motor direction
        backLeft.setDirection(DcMotorEx.Direction.REVERSE);
        FlywheelRight.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    public void drive(double forward, double strafe, double turn, double intakeVelocity, double flywheelVelocityR,double flywheelVelocityL, double GateServoPos){ //Variables to set actuators power and speed
        double frontLeftPower = forward + strafe + turn;
        double backLeftPower = forward - strafe + turn;  // basic calculations for Mecanum Drivetrain
        double frontRightPower = forward - strafe - turn;
        double backRightPower = forward + strafe - turn;

        double maxPower = 1.0; //setting maximum power/speed of drivetrain
        double maxSpeed = 1.0;

        maxPower = Math.max(maxPower, Math.abs(frontLeftPower));
        maxPower = Math.max(maxPower, Math.abs(backLeftPower)); //making sure the Power set to the motors never goes over 1
        maxPower = Math.max(maxPower, Math.abs(frontRightPower));
        maxPower = Math.max(maxPower, Math.abs(backRightPower));

        frontLeft.setPower(maxSpeed * frontLeftPower / maxPower);
        backLeft.setPower(maxSpeed * backLeftPower / maxPower);
        frontRight.setPower(maxSpeed * frontRightPower / maxPower);
        backRight.setPower(maxSpeed * backRightPower / maxPower);
        intake.setVelocity(intakeVelocity);    // setting the Speed/Power of the actuators with the variables created earlier
        FlywheelLeft.setVelocity(flywheelVelocityL);
        FlywheelRight.setVelocity(flywheelVelocityR);
        GateServo.setPosition(GateServoPos);
    }
}
