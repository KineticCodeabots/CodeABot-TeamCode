package org.firstinspires.ftc.teamcode.archive;

import com.qualcomm.hardware.motors.RevRoboticsCoreHexMotor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Weird_3wheel_robot", group = "TeleOp")


public class Weird_3wheel_robot extends OpMode {
    @Override
    public void init() {
        RevRoboticsCoreHexMotor LeftDriveMotor = (RevRoboticsCoreHexMotor) hardwareMap.dcMotor.get("LeftDriveMotor");
        RevRoboticsCoreHexMotor RightDriveMotor = (RevRoboticsCoreHexMotor) hardwareMap.dcMotor.get("RightDriveMotor");
        RevRoboticsCoreHexMotor TurningMotor = (RevRoboticsCoreHexMotor) hardwareMap.dcMotor.get("TurningMotor");

    }

    @Override
    public void loop() {

    }
}
