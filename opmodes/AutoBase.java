package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public abstract class AutoBase extends LinearOpMode {
    private Robot robot = new Robot(this);

    @Override
    public void runOpMode() {
        robot.setupAuto();

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }
}
