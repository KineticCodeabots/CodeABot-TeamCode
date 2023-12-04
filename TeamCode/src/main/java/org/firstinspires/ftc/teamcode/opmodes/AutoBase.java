package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.CodeabotCommon;

public abstract class AutoBase extends LinearOpMode {
    private final CodeabotCommon.Alliance alliance = null;
    private final CodeabotCommon.StartingLocation startingLocation = null;

    private Robot robot = new Robot(this);

    @Override
    public void runOpMode() {
        robot.setupAuto();

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }
}
