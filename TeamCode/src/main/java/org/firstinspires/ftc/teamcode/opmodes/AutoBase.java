package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.CodeabotCommon;

public abstract class AutoBase extends LinearOpMode {
    private static final double DRIVE_SPEED = 0.7;
    private static final double TURN_SPEED = 0.2;

    private final CodeabotCommon.Alliance alliance = CodeabotCommon.Alliance.BLUE;
    private final CodeabotCommon.StartingLocation startingLocation = CodeabotCommon.StartingLocation.AUDIENCE;

    final private AutoRobot robot = new AutoRobot(this);

    @Override
    public void runOpMode() {
        robot.init();
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        robot.start();
        robot.driveStraight(DRIVE_SPEED, 33, 0);
        alianceTurnToHeading(TURN_SPEED, -90);
        if (startingLocation == CodeabotCommon.StartingLocation.AUDIENCE) {
            robot.driveStraight(DRIVE_SPEED, 80, -90);
        }
        alianceTurnToHeading(TURN_SPEED, -180);
        robot.driveStraight(DRIVE_SPEED, 20, -180);
    }

    private void alianceTurnToHeading(double maxTurnSpeed, double heading) {
        if (alliance == CodeabotCommon.Alliance.BLUE) {
            robot.turnToHeading(maxTurnSpeed, heading);
        } else {
            robot.turnToHeading(maxTurnSpeed, -heading);
        }
    }
}
