package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.CodeabotCommon;
import org.firstinspires.ftc.teamcode.vision.TeamPropDetermination;

public abstract class Auto extends LinearOpMode {
    // Constants
    private static final double DRIVE_SPEED = 0.7;
    private static final double TURN_SPEED = 0.2;

    private double currentHeadaing = 0;

    // Match State
    protected CodeabotCommon.Alliance alliance = null;
    protected CodeabotCommon.StartingLocation startingLocation = null;

    private AutoRobot robot = null;

    public Auto(CodeabotCommon.Alliance alliance, CodeabotCommon.StartingLocation startingLocation) {
        this.alliance = alliance;
        this.startingLocation = startingLocation;
        this.robot = new AutoRobot(this, alliance);
    }

    @Override
    public void runOpMode() {
        robot.init();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        while (opModeInInit()) {
            // TeamPropDetermination telemetry
            telemetry.addData("Alliance", alliance.toString());
            telemetry.addData("Starting Location", startingLocation.toString());
            robot.addTeamPropTelemetry();
            telemetry.update();
        }

        waitForStart();

        // Get determined team prop position
        TeamPropDetermination.Position teamPropPosition = robot.getTeamPropPosition();
        robot.setTeamPropDeterminationEnabled(false);
        telemetry.log().add("Team Prop Position: " + teamPropPosition.toString());

        robot.start();
        robot.setHandPosition(0.4);
        robot.setGripperState(true);

        // Starts 6.5 inches from truss

        if (startingLocation == CodeabotCommon.StartingLocation.AUDIENCE) {
            driveStraight(32);
            robot.setGripperState(false);
            allianceTurnToHeading(TURN_SPEED, -90);
            driveStraight(70);
            allianceTurnToHeading(TURN_SPEED, 0);
            driveStraight(27);
            allianceTurnToHeading(TURN_SPEED, -90);
            driveStraight(20);
        } else {
            driveStraight(7);
            allianceTurnToHeading(TURN_SPEED, -90);
            driveStraight(30);
        }
    }

    private void allianceTurnToHeading(double maxTurnSpeed, double heading) {
        if (alliance == CodeabotCommon.Alliance.BLUE) {
            robot.turnToHeading(maxTurnSpeed, heading);
            currentHeadaing = heading;
        } else {
            robot.turnToHeading(maxTurnSpeed, -heading);
            currentHeadaing = -heading;
        }
    }

    private void driveStraight(double distance) {
        robot.driveStraight(DRIVE_SPEED, distance, currentHeadaing);
    }
}
