package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.CodeabotCommon;
import org.firstinspires.ftc.teamcode.vision.TeamPropDetermination;

public abstract class BaseAuto extends LinearOpMode {
    // Constants
    private static final double DRIVE_SPEED = 0.7;
    private static final double TURN_SPEED = 0.2;

    private double currentHeadaing = 0;

    // Match State
    protected CodeabotCommon.Alliance alliance = null;
    protected CodeabotCommon.StartingLocation startingLocation = null;

    private AutoRobot robot = null; // TODO: check that correct alliance is passed in

    public BaseAuto(CodeabotCommon.Alliance alliance, CodeabotCommon.StartingLocation startingLocation) {
        this.alliance = alliance;
        this.startingLocation = startingLocation;
        this.robot = new AutoRobot(this, alliance);
    }

    @Override
    public void runOpMode() {
        robot.init();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // TODO: better way of detecting that teampropdetermination is available

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
        // TODO: log team prop position
        telemetry.log().add("Team Prop Position: " + teamPropPosition.toString());

        robot.start();
        robot.setHandPosition(0.4);
        robot.setGripperState(false); // TODO: flip gripper state from default open to default closed

        // Starts 6.5 inches from truss

        // TODO: implement spike mark scoring
        if (startingLocation == CodeabotCommon.StartingLocation.AUDIENCE) {
            robot.driveStraight(DRIVE_SPEED, 32, 0);
            robot.setGripperState(true);
            allianceTurnToHeading(TURN_SPEED, -90);
            robot.driveStraight(DRIVE_SPEED, 70, currentHeadaing);
            allianceTurnToHeading(TURN_SPEED, 0);
            robot.driveStraight(DRIVE_SPEED, 27, currentHeadaing);
            allianceTurnToHeading(TURN_SPEED, -90);
            robot.driveStraight(DRIVE_SPEED, 20, currentHeadaing);
        } else {
            robot.driveStraight(DRIVE_SPEED, 7, 0);
            allianceTurnToHeading(TURN_SPEED, -90);
            robot.driveStraight(DRIVE_SPEED, 30, currentHeadaing);
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
}
