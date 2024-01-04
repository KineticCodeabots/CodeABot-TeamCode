package org.firstinspires.ftc.teamcode.opmodes;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.CodeabotCommon;
import org.firstinspires.ftc.teamcode.vision.TeamPropDetermination;
import org.firstinspires.ftc.vision.VisionPortal;

public abstract class AutoBase extends LinearOpMode {
    private static final double DRIVE_SPEED = 0.7;
    private static final double TURN_SPEED = 0.2;

    private final CodeabotCommon.Alliance alliance = CodeabotCommon.Alliance.BLUE;
    private final CodeabotCommon.StartingLocation startingLocation = CodeabotCommon.StartingLocation.AUDIENCE;

    final private AutoRobot robot = new AutoRobot(this);

    @Override
    public void runOpMode() {
        robot.init();
        TeamPropDetermination teamPropDeterminationProcessor = new TeamPropDetermination(telemetry, alliance);

        VisionPortal.Builder visionPortalBuilder = new VisionPortal.Builder();
        visionPortalBuilder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(teamPropDeterminationProcessor)
                .setCameraResolution(new Size(640, 480))
                .enableLiveView(true)
                .setAutoStopLiveView(true)
                .build();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        while (opModeInInit()) {
            teamPropDeterminationProcessor.addTelemetry();
            telemetry.update();
        }
        waitForStart();

        TeamPropDetermination.Position teamPropPosition = teamPropDeterminationProcessor.getPosition();

        robot.start();
        robot.resetServos();

        robot.driveStraight(DRIVE_SPEED, 33, 0);
        // TODO: implement spike mark scoring
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
