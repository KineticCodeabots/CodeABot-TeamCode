package org.firstinspires.ftc.teamcode.opmodes.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.CodeabotCommon;
import org.firstinspires.ftc.teamcode.opmodes.AutoBase;

@Autonomous(name="Blue/Audience", group="blue")
public class BlueAudienceAuto extends AutoBase {
    private final CodeabotCommon.Alliance alliance = CodeabotCommon.Alliance.BLUE;
    private final CodeabotCommon.StartingLocation startingLocation = CodeabotCommon.StartingLocation.AUDIENCE;
}
