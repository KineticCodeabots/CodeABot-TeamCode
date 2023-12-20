package org.firstinspires.ftc.teamcode.opmodes.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.CodeabotCommon;
import org.firstinspires.ftc.teamcode.opmodes.AutoBase;

@Autonomous(name = "Red/Audience", group = "red")
public class RedAudienceAuto extends AutoBase {
    private final CodeabotCommon.Alliance alliance = CodeabotCommon.Alliance.RED;
    private final CodeabotCommon.StartingLocation startingLocation = CodeabotCommon.StartingLocation.AUDIENCE;
}
