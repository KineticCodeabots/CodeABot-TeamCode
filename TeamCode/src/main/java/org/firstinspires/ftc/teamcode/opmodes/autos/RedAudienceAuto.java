package org.firstinspires.ftc.teamcode.opmodes.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.CodeabotCommon;
import org.firstinspires.ftc.teamcode.opmodes.BaseAuto;

@Autonomous(name = "Red/Audience", group = "red")
public class RedAudienceAuto extends BaseAuto {
    public RedAudienceAuto() {
        super();
        alliance = CodeabotCommon.Alliance.RED;
        startingLocation = CodeabotCommon.StartingLocation.AUDIENCE;
    }

}
