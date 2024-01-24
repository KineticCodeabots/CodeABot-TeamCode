package org.firstinspires.ftc.teamcode.opmodes.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.CodeabotCommon;
import org.firstinspires.ftc.teamcode.opmodes.BaseAuto;

@Autonomous(name = "Blue/Audience", group = "blue")
public class BlueAudienceAuto extends BaseAuto {
    public BlueAudienceAuto() {
        super(CodeabotCommon.Alliance.BLUE, CodeabotCommon.StartingLocation.AUDIENCE);
    }
}
