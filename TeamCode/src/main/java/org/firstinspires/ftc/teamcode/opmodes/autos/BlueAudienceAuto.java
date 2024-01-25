package org.firstinspires.ftc.teamcode.opmodes.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.CodeabotCommon;
import org.firstinspires.ftc.teamcode.opmodes.Auto;

@Autonomous(name = "Blue/Audience", group = "blue")
public class BlueAudienceAuto extends Auto {
    public BlueAudienceAuto() {
        super(CodeabotCommon.Alliance.BLUE, CodeabotCommon.StartingLocation.AUDIENCE);
    }
}
