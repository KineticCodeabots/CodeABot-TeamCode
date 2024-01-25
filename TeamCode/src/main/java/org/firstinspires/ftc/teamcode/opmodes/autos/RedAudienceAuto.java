package org.firstinspires.ftc.teamcode.opmodes.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.CodeabotCommon;
import org.firstinspires.ftc.teamcode.opmodes.Auto;

@Autonomous(name = "Red/Audience", group = "red")
public class RedAudienceAuto extends Auto {
    public RedAudienceAuto() {
        super(CodeabotCommon.Alliance.RED, CodeabotCommon.StartingLocation.AUDIENCE);
    }
}
