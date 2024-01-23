package org.firstinspires.ftc.teamcode.opmodes.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.CodeabotCommon;
import org.firstinspires.ftc.teamcode.opmodes.BaseAuto;

@Autonomous(name = "Red/Backstage", group = "red")
public class RedBackstageAuto extends BaseAuto {
    public RedBackstageAuto() {
        super();
        alliance = CodeabotCommon.Alliance.RED;
        startingLocation = CodeabotCommon.StartingLocation.BACKSTAGE;
    }
}
