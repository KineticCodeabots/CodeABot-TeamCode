package org.firstinspires.ftc.teamcode.opmodes.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.CodeabotCommon;
import org.firstinspires.ftc.teamcode.opmodes.BaseAuto;

@Autonomous(name = "Blue/Backstage", group = "blue")
public class BlueBackstageAuto extends BaseAuto {
    public BlueBackstageAuto() {
        super(CodeabotCommon.Alliance.BLUE, CodeabotCommon.StartingLocation.BACKSTAGE);
    }
}
