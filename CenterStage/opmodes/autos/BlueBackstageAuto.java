package org.firstinspires.ftc.teamcode.opmodes.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.CodeabotCommon;
import org.firstinspires.ftc.teamcode.opmodes.Auto;

@Autonomous(name = "Blue/Backstage", group = "blue")
public class BlueBackstageAuto extends Auto {
    public BlueBackstageAuto() {
        super(CodeabotCommon.Alliance.BLUE, CodeabotCommon.StartingLocation.BACKSTAGE);
    }
}
