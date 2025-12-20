package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "CatBot Auto BLUE", group = "Autonomous")
public class CatBotAutoBlue extends BaseCatBotAuto {
    @Override
    protected Alliance getAlliance() {
        return Alliance.BLUE;
    }
}