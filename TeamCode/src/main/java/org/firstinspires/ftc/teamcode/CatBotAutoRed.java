package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "CatBot Auto RED", group = "Autonomous")
public class CatBotAutoRed extends BaseCatBotAuto {
    @Override
    protected Alliance getAlliance() {
        return Alliance.RED;
    }
}