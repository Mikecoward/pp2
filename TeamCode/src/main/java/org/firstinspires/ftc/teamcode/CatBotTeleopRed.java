package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Cat Bot Teleop RED", group = "Teleop")
public class CatBotTeleopRed extends BaseCatBotTeleop {
    @Override
    protected Alliance getAlliance() {
        return Alliance.RED;
    }
}