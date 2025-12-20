package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Cat Bot Teleop BLUE", group = "Teleop")
public class CatBotTeleopBlue extends BaseCatBotTeleop {
    @Override
    protected Alliance getAlliance() {
        return Alliance.BLUE;
    }
}