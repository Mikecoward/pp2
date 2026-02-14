package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Cat Bot Teleop BLUEa", group = "Teleop")
public class CatBotTeleopBlue extends BaseCatBotTeleop {
    @Override
    protected Alliance getAlliance() {
        return Alliance.BLUE;
    }
}