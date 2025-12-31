package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Autonomous(name = "Cat Bot Auto RED Dummy", group = "Autonomous")
public class CatBotAutoRedDummy extends basecatbotAuto2   {
    @Override
    protected Alliance getAlliance() {
        return Alliance.BLUE;
    }
}