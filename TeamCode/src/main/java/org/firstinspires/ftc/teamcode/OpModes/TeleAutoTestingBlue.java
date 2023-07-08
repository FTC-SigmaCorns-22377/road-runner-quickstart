package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.teamcode.Utils.Team;

@Disabled
public class TeleAutoTestingBlue extends TeleAutoTesting{
    @Override
    public Team getTeam() {
        return Team.BLUE;
    }
}
