package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Utils.Team;

@Disabled
public class VisionLatteBlue extends VisionLatteRed {
    @Override
    public Team getTeam() {
        return Team.BLUE;
    }
}