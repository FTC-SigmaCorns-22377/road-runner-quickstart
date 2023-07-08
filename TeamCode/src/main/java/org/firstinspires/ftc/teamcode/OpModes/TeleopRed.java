package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.OpModes.CRI_OPMODES.Teleop;
import org.firstinspires.ftc.teamcode.Utils.Team;

@TeleOp
public class TeleopRed extends Teleop {
	@Override
	public Team getTeam() {
		return Team.RED;
	}
}
