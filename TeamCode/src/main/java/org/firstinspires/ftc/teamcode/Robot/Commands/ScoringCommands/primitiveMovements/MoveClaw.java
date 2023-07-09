package org.firstinspires.ftc.teamcode.Robot.Commands.ScoringCommands.primitiveMovements;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism.Turret;

public class MoveClaw extends Command {

	double delayS = 0.15;

	Turret turret;
	Turret.ClawStates clawState;

	ElapsedTime timer = new ElapsedTime();

	public MoveClaw(Turret turret, Turret.ClawStates clawState) {
		this.turret = turret;
		this.clawState = clawState;
	}

	@Override
	public void init() {
		turret.setClawGrabbing(clawState);
		timer.reset();
	}

	@Override
	public void periodic() {

	}

	@Override
	public boolean completed() {
		return timer.seconds() > delayS;
	}

	@Override
	public void shutdown() {

	}
}
