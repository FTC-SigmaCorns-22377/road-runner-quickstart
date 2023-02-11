package org.firstinspires.ftc.teamcode.Robot.Commands.VisionCommands;

import org.firstinspires.ftc.teamcode.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism.Turret;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Vision.BackCamera;
import org.firstinspires.ftc.teamcode.VisionUtils.Cone;

public class TurretToConestack extends Command {
	private final Turret turret;
	private final BackCamera backCamera;
	private Cone target = null;

	public TurretToConestack(Turret turret, BackCamera backCamera) {
		super(turret, backCamera);
		this.turret = turret;
		this.backCamera = backCamera;
	}

	@Override
	public void init() {
		this.target = this.backCamera.getConeStack();
		if (this.target != null) {
			this.turret.setBasedTurretPosition(this.target.position.angle + this.target.position.cameraPosition.getHeading());
		}
	}

	@Override
	public void periodic() {
	}

	@Override
	public boolean completed() {
		return true;
	}

	@Override
	public void shutdown() {
		turret.shutdown();
	}

}
