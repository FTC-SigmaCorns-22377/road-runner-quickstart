package org.firstinspires.ftc.teamcode.Robot.Commands.ScoringCommands.primitiveMovements;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism.Turret;
import org.firstinspires.ftc.teamcode.VisionUtils.IntakeParameters;

public class MoveArmIfCone extends Command {
    double delayS = 0.15;

    Turret turret;
    Turret.ArmStates armStates;
    IntakeParameters intakeParameters;

    ElapsedTime timer = new ElapsedTime();

    public MoveArmIfCone(Turret turret, Turret.ArmStates armStates, IntakeParameters intakeParameters) {
        this.turret = turret;
        this.armStates = armStates;
        this.intakeParameters = intakeParameters;
    }

    @Override
    public void init() {
        if (intakeParameters.foundCone) {
            turret.setArm(armStates);
            timer.reset();
        }
    }

    @Override
    public void periodic() {

    }

    @Override
    public boolean completed() {
        return (timer.seconds() > delayS) || !intakeParameters.foundCone;
    }

    @Override
    public void shutdown() {

    }
}
