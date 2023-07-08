package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.CommandFramework.BaseAuto;
import org.firstinspires.ftc.teamcode.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.CommandFramework.CommandScheduler;
import org.firstinspires.ftc.teamcode.Robot.Commands.MiscCommands.ManualControl;
import org.firstinspires.ftc.teamcode.Robot.Commands.ScoringCommands.primitiveMovements.ManualTurretTesting;
import org.firstinspires.ftc.teamcode.Utils.Team;


@Disabled
public class aaWhatIsWrongWithOurRobot extends BaseAuto {

    @Override
    public Command setupAuto(CommandScheduler scheduler) {
        waitForStart();
        return new ManualControl(robot.scoringMechanism.turret, robot.scoringMechanism.horizontalExtension);
    }

    @Override
    public Team getTeam() {
        return Team.RED;
    }
}
