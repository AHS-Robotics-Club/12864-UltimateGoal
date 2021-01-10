package org.firstinspires.ftc.teamcode.commands.groups;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.Com_Shooter;

public class SequentialShooter extends SequentialCommandGroup {

    private Com_Shooter shooterCommand;

    public SequentialShooter(InstantCommand runFlyWheelCommand, WaitCommand waitCommand,
                             Com_Shooter shooterCommand) {
        this.shooterCommand = shooterCommand;

        addCommands(runFlyWheelCommand, waitCommand, shooterCommand);
    }

    @Override
    public void end(boolean interrupted){
        shooterCommand.stopShooter();
        shooterCommand.returnHome();
    }
}
