package org.firstinspires.ftc.teamcode.commands.groups;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.Com_Shooter;

public class SequentialShooter extends SequentialCommandGroup {

    private InstantCommand runFlyWheelCommand;
    private WaitCommand waitCommand;
    private Com_Shooter shooterCommand;

    public SequentialShooter(InstantCommand commandOne, WaitCommand commandTwo,
                             Com_Shooter commandThree) {
        runFlyWheelCommand = commandOne;
        waitCommand = commandTwo;
        shooterCommand = commandThree;

        addCommands(runFlyWheelCommand, waitCommand, shooterCommand);
    }

    @Override
    public void end(boolean interrupted){
        shooterCommand.stopShooter();
    }
}
