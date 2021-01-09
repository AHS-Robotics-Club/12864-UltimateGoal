package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

public class Com_Outtake extends CommandBase {

    private IntakeSubsystem intakeSubsystem;

    public Com_Outtake(IntakeSubsystem subsystem) {
        intakeSubsystem = subsystem;

        addRequirements(intakeSubsystem);
    }

    @Override
    public void execute(){
        intakeSubsystem.outtake();
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.stop();
    }
}
