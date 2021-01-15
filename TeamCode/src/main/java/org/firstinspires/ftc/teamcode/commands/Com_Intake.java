package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

public class Com_Intake extends CommandBase {

    private IntakeSubsystem intakeSubsystem;

    public Com_Intake(IntakeSubsystem subsystem) {
        intakeSubsystem = subsystem;

        addRequirements(intakeSubsystem);
    }

    @Override
    public void execute(){
        intakeSubsystem.start();
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.stop();
    }
}
