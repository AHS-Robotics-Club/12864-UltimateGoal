package org.firstinspires.ftc.teamcode.subsystems.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSystem;

public class Com_IntakeStart extends CommandBase {
    private final IntakeSystem intakeSystem;

    public Com_IntakeStart(IntakeSystem subby){
        intakeSystem = subby;
        addRequirements(intakeSystem);
    }
    @Override
    public void execute(){
        intakeSystem.suck();
    }
}