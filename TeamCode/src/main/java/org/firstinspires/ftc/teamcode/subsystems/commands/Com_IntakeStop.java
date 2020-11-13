package org.firstinspires.ftc.teamcode.subsystems.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSystem;

public class Com_IntakeStop extends CommandBase {
    private final IntakeSystem intakeSystem;

    public Com_IntakeStop(IntakeSystem subby){
        intakeSystem = subby;
        addRequirements(intakeSystem);
    }
    @Override
    public void execute(){
        intakeSystem.stop();
    }
}
