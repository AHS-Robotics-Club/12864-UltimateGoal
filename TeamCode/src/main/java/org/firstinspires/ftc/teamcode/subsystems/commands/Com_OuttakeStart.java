package org.firstinspires.ftc.teamcode.subsystems.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandScheduler;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSystem;

public class Com_OuttakeStart extends CommandBase {
    private final IntakeSystem intakeSystem;

    public Com_OuttakeStart(IntakeSystem subby){
        intakeSystem = subby;
        addRequirements(intakeSystem);
    }
    @Override
    public void execute(){
        intakeSystem.suck();
    }
    @Override
    public void cancel() {
        intakeSystem.stop(); /* or whatever the name of that private instance is */
        CommandScheduler.getInstance().cancel(this);
    }
}

