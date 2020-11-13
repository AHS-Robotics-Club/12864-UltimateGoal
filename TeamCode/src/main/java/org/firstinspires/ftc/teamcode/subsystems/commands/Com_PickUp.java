package org.firstinspires.ftc.teamcode.subsystems.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.WobbleSystem;

public class Com_PickUp extends CommandBase {
    private final WobbleSystem wobblySystem;

    public Com_PickUp(WobbleSystem subby){
        wobblySystem = subby;
        addRequirements(wobblySystem);
    }
    @Override
    public void execute(){
        wobblySystem.spinMeRightRoundBaby();
    }
}