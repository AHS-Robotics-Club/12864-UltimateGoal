package org.firstinspires.ftc.teamcode.subsystems.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.RingPushSystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSystem;

public class Com_Push extends CommandBase {
    private final RingPushSystem pushSystem;
    public Com_Push(RingPushSystem subby){
        pushSystem = subby;
        addRequirements(pushSystem);
    }
    @Override
    public void execute(){
        pushSystem.push();
    }
}
