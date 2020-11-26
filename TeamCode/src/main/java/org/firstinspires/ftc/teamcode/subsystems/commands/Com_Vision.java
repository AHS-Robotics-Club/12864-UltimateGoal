package org.firstinspires.ftc.teamcode.subsystems.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.VisionSystem;

public class Com_Vision extends CommandBase {
    private final VisionSystem visionSystem;

    public Com_Vision(VisionSystem subby){
        visionSystem = subby;
        addRequirements(subby);
    }
    @Override
    public void initialize(){
        visionSystem.getStackSize();
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}
