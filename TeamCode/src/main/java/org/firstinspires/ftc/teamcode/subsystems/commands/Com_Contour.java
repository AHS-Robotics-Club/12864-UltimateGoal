package org.firstinspires.ftc.teamcode.subsystems.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.ContourVisionSystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSystem;

public class Com_Contour extends CommandBase {
    private final ContourVisionSystem visionSystem;

    public Com_Contour(ContourVisionSystem subby){
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
