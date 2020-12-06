package org.firstinspires.ftc.teamcode.subsystems.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.ContourVisionSystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSystem;

public class Com_Contour extends CommandBase {
    private final ContourVisionSystem visionSystem;
    private ElapsedTime timer;
    public Com_Contour(ContourVisionSystem subby, ElapsedTime timee){
        visionSystem = subby;

        timer = timee;
        addRequirements(subby);
    }

    @Override
    public void initialize(){
        timer.reset();
        while(timer.seconds() >= 1)
            visionSystem.getStackSize();
    }

    @Override
    public boolean isFinished(){
        return timer.seconds() >= 1.5;
    }
}
