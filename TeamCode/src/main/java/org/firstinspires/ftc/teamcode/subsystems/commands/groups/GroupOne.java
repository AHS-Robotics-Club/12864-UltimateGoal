package org.firstinspires.ftc.teamcode.subsystems.commands.groups;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.DriveSystem;
import org.firstinspires.ftc.teamcode.subsystems.WobbleSystem;
import org.firstinspires.ftc.teamcode.subsystems.commands.Com_PutDown;
import org.firstinspires.ftc.teamcode.subsystems.commands.drive.Com_DriveTime;

public class GroupOne extends SequentialCommandGroup {
    public GroupOne(DriveSystem drive, ElapsedTime time, VoltageSensor voltageSensor, WobbleSystem wobbleSystem) {
        addCommands(
                new Com_DriveTime(drive,0.5, 0D, 0D, time, 2.0),
                new Com_DriveTime(drive, (13/voltageSensor.getVoltage()) * -0.55, 0D, 0D, time, 4.5),
                new Com_DriveTime(drive, 0D, (13/voltageSensor.getVoltage()) * -0.55, 0D, time, 3.9),
                new Com_PutDown(wobbleSystem, time),
                new Com_DriveTime(drive, 0D, (13/voltageSensor.getVoltage())*0.55, 0D, time, 2)
                );

    }
}