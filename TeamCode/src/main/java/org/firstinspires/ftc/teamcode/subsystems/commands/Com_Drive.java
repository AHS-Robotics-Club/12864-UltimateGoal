package org.firstinspires.ftc.teamcode.subsystems.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.DriveSystem;

import java.util.function.DoubleSupplier;

public class Com_Drive extends CommandBase {
    private final DriveSystem mecDrive;
    private final DoubleSupplier m_strafe;
    private final DoubleSupplier m_forward;
    private final DoubleSupplier m_turn;

    public Com_Drive(DriveSystem subsystem, DoubleSupplier strafe, DoubleSupplier forward, DoubleSupplier turn){
        mecDrive = subsystem;
        m_strafe = strafe;
        m_forward = forward;
        m_turn = turn;

        addRequirements(subsystem);
    }

    @Override
    public void execute(){
        //TODO: Check config and wiring to see what is up with this poopoo
        mecDrive.drive(m_strafe.getAsDouble(), m_forward.getAsDouble(), m_turn.getAsDouble());
    }
}
