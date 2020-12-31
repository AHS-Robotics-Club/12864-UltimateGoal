package org.firstinspires.ftc.teamcode.commands.drive;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.DriveSystem;

import java.util.function.DoubleSupplier;

public class Com_Drive extends CommandBase {
    private final DriveSystem mecDrive;
    private final DoubleSupplier m_strafe, m_forward, m_turn;
    private final DoubleSupplier multiplier;

    public Com_Drive(DriveSystem subsystem, DoubleSupplier strafe, DoubleSupplier forward, DoubleSupplier turn, DoubleSupplier mult){
        mecDrive = subsystem;
        m_strafe = strafe;
        m_forward = forward;
        m_turn = turn;
        multiplier = mult;

        addRequirements(subsystem);
    }
    public Com_Drive(DriveSystem subsystem, DoubleSupplier strafe, DoubleSupplier forward, DoubleSupplier turn){
        mecDrive = subsystem;
        m_strafe = strafe;
        m_forward = forward;
        m_turn = turn;
        multiplier = ()-> 1.0;

        addRequirements(subsystem);
    }

    @Override
    public void execute(){
        mecDrive.drive(m_strafe.getAsDouble() * multiplier.getAsDouble(),
                m_forward.getAsDouble() * multiplier.getAsDouble(),
                m_turn.getAsDouble() * multiplier.getAsDouble());
    }
}
