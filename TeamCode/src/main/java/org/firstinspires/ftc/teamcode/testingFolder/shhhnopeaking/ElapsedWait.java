package org.firstinspires.ftc.teamcode.testingFolder.shhhnopeaking;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

public class ElapsedWait extends CommandBase {

    protected ElapsedTime timer;
    private long leTime;

    public ElapsedWait(long millis) {
        timer = new ElapsedTime();
        leTime = millis;
        setName(m_name + ": " + millis + " milliseconds");
    }

    @Override
    public void initialize() {
        timer.reset();
    }

    @Override
    public boolean isFinished() {
        return timer.milliseconds() >= leTime;
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
}
