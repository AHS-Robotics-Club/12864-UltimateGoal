package org.firstinspires.ftc.teamcode.jacksonSama.shhhnopeaking;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.button.Button;

public class BetterToggle extends Button {

    public BetterToggle multiToggle(boolean interruptible, final Command ...command){
        CommandScheduler.getInstance().addButton(new Runnable() {
            private int i = 0;
            private boolean m_pressedLast = get();

            @Override
            public void run() {
                boolean pressed = get();
                if (!m_pressedLast && pressed) {
                    if (i == command.length)
                        i = 0;
                    if (i > 0 && command[i - 1].isScheduled())
                        command[i - 1].cancel();
                    else if (i == 0 && command[command.length - 1].isScheduled())
                        command[command.length - 1].cancel();
                    else
                        command[i].schedule(interruptible);
                    i++;
                }
                    m_pressedLast = pressed;
            }
        });
        return this;
    }
}
