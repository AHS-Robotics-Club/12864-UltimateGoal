package org.firstinspires.ftc.teamcode.jacksonSama.shhhnopeaking;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.button.Button;
//I call this better toggle because its like more toggle options
public class BetterToggle extends Button {

    public BetterToggle multiToggle(boolean interruptible, final Command... command) {
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
                    command[i].schedule(interruptible);
                    i++;
                }
                m_pressedLast = pressed;
            }
        });
        return this;
    }

    public BetterToggle toggleWhenPressed(final Command command1, final Command command2, boolean interruptible) {
        CommandScheduler.getInstance().addButton(new Runnable() {
            private boolean m_pressedLast = get();
            @Override
            public void run() {
                boolean pressed = get();
                if (!m_pressedLast && pressed) {
                    if (command1.isScheduled()) {
                        command1.cancel();
                        command2.schedule(interruptible);
                    } else if(command2.isScheduled()){
                        command2.cancel();
                        command1.schedule();
                    } else {
                        command1.schedule(interruptible);
                    }
                }

                m_pressedLast = pressed;
            }
        });
        return this;
    }

    public BetterToggle toggleWhenPressed(final Command command1, final Command command2) {
        toggleWhenPressed(command1, command2, true);
        return this;
    }

    public BetterToggle toggleWhenPressed(final Runnable runnable1, final Runnable runnable2) {
        toggleWhenPressed(new InstantCommand(runnable1), new InstantCommand(runnable2));
        return this;
    }

}
