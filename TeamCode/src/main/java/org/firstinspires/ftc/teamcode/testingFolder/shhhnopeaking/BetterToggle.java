package org.firstinspires.ftc.teamcode.testingFolder.shhhnopeaking;

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
            //hehehehehe toggle between 100 commands
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

    public BetterToggle toggleWhenPressed(final Command commandOne, final Command commandTwo, boolean interruptible) {
        CommandScheduler.getInstance().addButton(new Runnable() {
            private boolean m_pressedLast = get();
            private boolean m_firstCommandActive = false;

            @Override
            public void run() {
                boolean pressed = get();
                if (!m_pressedLast && pressed) {
                    if (m_firstCommandActive) {
                        if (commandOne.isScheduled()) {
                            commandOne.cancel();
                        }
                        commandTwo.schedule(interruptible);
                    } else {
                        if (commandTwo.isScheduled()) {
                            commandTwo.cancel();
                        }
                        commandOne.schedule(interruptible);
                    }

                    m_firstCommandActive = !m_firstCommandActive;
                }

                m_pressedLast = pressed;
            }
        });
        return this;
    }

    public BetterToggle toggleWhenPressed(final Command commandOne, final Command commandTwo) {
        toggleWhenPressed(commandOne, commandTwo, true);
        return this;
    }

    public BetterToggle toggleWhenPressed(final Runnable runnableOne, final Runnable runnableTwo) {
        toggleWhenPressed(new InstantCommand(runnableOne), new InstantCommand(runnableTwo));
        return this;
    }

}
