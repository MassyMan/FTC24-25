package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.ArrayList;
import java.util.List;

@Disabled
public class ActionScheduler {

    private final List<ScheduledAction> scheduledActions = new ArrayList<>();
    private final ElapsedTime timer = new ElapsedTime();

    public void schedule(double delay, Runnable action) {
        scheduledActions.add(new ScheduledAction(timer.seconds() + delay, action));
    }

    public void runScheduled() {
        double currentTime = timer.seconds();
        List<ScheduledAction> executedActions = new ArrayList<>();

        for (ScheduledAction action : scheduledActions) {
            if (currentTime >= action.runAt) {
                action.action.run();
                executedActions.add(action);
            }
        }

        // Remove executed actions from the list
        scheduledActions.removeAll(executedActions);
    }

    private static class ScheduledAction {
        double runAt;
        Runnable action;

        ScheduledAction(double runAt, Runnable action) {
            this.runAt = runAt;
            this.action = action;
        }
    }
}
