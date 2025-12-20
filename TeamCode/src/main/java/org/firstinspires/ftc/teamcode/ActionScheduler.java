package org.firstinspires.ftc.teamcode;

import java.util.Comparator;
import java.util.PriorityQueue;

public class ActionScheduler {
    private static class Scheduled {
        final double dueSec;
        final Runnable action;
        Scheduled(double dueSec, Runnable action) {
            this.dueSec = dueSec;
            this.action = action;
        }
    }

    private final PriorityQueue<Scheduled> pq =
            new PriorityQueue<>(Comparator.comparingDouble(s -> s.dueSec));

    /** Schedule an action delayMs from "now". */
    public void inMs(double nowSec, long delayMs, Runnable action) {
        pq.add(new Scheduled(nowSec + delayMs / 1000.0, action));
    }

    /** Schedule an action at an absolute time (seconds). */
    public void atSec(double dueSec, Runnable action) {
        pq.add(new Scheduled(dueSec, action));
    }

    /** Call frequently (each loop). Runs all due actions. */
    public void update(double nowSec) {
        while (!pq.isEmpty() && pq.peek().dueSec <= nowSec) {
            pq.poll().action.run();
        }
    }

    public void clear() {
        pq.clear();
    }

    public boolean isEmpty() {
        return pq.isEmpty();
    }
}