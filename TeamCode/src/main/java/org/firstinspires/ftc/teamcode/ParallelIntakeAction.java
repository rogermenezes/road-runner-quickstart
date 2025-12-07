package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

public class ParallelIntakeAction implements Action {
    private final ParallelShooter parallelShooter;

    public ParallelIntakeAction(ParallelShooter shooter) { this.parallelShooter = shooter; }

    @Override
    public boolean run(@NonNull TelemetryPacket packet) {
        parallelShooter.updateIntake(null); // or pass telemetry
        return parallelShooter.count < 3;   // keep running until full
    }
}