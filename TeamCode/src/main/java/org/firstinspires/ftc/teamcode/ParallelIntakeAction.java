package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ParallelIntakeAction implements Action {
    private final ParallelShooter shooter;
    private final Telemetry telemetry;

    ElapsedTime timer = new ElapsedTime();


    public ParallelIntakeAction(ParallelShooter shooter, Telemetry telemetry) {
        this.shooter = shooter;
        this.telemetry = telemetry;
        // if you want to start intake motor here:
        //shooter.intake.setPower(1.0); // or shooter.intakeOn();
    }

    @Override
    public boolean run(@NonNull TelemetryPacket packet) {
        shooter.updateIntakeV2(telemetry);
        return (timer.seconds() < 10);   // keep running until full
    }
}