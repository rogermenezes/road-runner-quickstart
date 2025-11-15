package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {
    private DcMotorEx motor;

    public Intake(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, "intake");
    }

    /** Run the intake inward (collect) */
    public Action intakeIn(double power) {
        return new Action() {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    motor.setPower(power);
                    initialized = true;
                }

                // Optional: report telemetry to Dashboard
                packet.put("intakePower", motor.getPower());

                // Return false → one-shot action (no blocking)
                return false;
            }
        };
    }

    /** Run the intake outward (reverse) */
    public Action intakeOut(double power) {
        return new Action() {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    motor.setPower(-power);
                    initialized = true;
                }

                packet.put("intakePower", motor.getPower());
                return false;
            }
        };
    }

    /** Stop the intake */
    public Action stop() {
        return new Action() {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    motor.setPower(0.0);
                    initialized = true;
                }

                packet.put("intakePower", motor.getPower());
                return false;
            }
        };
    }
}
