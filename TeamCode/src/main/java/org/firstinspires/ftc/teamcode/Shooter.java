package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Shooter {

    private CRServo drum;
    private CRServo kicker;
    private DcMotor turret1;
    private DcMotor turret2;
    private DcMotor encoder;

    private final ElapsedTime timer = new ElapsedTime();
    private static final double TICKS_PER_REV = 28.0;

    public Shooter(HardwareMap hardwareMap) {
        drum    = hardwareMap.get(CRServo.class, "drum");
        kicker  = hardwareMap.get(CRServo.class, "kicker");
        turret1 = hardwareMap.get(DcMotor.class, "turret1");
        turret2 = hardwareMap.get(DcMotor.class, "turret2");
        encoder = hardwareMap.get(DcMotor.class, "encoder");

        encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public double getDegrees() {
        return (encoder.getCurrentPosition() / TICKS_PER_REV) * 360.0;
    }

    public void advanceDrumTo(LinearOpMode opMode,
                               double targetDegrees,
                               Telemetry telemetry,
                               boolean logDegrees) {
        double degrees = getDegrees();

        while (opMode.opModeIsActive() && degrees < targetDegrees) {
            degrees = getDegrees();
            if (logDegrees && telemetry != null) {
                telemetry.addData("ShooterDegrees", "%.1f", degrees);
                telemetry.update();
            }
            drum.setPower(-1.0);
        }
        drum.setPower(0.0);
    }

    private void kickerCycle(LinearOpMode opMode) {
        // wait 700 ms
        timer.reset();
        while (opMode.opModeIsActive() && timer.milliseconds() < 700) {
            // just wait
        }

        // forward 500 ms
        timer.reset();
        while (opMode.opModeIsActive() && timer.milliseconds() < 500) {
            kicker.setPower(1.0);
        }

        // back 125 ms
        timer.reset();
        while (opMode.opModeIsActive() && timer.milliseconds() < 125) {
            kicker.setPower(-0.5);
        }

        // small holding power like your TeleOp
        //kicker.setPower(-0.01);
    }

    /** Fixed spin-up for auto (no gamepad). */
    public void spinUpForAuto() {
        turret1.setPower(-0.8);
        turret2.setPower(0.8);
    }

    public void stopShooter() {
        turret1.setPower(0.0);
        turret2.setPower(0.0);
    }

    /** Shoot 3 balls in autonomous, then return drum to intake position. */
    public void shootThreeBalls(LinearOpMode opMode, Telemetry telemetry) {
        // ---- First ball ----

        kicker.setPower(-0.01);
        double targetDegrees = getDegrees() + 8750;
        advanceDrumTo(opMode, targetDegrees, telemetry, true);
        kickerCycle(opMode);

        // ---- Second ball ----
        targetDegrees += 35000;
        advanceDrumTo(opMode, targetDegrees, telemetry, false);
        kickerCycle(opMode);

        // ---- Third ball ----
        targetDegrees += 35000;
        advanceDrumTo(opMode, targetDegrees, telemetry, false);
        kickerCycle(opMode);

        // Return drum toward intake position
        targetDegrees += 17500;
        advanceDrumTo(opMode, targetDegrees, telemetry, false);

        // stop flywheels
        stopShooter();
    }
}

