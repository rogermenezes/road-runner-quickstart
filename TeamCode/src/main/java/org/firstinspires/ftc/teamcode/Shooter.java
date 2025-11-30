package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Shooter {

    private CRServo drum;
    private CRServo kicker;
    private DcMotor turret1;
    private DcMotor turret2;
    private DcMotor encoder;

    NormalizedColorSensor front;
    NormalizedColorSensor left;
    NormalizedColorSensor right;
    NormalizedColorSensor bottom;

    private final ElapsedTime timer = new ElapsedTime();
    private static final double TICKS_PER_REV = 28.0;

    public Shooter(HardwareMap hardwareMap) {
        drum    = hardwareMap.get(CRServo.class, "drum");
        kicker  = hardwareMap.get(CRServo.class, "kicker");
        turret1 = hardwareMap.get(DcMotor.class, "turret1");
        turret2 = hardwareMap.get(DcMotor.class, "turret2");
        encoder = hardwareMap.get(DcMotor.class, "encoder");

        front = hardwareMap.get(NormalizedColorSensor.class, "front");
        front.setGain(20);
        left = hardwareMap.get(NormalizedColorSensor.class, "left");
        left.setGain(20);
        right = hardwareMap.get(NormalizedColorSensor.class, "right");
        right.setGain(20);
        bottom = hardwareMap.get(NormalizedColorSensor.class, "bottom");
        bottom.setGain(20);

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



    public void advanceDrumForIntake(LinearOpMode opMode,
                              double targetDegrees,
                              Telemetry telemetry,
                              boolean logDegrees,
                                     String detectedColor) {

        telemetry.addData("target degrees (adv)", targetDegrees);
        telemetry.update();

        double degrees = getDegrees();
        while (opMode.opModeIsActive() && degrees < targetDegrees) {
            if (logDegrees && telemetry != null) {
                telemetry.addData("ShooterDegrees", "%.1f", degrees);
                telemetry.update();
            }
            drum.setPower(-3.0);
            degrees = getDegrees();
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

    public void warmUpDrum(LinearOpMode opMode, Telemetry telemetry, boolean firstIntakeEver) {

        double targetDegrees = getDegrees() + 35000;
        advanceDrumForIntake(opMode, targetDegrees, telemetry, true, "");
        targetDegrees = getDegrees() + 35000;
        advanceDrumForIntake(opMode, targetDegrees, telemetry, true, "");
        targetDegrees = getDegrees() + 35000;
        advanceDrumForIntake(opMode, targetDegrees, telemetry, true, "");
    }

    public void intakeThreeBalls(LinearOpMode opMode, Telemetry telemetry, boolean firstIntakeEver) {

        //verify with Harsh the below statement
        double targetDegrees = getDegrees() + 35000;
        if (firstIntakeEver) {
            targetDegrees = 25000;
            telemetry.addData("target degrees, first time: ", targetDegrees);
        }
        advanceDrumForIntake(opMode, targetDegrees, telemetry, true, "");
        targetDegrees = getDegrees() + 35000;
        int ballsIntake = 0;
        while(ballsIntake < 3) {
            NormalizedRGBA colors = front.getNormalizedColors();
            float r = colors.red / colors.alpha;
            float g = colors.green / colors.alpha;
            float b = colors.blue / colors.alpha;

            String detectedColor = detectGreenOrPurple(r, g, b, telemetry);
            if (!detectedColor.equalsIgnoreCase("P") &&
                    !detectedColor.equalsIgnoreCase("G")) {
                continue;
            }
            telemetry.addData("Detected Color", detectedColor);

            advanceDrumForIntake(opMode, targetDegrees, telemetry, true, "");
            targetDegrees += 35000;
            ballsIntake++;

        }

    }

    private String detectGreenOrPurple(float r, float g, float b, Telemetry telemetry) {
        float sum = r + g + b;

        double nr = (double) r / sum;
        double ng = (double) g / sum;
        double nb = (double) b / sum;
        telemetry.addData("nr", nr);
        telemetry.addData("ng", ng);
        telemetry.addData("nb", nb);

        if (nr >= 0.24 && nr <= 0.30 && ng >= 0.26 && ng <= 0.38 && nb >= 0.35 && nb <= 0.51) {
            return "P";
        }
        else if (nr >= 0.11 && nr <= 0.21 && ng >= 0.43 && nb >= 0.32 ) {
            return "G";
        }
        else if (nr >= 0.21 && nr <= 0.30 && ng >= 0.40 && ng <= 0.45 && nb >= 0.30 && nb <= 0.41) {
            return "N";
        }
        else if (nr >= 0.32 && nr <= 0.37 && ng >= 0.37 && ng <= 0.40 && nb >= 0.24 && nb <= 0.29 ) {
            return "R";
        }
        else {
            return "U";
        }
    }


}

