package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Shooter {

    private Servo drum;
    private DcMotor turret1;
    private DcMotor turret2;
    private DcMotor encoder;

    private DcMotor intake;

    private Servo kicker;

    NormalizedColorSensor front;
    NormalizedColorSensor left;
    NormalizedColorSensor right;
    NormalizedColorSensor bottom;

    private final ElapsedTime runTime = new ElapsedTime();
    private static final double TICKS_PER_REV = 28.0;

    int count = -1;

    double position = 0.0;



    public Shooter(HardwareMap hardwareMap) {
        drum    = hardwareMap.get(Servo.class, "drum");
        drum.setPosition(0);

        turret1 = hardwareMap.get(DcMotor.class, "turret1");
        turret2 = hardwareMap.get(DcMotor.class, "turret2");
        encoder = hardwareMap.get(DcMotor.class, "encoder");

        kicker = hardwareMap.get(Servo.class, "kicker");

        front = hardwareMap.get(NormalizedColorSensor.class, "front");
        front.setGain(20);
        left = hardwareMap.get(NormalizedColorSensor.class, "left");
        left.setGain(20);
        right = hardwareMap.get(NormalizedColorSensor.class, "right");
        right.setGain(20);
        bottom = hardwareMap.get(NormalizedColorSensor.class, "bottom");
        bottom.setGain(20);

        intake = hardwareMap.dcMotor.get("intake");

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
            //drum.setPower(-1.0);
        }
        //drum.setPower(0.0);
    }

    public void intake(
            LinearOpMode opMode,
            Telemetry telemetry
            ) {

        intake.setPower(1);
        telemetry.addData("Drum position", drum.getPosition());
        telemetry.addData("count", count);

        //Rotate the drum while the count is < 3 and there is a ball sucked in and we are not shooting
        while (
                opMode.opModeIsActive() &&
                (count < 3)) {

            NormalizedRGBA colors = front.getNormalizedColors();
            float r = colors.red / colors.alpha;
            float g = colors.green / colors.alpha;
            float b = colors.blue / colors.alpha;

            String detectedColor = detectGreenOrPurple(r, g, b, telemetry);
            if (!detectedColor.equalsIgnoreCase("P") &&
                    !detectedColor.equalsIgnoreCase("G")) {
                continue;
            }


            count = count + 1;
            telemetry.addData("count", count);
            telemetry.update();
            if (count == 1 || count == 2) { //I wonder why the count becomes 2 sometimes.
                position = position + 0.2;
                drum.setPosition(position);
            } else if (count == 3) {
                position = position + 0.42;
                drum.setPosition(position);
            }
            telemetry.addData("Drum position", drum.getPosition());

            runTime.reset();
            while(runTime.milliseconds() < 125) {}
        }
    }

    private void setShooterSpeed(double high, double mid, double low) {
            turret1.setPower(high);
            turret2.setPower(-high);

//        if (gamepad1.dpad_up){
//            turret1.setPower(high);
//            turret2.setPower(-high);
//        }
//        if (gamepad1.dpad_right){
//            turret1.setPower(mid);
//            turret2.setPower(-mid);
//        }
//        if (gamepad1.dpad_left){
//            turret1.setPower(mid);
//            turret2.setPower(-mid);
//        }
//        if (gamepad1.dpad_down){
//            turret1.setPower(low);
//            turret2.setPower(-low);
//        }
    }


    public void aboutToShoot() {
        setShooterSpeed(1, 0.9, 0.8);
        //Realign the kicker
        kicker.setPosition(0.01);

    }

    public void shootThreeBalls(double firstPos, double secondPos, double thirdPos) {

        //Shoot first ball
        position = firstPos;
        drum.setPosition(position);
        runTime.reset();
        while(runTime.milliseconds() < 1000) {}

        kicker.setPosition(0.5);

        runTime.reset();
        while(runTime.milliseconds() < 125) {}
        kicker.setPosition(0.01);


        //Shoot second ball
        setShooterSpeed(1, 0.9, 0.8);

        position = secondPos;
        drum.setPosition(position);

        runTime.reset();
        while(runTime.milliseconds() < 1000) {}

        kicker.setPosition(0.5);
        runTime.reset();
        while(runTime.milliseconds() < 125) {}
        kicker.setPosition(0.01);


        //Shoot third ball
        setShooterSpeed(1, 0.9, 0.8);
        position = thirdPos;
        drum.setPosition(position);

        runTime.reset();
        while(runTime.milliseconds() < 1000) {}

        kicker.setPosition(0.5);
        runTime.reset();
        while(runTime.milliseconds() < 125) {}
        kicker.setPosition(0.01);

        count = 0;
        count--; //I cant explain why it has to be -1

        //Move back to 0
        position = 0;
        drum.setPosition(position);

        runTime.reset();
        while(runTime.milliseconds() < 125) {}

        //Reset shooter power
        turret1.setPower(0);
        turret2.setPower(0);

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
            //drum.setPower(-3.0);
            degrees = getDegrees();
        }
        //drum.setPower(0.0);
    }

    private void kickerCycle(LinearOpMode opMode) {
        // wait 700 ms
        runTime.reset();
        while (opMode.opModeIsActive() && runTime.milliseconds() < 700) {
            // just wait
        }

        // forward 500 ms
        runTime.reset();
        while (opMode.opModeIsActive() && runTime.milliseconds() < 500) {
            //kicker.setPower(1.0);
        }

        // back 125 ms
        runTime.reset();
        while (opMode.opModeIsActive() && runTime.milliseconds() < 125) {
            //kicker.setPower(-0.5);
        }

        // small holding power like your TeleOp
        //kicker.setPower(-0.01);
    }

    /** Fixed spin-up for auto (no gamepad). */
    public void spinUpForAuto() {
        turret1.setPower(-0.9);
        turret2.setPower(0.9);
    }

    public void stopShooter() {
        turret1.setPower(0.0);
        turret2.setPower(0.0);
    }

    /** Shoot 3 balls in autonomous, then return drum to intake position. */
    public void shootThreeBalls(LinearOpMode opMode, Telemetry telemetry) {
        // ---- First ball ----

        //kicker.setPower(-0.01);
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

        //verify with Harsha the below statement
        double targetDegrees = getDegrees() + 35000;
        if (firstIntakeEver) {
            targetDegrees = 25000;
            telemetry.addData("target degrees, first time: ", targetDegrees);
        }
        advanceDrumForIntake(opMode, targetDegrees, telemetry, true, "");
        targetDegrees = getDegrees() + 35000;
        int ballsIntake = 0;
        while(opMode.opModeIsActive() && ballsIntake < 3) {
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

