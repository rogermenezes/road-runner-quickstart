package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ParallelShooter {

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

    /**
     * counts the number of ball in the drum.
     */
    int count = -1;

    double position = 0.0;

    public ParallelShooter(HardwareMap hardwareMap) {
        drum = hardwareMap.get(Servo.class, "drum");
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

    public void intakeOn() { intake.setPower(1); }
    public void intakeOff() { intake.setPower(0); }

    public void updateIntake(Telemetry telemetry) {
        if (count >= 3) return;

        NormalizedRGBA colors = front.getNormalizedColors();
        float r = colors.red / colors.alpha;
        float g = colors.green / colors.alpha;
        float b = colors.blue / colors.alpha;

        String detectedColor = detectGreenOrPurple(r, g, b, telemetry);
        if (!detectedColor.equalsIgnoreCase("P") &&
                !detectedColor.equalsIgnoreCase("G")) {
            return; // Nothing to do this tick
        }

        count++;
        if (count == 1 || count == 2) {
            position += 0.2;
        } else if (count == 3) {
            position += 0.42;
        }
        drum.setPosition(position);
    }

    private void setShooterSpeed(double high, double mid, double low) {
        //TODO: we need some logic on when to use one of high, mid or low
        turret1.setPower(high);
        turret2.setPower(-high);
    }

    public void shootThreeBalls(double firstPos, double secondPos, double thirdPos) {

        //about to shoot
        setShooterSpeed(1, 0.9, 0.8);
        kicker.setPosition(0.01);

        //Shoot first ball
        position = firstPos;
        drum.setPosition(position);
        runTime.reset();
        while(runTime.milliseconds() < 1000) {}

        //lift the kicker
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

    /** Fixed spin-up for auto (no gamepad). */
    public void spinUpForAuto() {
        turret1.setPower(-0.9);
        turret2.setPower(0.9);
    }

    public void stopShooter() {
        turret1.setPower(0.0);
        turret2.setPower(0.0);
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

