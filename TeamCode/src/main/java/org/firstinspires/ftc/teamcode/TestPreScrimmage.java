package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import java.util.List;

@TeleOp
public class TestPreScrimmage extends OpMode {
    DcMotor FL, FR, BL, BR;
    private Servo herringbone;
    private Servo rgbmid;
    private Servo rgbmid2;
    private Servo rgbmid3;
    private CRServo drum;
    private DcMotor turret1;
    private DcMotor turret2;
    private DcMotor intake;
    private DcMotor encoder;
    double total;
    double pos;
    NormalizedColorSensor front;
    NormalizedColorSensor left;
    NormalizedColorSensor right;
    NormalizedColorSensor bottom;
    String pattern_real = null;
    String pattern_current = null;

    private Limelight3A limelight;
    volatile double sharedTx = 0;
    volatile Position sharedPos = null;
    private double TARGET_DEGREES = 25000.0; //first time the encoder is slow
    ElapsedTime runTime = new ElapsedTime();
    int count = 0;
    final double TICKS_PER_REV = 28.0;

    private CRServo kicker;

    @Override
    public void init() {

        drum = hardwareMap.get(CRServo.class, "drum");

        telemetry.addData("Status", "Initialized");
        FL = hardwareMap.dcMotor.get("FL");
        FR = hardwareMap.dcMotor.get("FR");
        BL = hardwareMap.dcMotor.get("BL");
        BR = hardwareMap.dcMotor.get("BR");
        rgbmid = hardwareMap.get(Servo.class, "rgbmid");
        rgbmid2 = hardwareMap.get(Servo.class, "rgbmid2");
        rgbmid3 = hardwareMap.get(Servo.class, "rgbmid3");
        herringbone = hardwareMap.get(Servo.class, "herringbone");
        drum = hardwareMap.get(CRServo.class, "drum");
        turret1 = hardwareMap.dcMotor.get("turret1");
        turret2 = hardwareMap.dcMotor.get("turret2");
        intake = hardwareMap.dcMotor.get("intake");
        kicker = hardwareMap.get(CRServo.class, "kicker");
        telemetry.setMsTransmissionInterval(3);
        front = hardwareMap.get(NormalizedColorSensor.class, "front");
        front.setGain(20);
        left = hardwareMap.get(NormalizedColorSensor.class, "left");
        left.setGain(20);
        right = hardwareMap.get(NormalizedColorSensor.class, "right");
        right.setGain(20);
        bottom = hardwareMap.get(NormalizedColorSensor.class, "bottom");
        bottom.setGain(20);
        limelight = hardwareMap.get(Limelight3A.class, "Limelight30735");
        limelight.pipelineSwitch(0);
        limelight.start();

        encoder = hardwareMap.dcMotor.get("encoder");
        encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Reverse right side
        FR.setDirection(DcMotor.Direction.REVERSE);
        FL.setDirection(DcMotor.Direction.REVERSE);
        BL.setDirection(DcMotor.Direction.REVERSE);
    }

    @Override
    public void loop() {

        NormalizedRGBA colors = front.getNormalizedColors();
        float r = colors.red / colors.alpha;
        float g = colors.green / colors.alpha;
        float b = colors.blue / colors.alpha;

        NormalizedRGBA colors2 = left.getNormalizedColors();
        float r2 = colors2.red / colors2.alpha;
        float g2 = colors2.green / colors2.alpha;
        float b2 = colors2.blue / colors2.alpha;

        NormalizedRGBA colors3 = right.getNormalizedColors();
        float r3 = colors3.red / colors3.alpha;
        float g3 = colors3.green / colors3.alpha;
        float b3 = colors3.blue / colors3.alpha;

        NormalizedRGBA colors4 = bottom.getNormalizedColors();
        float r4 = colors4.red / colors4.alpha;
        float g4 = colors4.green / colors4.alpha;
        float b4 = colors4.blue / colors4.alpha;


        String detectedColor = detectGreenOrPurple(r, g, b);
        telemetry.addData("Detected Color", detectedColor);
        if (detectedColor.equalsIgnoreCase("P")) {
            pos = 0.71;//purple
            rgbmid.setPosition(pos);
        } else if (detectedColor.equalsIgnoreCase("G")) {
            pos = 0.5;//green
            rgbmid.setPosition(pos);
        } else{
            rgbmid.setPosition(0);
        }
        String detectedColor2 = detectGreenOrPurple(r2, g2, b2);
        telemetry.addData("Detected Color2", detectedColor2);
        if (detectedColor2.equalsIgnoreCase("P")) {
            pos = 0.71;//purple
            rgbmid2.setPosition(pos);
        } else if (detectedColor2.equalsIgnoreCase("G")) {
            pos = 0.5;//green
            rgbmid2.setPosition(pos);
        } else{
            rgbmid2.setPosition(0);
        }
        String detectedColor3 = detectGreenOrPurple(r3, g3, b3);
        telemetry.addData("Detected Color3", detectedColor3);
        if (detectedColor3.equalsIgnoreCase("P")) {
            pos = 0.71;//purple
            rgbmid3.setPosition(pos);
        } else if (detectedColor3.equalsIgnoreCase("G")) {
            pos = 0.5;//green
            rgbmid3.setPosition(pos);
        } else{
            rgbmid3.setPosition(0);
        }
        String detectedColor4 = detectGreenOrPurple(r4, g4, b4);
        telemetry.addData("Detected Color Bottom", detectedColor4);
        LLResult result = limelight.getLatestResult();
        List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
        telemetry.addData("numTags", tags.size());

        for (LLResultTypes.FiducialResult tag : tags) {
            switch ((int)tag.getFiducialId()) {
                case 23: pattern_real = "PPG";break;
                case 21: pattern_real = "GPP";break;
                case 22: pattern_real = "PGP";break;
            }
            telemetry.addData("Pattern Actual", pattern_real);
        }

        pattern_current = detectedColor3+detectedColor+detectedColor2;
        telemetry.addData("Current Pattern", pattern_current);
        //Raise the turret up and down
        if (gamepad1.dpad_up) {
            herringbone.setPosition(-0.3);
        }
        if (gamepad1.dpad_down) {
            herringbone.setPosition(0.3);
        }

        //Manual setting the shooter speed
        if (gamepad1.y) {
            turret1.setPower(-0.8);
            turret2.setPower(0.8);
        }

        //Manual reset the shooter speed
        if (gamepad1.x) {
            turret1.setPower(0);
            turret2.setPower(0);
        }

        //Manual kicker
        if (gamepad1.a) {
            count = 3;
        }

        //Manual ball spit out
        if (gamepad1.b) {
            runTime.reset();
            while(runTime.milliseconds() < 750) {
                intake.setPower(-1);
            }
        }

        //If Noball is detected start the intake. In case of jam, keep the intake on. Manually unjam
        //if (detectedColor.equalsIgnoreCase("NoBall") || detectedColor.equalsIgnoreCase("Red")) {
        intake.setPower(1);
        //} else {
        //     intake.setPower(0);
        //}

        //Fine adjustment in case it overshot
        if (gamepad1.left_trigger > 0) {
            drum.setPower(-0.3);
            TARGET_DEGREES = (encoder.getCurrentPosition() / TICKS_PER_REV) * 360.0;
        }

        //Stop fine adjustment
        if (gamepad1.right_trigger > 0) {
            drum.setPower(0);
        }

        telemetry.addData("count", count);
        //Rotate the drum while the count is < 3 and there is a ball sucked in and we are not shooting
        if ((count < 3) && (detectedColor.equalsIgnoreCase("P") || detectedColor.equalsIgnoreCase("G")) && (!gamepad1.left_bumper || !gamepad1.right_bumper)) {
            double degrees = (encoder.getCurrentPosition() / TICKS_PER_REV) * 360.0;
            while (degrees < TARGET_DEGREES) {
                degrees = (encoder.getCurrentPosition() / TICKS_PER_REV) * 360.0;
                drum.setPower(-1.0);
            }

            drum.setPower(0.0);
            count++;
            telemetry.addData("count", count);

            TARGET_DEGREES += 35000;
        }

        //If count is 3 start the chains to shoot.
        //if (count == 3) {
        if (gamepad1.dpad_up){
            turret1.setPower(-1);
            turret2.setPower(1);
        }
        if (gamepad1.dpad_right){
            turret1.setPower(-0.9);
            turret2.setPower(0.9);
        }
        if (gamepad1.dpad_left){
            turret1.setPower(-0.8);
            turret2.setPower(0.8);
        }
        if (gamepad1.dpad_down){
            turret1.setPower(-0.74);
            turret2.setPower(0.74);
        }
        //}

        //Realign the kicker
        kicker.setPower(-0.01);

        //Shoot 3 balls when left bumper is clicked.
        if (gamepad1.left_bumper) {
            shootThreeBalls();
        }

        pattern_real = "GPP";
        if (gamepad1.right_bumper) {
            //If real pattern is PPG
            if (pattern_real != null && pattern_real.equals("PPG")) {
                //if current pattern is PPG
                if (pattern_current.equals("PPG")) {
                    shootThreeBalls();
                }
                //if current pattern is GPP
                if (pattern_current.equals("GPP")) {
                    //Move fwd and then shoot threeballs
                    TARGET_DEGREES = (encoder.getCurrentPosition() / TICKS_PER_REV) * 360.0 + 35000;

                    double degrees = (encoder.getCurrentPosition() / TICKS_PER_REV) * 360.0;
                    while (degrees < TARGET_DEGREES) {
                        degrees = (encoder.getCurrentPosition() / TICKS_PER_REV) * 360.0;
                        telemetry.addData("Degrees", "%.1f", degrees);
                        drum.setPower(-1.0);
                    }
                    drum.setPower(0.0);

                    shootThreeBalls();
                }
                if (pattern_current.equals("PGP")) {
                    //Move reverse and then shoot threeballs
                    TARGET_DEGREES = (encoder.getCurrentPosition() / TICKS_PER_REV) * 360.0 - 37500;

                    double degrees = (encoder.getCurrentPosition() / TICKS_PER_REV) * 360.0;
                    while (degrees > TARGET_DEGREES) {
                        degrees = (encoder.getCurrentPosition() / TICKS_PER_REV) * 360.0;
                        telemetry.addData("Degrees", "%.1f", degrees);
                        drum.setPower(1.0);
                    }
                    drum.setPower(0.0);
                    shootThreeBalls();
                }
            }
            if (pattern_real != null && pattern_real.equals("GPP")) {
                //if current pattern is PPG
                if (pattern_current.equals("GPP")) {
                    shootThreeBalls();
                }
                //if current pattern is GPP
                if (pattern_current.equals("PGP")) {
                    //Move fwd and then shoot threeballs
                    TARGET_DEGREES = (encoder.getCurrentPosition() / TICKS_PER_REV) * 360.0 + 35000;

                    double degrees = (encoder.getCurrentPosition() / TICKS_PER_REV) * 360.0;
                    while (degrees < TARGET_DEGREES) {
                        degrees = (encoder.getCurrentPosition() / TICKS_PER_REV) * 360.0;
                        telemetry.addData("Degrees", "%.1f", degrees);
                        drum.setPower(-1.0);
                    }
                    drum.setPower(0.0);

                    shootThreeBalls();
                }
                if (pattern_current.equals("PPG")) {
                    //Move reverse and then shoot threeballs
                    TARGET_DEGREES = (encoder.getCurrentPosition() / TICKS_PER_REV) * 360.0 - 37500;

                    double degrees = (encoder.getCurrentPosition() / TICKS_PER_REV) * 360.0;
                    while (degrees > TARGET_DEGREES) {
                        degrees = (encoder.getCurrentPosition() / TICKS_PER_REV) * 360.0;
                        telemetry.addData("Degrees", "%.1f", degrees);
                        drum.setPower(1.0);
                    }
                    drum.setPower(0.0);
                    shootThreeBalls();
                }
            }
        }
        double y = -gamepad1.left_stick_y;   // forward / back
        double x = gamepad1.left_stick_x;    // strafe (if mecanum)
        double rx = gamepad1.right_stick_x;  // turn

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

        double fl = (y + x + rx) / denominator;
        double fr = (y - x - rx) / denominator;
        double bl = (y - x + rx) / denominator;
        double br = (y + x - rx) / denominator;

        FL.setPower(fl*0.8);
        FR.setPower(fr*0.8);
        BL.setPower(bl*0.8);
        BR.setPower(br*0.8);

    } // end of loop()

    private void shootThreeBalls() {

        //Shoot first ball
        TARGET_DEGREES = (encoder.getCurrentPosition() / TICKS_PER_REV) * 360.0 + 8750;

        double degrees = (encoder.getCurrentPosition() / TICKS_PER_REV) * 360.0;
        while (degrees < TARGET_DEGREES) {
            degrees = (encoder.getCurrentPosition() / TICKS_PER_REV) * 360.0;
            telemetry.addData("Degrees", "%.1f", degrees);
            drum.setPower(-1.0);
        }
        drum.setPower(0.0);

        runTime.reset();
        while(runTime.milliseconds() < 500) {}

        runTime.reset();
        while(runTime.milliseconds() < 500) {
            kicker.setPower(1);
        }
        runTime.reset();
        while(runTime.milliseconds() < 125) {
            kicker.setPower(-0.5);
        }

        if (gamepad1.dpad_up){
            turret1.setPower(-1);
            turret2.setPower(1);
        }
        if (gamepad1.dpad_right){
            turret1.setPower(-0.85);
            turret2.setPower(0.85);
        }
        if (gamepad1.dpad_left){
            turret1.setPower(-0.85);
            turret2.setPower(0.85);
        }
        if (gamepad1.dpad_down){
            turret1.setPower(-0.74);
            turret2.setPower(0.74);
        }

        //Shoot second ball
        TARGET_DEGREES += 35000;
        degrees = (encoder.getCurrentPosition() / TICKS_PER_REV) * 360.0;
        while (degrees < TARGET_DEGREES) {
            degrees = (encoder.getCurrentPosition() / TICKS_PER_REV) * 360.0;
            drum.setPower(-1.0);
        }
        drum.setPower(0.0);

        runTime.reset();
        while(runTime.milliseconds() < 500) {}

        runTime.reset();
        while(runTime.milliseconds() < 500) {
            kicker.setPower(1);
        }
        runTime.reset();
        while(runTime.milliseconds() < 125) {
            kicker.setPower(-0.5);
        }
        if (gamepad1.dpad_up){
            turret1.setPower(-1);
            turret2.setPower(1);
        }
        if (gamepad1.dpad_right){
            turret1.setPower(-0.85);
            turret2.setPower(0.85);
        }
        if (gamepad1.dpad_left){
            turret1.setPower(-0.88);
            turret2.setPower(0.88);
        }
        if (gamepad1.dpad_down){
            turret1.setPower(-0.74);
            turret2.setPower(0.74);
        }

        //Shoot third ball
        TARGET_DEGREES += 35000;
        degrees = (encoder.getCurrentPosition() / TICKS_PER_REV) * 360.0;
        while (degrees < TARGET_DEGREES) {
            degrees = (encoder.getCurrentPosition() / TICKS_PER_REV) * 360.0;
            drum.setPower(-1.0);
        }
        drum.setPower(0.0);

        runTime.reset();
        while(runTime.milliseconds() < 500) {}

        runTime.reset();
        while(runTime.milliseconds() < 500) {
            kicker.setPower(1);
        }
        runTime.reset();
        while(runTime.milliseconds() < 125) {
            kicker.setPower(-0.5);
        }

        count = 0;
        count--;

        //Get back to Position to intake
        TARGET_DEGREES += 17500;
        degrees = (encoder.getCurrentPosition() / TICKS_PER_REV) * 360.0;
        while (degrees < TARGET_DEGREES) {
            degrees = (encoder.getCurrentPosition() / TICKS_PER_REV) * 360.0;
            drum.setPower(-1.0);
        }
        drum.setPower(0.0);

        //Reset shooter power
        turret1.setPower(0);
        turret2.setPower(0);

    }

    private String detectGreenOrPurple(float r, float g, float b) {
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

} // end of class