package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import java.util.List;

@TeleOp
public class TeleOpScrimmageV1 extends OpMode {
    DcMotor FL, FR, BL, BR;
    private Servo rgbmid;
    private Servo rgbmid2;
    private Servo rgbmid3;
    private Servo drum;
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
    int count = -1;
    final double TICKS_PER_REV = 28.0;
    double position = 0.0;

    private Servo kicker;
    private String detectedColor = null;
    private String detectedColor2 = null;
    private String detectedColor3 = null;
    private String detectedColor4 = null;

    @Override
    public void init() {

        drum = hardwareMap.get(Servo.class, "drum");

        telemetry.addData("Status", "Initialized");
        FL = hardwareMap.dcMotor.get("FL");
        FR = hardwareMap.dcMotor.get("FR");
        BL = hardwareMap.dcMotor.get("BL");
        BR = hardwareMap.dcMotor.get("BR");
        rgbmid = hardwareMap.get(Servo.class, "rgbmid");
        rgbmid2 = hardwareMap.get(Servo.class, "rgbmid2");
        rgbmid3 = hardwareMap.get(Servo.class, "rgbmid3");
        turret1 = hardwareMap.dcMotor.get("turret1");
        turret2 = hardwareMap.dcMotor.get("turret2");
        intake = hardwareMap.dcMotor.get("intake");
        kicker = hardwareMap.get(Servo.class, "kicker");
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

        drum.setPosition(0);

    }

    @Override
    public void loop() {

        //Get Led colors and set led
        NormalizedRGBA colors = front.getNormalizedColors();
        NormalizedRGBA colors2 = left.getNormalizedColors();
        NormalizedRGBA colors3 = right.getNormalizedColors();
        NormalizedRGBA colors4 = bottom.getNormalizedColors();

        detectedColor = detectGreenOrPurple(colors); //front
        telemetry.addData("Detected Color front: ", detectedColor);
        setLed(detectedColor, rgbmid);

        detectedColor2 = detectGreenOrPurple(colors2); //left
        telemetry.addData("Detected Color left: ", detectedColor);
        setLed(detectedColor2, rgbmid2);

        detectedColor3 = detectGreenOrPurple(colors3); //right
        telemetry.addData("Detected Color right: ", detectedColor3);
        setLed(detectedColor3, rgbmid3);

        detectedColor4 = detectGreenOrPurple(colors4); //bottom
        telemetry.addData("Detected Color Bottom: ", detectedColor4);


        //Get the Limelight tags and compute the actual pattern and detected pattern
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

        pattern_current = detectedColor+detectedColor3+detectedColor2;
        telemetry.addData("Current Pattern", pattern_current);

        setManualControls();
        intake();


        setShooterSpeed(1, 0.9, 0.8);

        //Realign the kicker
        kicker.setPosition(0.01);

        //Shoot 3 balls when left bumper is clicked.
        if (gamepad1.left_bumper) {
            shootThreeBalls(0.2, 0.6, 0.99);
        }

        if (gamepad1.right_bumper) {
            shootThreeBallsSorted();
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


    private void setManualControls() {
        //Manual setting the shooter speed
        if (gamepad1.y) {
            turret1.setPower(1.0);
            turret2.setPower(-1.0);
        }

        //Manual reset the shooter speed
        if (gamepad1.x) {
            turret1.setPower(0);
            turret2.setPower(0);
        }

        //Manual ball spit out
        if (gamepad1.b) {
            runTime.reset();
            while(runTime.milliseconds() < 750) {
                intake.setPower(-1);
            }
        }
    }

    private void setLed(String detectedColor, Servo rgb) {
        if (detectedColor.equalsIgnoreCase("P")) {
            pos = 0.71;//purple
            rgb.setPosition(pos);
        } else if (detectedColor.equalsIgnoreCase("G")) {
            pos = 0.5;//green
            rgb.setPosition(pos);
        } else{
            rgb.setPosition(0);
        }
    }

    private void setShooterSpeed(double high, double mid, double low) {
        if (gamepad1.dpad_up){
            turret1.setPower(high);
            turret2.setPower(-high);
        }
        if (gamepad1.dpad_right){
            turret1.setPower(mid);
            turret2.setPower(-mid);
        }
        if (gamepad1.dpad_left){
            turret1.setPower(mid);
            turret2.setPower(-mid);
        }
        if (gamepad1.dpad_down){
            turret1.setPower(low);
            turret2.setPower(-low);
        }
    }

    private void intake() {
        intake.setPower(1);
        telemetry.addData("Drum position", drum.getPosition());
        telemetry.addData("count", count);
        //Rotate the drum while the count is < 3 and there is a ball sucked in and we are not shooting
        if ((count < 3) && (detectedColor.equalsIgnoreCase("P") || detectedColor.equalsIgnoreCase("G")) && (!gamepad1.left_bumper || !gamepad1.right_bumper)) {
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

    private void shootThreeBallsSorted() {
        //If real pattern is PPG
        pattern_real = "PPG";
        if (pattern_real != null && pattern_real.equals("PPG")) {
            //if current pattern is PPG
            if (pattern_current.equals(pattern_real)) {
                shootThreeBalls(0.2, 0.6, 1);
            }
            else if (pattern_current.equals("PGP")) {
                shootThreeBalls(0.2, 1, 0.6);
            }
            else if (pattern_current.equals("GPP")) {
                shootThreeBalls(0.6, 1, 0.2);
            }
        }
    }

    private void shootThreeBalls(double firstPos, double secondPos, double thirdPos) {

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

    private String detectGreenOrPurple(NormalizedRGBA colors) {

        float r = colors.red / colors.alpha;
        float g = colors.green / colors.alpha;
        float b = colors.blue / colors.alpha;

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