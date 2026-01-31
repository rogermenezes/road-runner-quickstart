package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import java.util.List;

@TeleOp
public class HighThruPut extends OpMode {

    ElapsedTime runTime = new ElapsedTime();

    // Drive motors
    DcMotor FL, FR, BL, BR;

    // Intake + turret
    DcMotor intake, intake2;
    DcMotorEx turret;
    DcMotorEx turret2;

    // Tongue and Turret Rotator
    Servo tongue;
    Servo turretrot;

    //Color sensor
    NormalizedColorSensor front;

    //Limelight
    Limelight3A limelight;

    //tongueClosed
    boolean tongueClosed = true;

    //startShooting
    boolean startShooting = false;

    //constants camera height and tag height
    double cameraHeight = 13; // inches
    double tagHeight    = 29;  // meters
    int count = 0;
    double distance = -1.0;

    // ===== PID / RPM Constants =====
    static final double TICKS_PER_REV = 28.0;
    static final double MAX_RPM = 6000.0;
    static final double INITIAL_RPM = 3500.0;
    double targetRPM = INITIAL_RPM; // INITIAL SPEED

    @Override
    public void init() {

        // Motors
        FL = hardwareMap.dcMotor.get("FL");
        FR = hardwareMap.dcMotor.get("FR");
        BL = hardwareMap.dcMotor.get("BL");
        BR = hardwareMap.dcMotor.get("BR");

        turret  = hardwareMap.get(DcMotorEx.class, "turret");
        turret2 = hardwareMap.get(DcMotorEx.class, "test");

        intake = hardwareMap.dcMotor.get("intake");
        intake2 = hardwareMap.dcMotor.get("intake2");

        tongue = hardwareMap.servo.get("tounge");

        // Reverse left side
        FL.setDirection(DcMotor.Direction.REVERSE);
        BL.setDirection(DcMotor.Direction.REVERSE);

        turret2.setDirection(DcMotor.Direction.REVERSE);

        tongue.setPosition(0); //tongue closed

        //start limelight
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

        //set the turret rotator at mid point.
        turretrot = hardwareMap.get(Servo.class, "turretrot");
        turretrot.setPosition(0.46);

        //color sensor
        front = hardwareMap.get(NormalizedColorSensor.class, "front");
        front.setGain(20);

        // Reset encoders and enable PID velocity
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turret2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Update telemetry
        telemetry.setMsTransmissionInterval(30);
        telemetry.update();

        runTime.reset();
    }

    @Override
    public void loop() {
        count++;
        //Move around based on left and right joysticks.
        moveAround();

        //Manual controls
        manualControls();

        //intake
        intake();

        //limelight auto adjustment every 250 loops once.
        if (count % 250 == 0) {
            moveAround();
            telemetry.addData("count", count);
            distance = limelightAdjustment();
            telemetry.addData("Current Distance", distance);
        }

        //pid adjustment
        pidAdjustment();
    }

    public void manualControls() {
        if (gamepad1.dpad_up) {
            shootThreeBalls();
        }

        //Close the tongue
        if (gamepad1.y) {
            tongue.setPosition(0);
        }
        //Open the tongue
        if (gamepad1.y) {
            tongue.setPosition(1);
        }
        //Reset the turret to middle
        if (gamepad1.a) {
            turretrot.setPosition(0.46);
        }
        //spit the balls out
        if (gamepad1.b) {
            intake2.setPower(-1);
        }
    }

    public void shootThreeBalls() {
        startShooting = true; //It would have started earlier anyway.Just a check in case color sensor fails.
        distance = limelightAdjustment();
        pidAdjustment();

        //start the tertiary
        tongue.setPosition(1); //open the tongue
        tongueClosed = false;

        //Start the intake system
        intake.setPower(-1);
        intake2.setPower(1);

        runTime.reset();
        while(runTime.milliseconds() < 2000) {} //2 seconds to shoot three balls
        tongue.setPosition(0); //close the tongue
    }

    //Adjust the angle and return the distance.
    private double limelightAdjustment() {
        //get limelight results.
        LLResult result = limelight.getLatestResult();

        if (result != null) {
            List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
            for (LLResultTypes.FiducialResult tag : tags) {

                if (tag.getFiducialId() == 24 || tag.getFiducialId() == 20) {
                    //get the correction.
                    double correction = (result.getTx()  * 0.92)/(double)360;
                    telemetry.addData("Correction", correction);
                    telemetry.addData("Current Position", turretrot.getPosition());
                    //set the position to rotate the turret
                    double turretPosition = turretrot.getPosition();
                    turretrot.setPosition(turretPosition + correction);
                    break;
                }
            }
        }

        //Now get the distance.
        result = limelight.getLatestResult();
        if (result != null) {
            List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
            for (LLResultTypes.FiducialResult tag : tags) {

                if (tag.getFiducialId() == 24 || tag.getFiducialId() == 20) {
                    //get the distance
                    double ty = result.getTy(); // degrees from Limelight
                    double distanceInches = (tagHeight - cameraHeight) /
                            Math.tan(Math.toRadians(ty));
                    return distanceInches;
                }
            }
        }

        return -1;
    }

    private void pidAdjustment() {
        if (startShooting) {
            if (targetRPM > MAX_RPM) targetRPM = MAX_RPM;
            if (targetRPM < 0) targetRPM = 0;

            telemetry.addData("Current Distance", distance);
            //based on the distance adjust the targetRPM
            if (distance == -1) {
                targetRPM = INITIAL_RPM;
            }
            else if (distance > 0 && distance < 68) {
                targetRPM = INITIAL_RPM;
            } else {
                targetRPM = MAX_RPM;
            }

            telemetry.addData("Current RPM", targetRPM);
            /*=====================
            APPLY PID VELOCITY
            ===================== */
            double targetTicksPerSec = (targetRPM * TICKS_PER_REV) / 60.0;

            turret.setVelocity(-targetTicksPerSec);
            turret2.setVelocity(-targetTicksPerSec);
        }
    }

    public void intake() {
        NormalizedRGBA colors = front.getNormalizedColors();
        String detectedColor = detectGreenOrPurple(colors); //front
        telemetry.addData("Detected Color", detectedColor);

        //If one ball is fully in stop the tertiary intake and warm up the shooter
        if ((detectedColor.equalsIgnoreCase("P") || detectedColor.equalsIgnoreCase("G")) && tongueClosed)  {
            //stop the tertiary intake
            intake.setPower(0);
            //warm up the shooter
            startShooting = true;
        } else {
            //Start the intake system
            intake.setPower(-1);
            intake2.setPower(1);
        }
    }

    private String detectGreenOrPurple(NormalizedRGBA colors) {

        float r = colors.red / colors.alpha;
        float g = colors.green / colors.alpha;
        float b = colors.blue / colors.alpha;

        float sum = r + g + b;

        double nr = (double) r / sum;
        double ng = (double) g / sum;
        double nb = (double) b / sum;
        // telemetry.addData("nr", nr);
        // telemetry.addData("ng", ng);
        // telemetry.addData("nb", nb);

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


    public void moveAround() {
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
    }

}
