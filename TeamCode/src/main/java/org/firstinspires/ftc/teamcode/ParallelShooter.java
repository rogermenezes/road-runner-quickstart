package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

import java.util.List;

public class ParallelShooter {

    public Servo drum;
    private DcMotorEx turret1;
    private DcMotorEx turret2;
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

    String pattern_real = null; // The Expected Pattern
    String pattern_current = null;//The Actual Pattern robot detects

    //PID TUNING VALUES for motor speed
    public static double Kpm = 0.0005;
    public static double Kim = 0.0;
    public static double Kdm = 0.0;
    public static double Kfm = 0.00034;

    public static double highPower = 0.88;
    public static double midPower = 0.7;
    public static double lowPower = 0.6;
    private double expectedPower = highPower;

    public static double INTEGRAL_MOTOR_MAX = 0.5;
    private double integralSumMotor = 0;
    private double lastErrorMotor = 0;
    // Target ticks/sec for your shooter motor
    // Anti-windup clamp
    public static double LOOP_PERIOD = 0.05;
    //PID TUNING VALUES ENDS

    ElapsedTime timer = new ElapsedTime();

    Telemetry telemetry;

    private String detectedColor = null;
    private String detectedColor2 = null;
    private String detectedColor3 = null;
    private String detectedColor4 = null;

    private Servo rgbmid;
    private Servo rgbmid2;
    private Servo rgbmid3;
    private Servo rgbmid4;

    double pos;

    private boolean drumMoving = false;
    private double nextDrumPosition = 0.0;

    //auto-align code
    boolean cr = false;
    boolean cb = false;

    private Limelight3A limelight;

    boolean headlighton = false;
    double txr = 0;
    int time = 100;


    MecanumDrive drive;

    double error = 1.0;


    public ParallelShooter(HardwareMap hardwareMap, Telemetry telemetry, MecanumDrive drive) {
        drum = hardwareMap.get(Servo.class, "drum");
        this.telemetry = telemetry;
        //drum.setPosition(0);
        this.drive = drive;

        telemetry.addData("Status", "Initialized");

        turret1 = hardwareMap.get(DcMotorEx.class, "turret1");
        turret2 = hardwareMap.get(DcMotorEx.class, "turret2");
        encoder = hardwareMap.get(DcMotor.class, "encoder");


        turret1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turret2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

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

        intake = hardwareMap.dcMotor.get("intake");

        encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rgbmid = hardwareMap.get(Servo.class, "rgbmid");
        rgbmid2 = hardwareMap.get(Servo.class, "rgbmid2");
        rgbmid3 = hardwareMap.get(Servo.class, "rgbmid3");
        rgbmid4 = hardwareMap.get(Servo.class, "rgbmid4");

        limelight = hardwareMap.get(Limelight3A.class, "Limelight30735");
        limelight.pipelineSwitch(0);
        limelight.start();


        drum.setPosition(0);

    }

    public void intakeOn() { intake.setPower(1); }
    public void intakeOff() { intake.setPower(0); }

    public void updateIntake(Telemetry telemetry) {
        if (count >= 3) return;

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

        if ((detectedColor.equals("P") || detectedColor.equals("G"))
                && detectedColor2.equals("U") && detectedColor3.equals("U")) {
            position = 0.4;
            drum.setPosition(position);
            runTime.reset();
            count = 1;
            while(runTime.milliseconds() < 125) {}
        }

        if ((detectedColor.equals("P") || detectedColor.equals("G"))
                && (detectedColor2.equals("P") || detectedColor2.equals("G"))
                && detectedColor3.equals("U")) {
            position = 0.82;
            drum.setPosition(position);
            runTime.reset();
            count = 2;
            while(runTime.milliseconds() < 125) {}
        }

        if ((detectedColor.equals("P") || detectedColor.equals("G"))
                && (detectedColor3.equals("P") || detectedColor3.equals("G"))
                && detectedColor2.equals("U")) {
            position = 0.2;
            drum.setPosition(position);
            runTime.reset();
            count = 3;
            while(runTime.milliseconds() < 125) {}
        }

    }

    public void updateIntakeV2(Telemetry telemetry) {

        telemetry.addData("inside update intake v2", 20);
        telemetry.update();
        // 1) If we're in the middle of a drum move, just wait for 125 ms to elapse
        if (drumMoving) {
            if (runTime.milliseconds() >= 125) {
                drumMoving = false;  // done waiting, allow new detection
            }
            return; // do nothing else this tick
        }

        // 2) If we already have 3 balls, stop
        if (count >= 3) return;

        // 3) Read sensors
        NormalizedRGBA colors  = front.getNormalizedColors();
        NormalizedRGBA colors2 = left.getNormalizedColors();
        NormalizedRGBA colors3 = right.getNormalizedColors();
        NormalizedRGBA colors4 = bottom.getNormalizedColors();

        detectedColor  = detectGreenOrPurple(colors);   // front
        detectedColor2 = detectGreenOrPurple(colors2);  // left
        detectedColor3 = detectGreenOrPurple(colors3);  // right
        detectedColor4 = detectGreenOrPurple(colors4);  // bottom

        if (telemetry != null) {
            telemetry.addData("Detected front",  detectedColor);
            telemetry.addData("Detected left",   detectedColor2);
            telemetry.addData("Detected right",  detectedColor3);
            telemetry.addData("Detected bottom", detectedColor4);
        }

        // 4) Decide what to do, and start a *non-blocking* 125 ms move

        // Case 1: only front sees a ball → 1st ball
        if ((detectedColor.equals("P") || detectedColor.equals("G"))
                && detectedColor2.equals("U") && detectedColor3.equals("U")) {

            position = 0.4;
            drum.setPosition(position);
            count = 1;

            drumMoving = true;
            runTime.reset();
            while(runTime.milliseconds() < 125) {}
            telemetry.addData("count ball:", count);
            telemetry.update();
            return;
        }

        // Case 2: front + left → 2nd ball
        if ((detectedColor.equals("P") || detectedColor.equals("G"))
                && (detectedColor2.equals("P") || detectedColor2.equals("G"))
                && detectedColor3.equals("U")) {

            position = 0.82;
            drum.setPosition(position);
            count = 2;

            drumMoving = true;
            runTime.reset();
            return;
        }

        // Case 3: front + right → 3rd ball
        if ((detectedColor.equals("P") || detectedColor.equals("G"))
                && (detectedColor3.equals("P") || detectedColor3.equals("G"))
                && detectedColor2.equals("U")) {

            position = 0.2;
            drum.setPosition(position);
            count = 3;

            telemetry.addData("count ball:", count);
            telemetry.update();

            drumMoving = true;
            runTime.reset();
            return;
        }
    }


    private void setShooterSpeed(double power) {
        //TODO: we need some logic on when to use one of high, mid or low
        telemetry.addData("expectedPower", expectedPower);
        turret1.setPower(power);
        turret2.setPower(-power);
    }


    public void shootThreeBallsV2(double firstPos, double secondPos, double thirdPos) {
        timer.reset();
        integralSumMotor = 0.0;
        lastErrorMotor = 0.0;

        //Shoot first ball
        setShooterSpeed(0.95);
        position = firstPos;
        drum.setPosition(position);
        runTime.reset();
        while(runTime.milliseconds() < 1300) {
            telemetry.addData("Here0", 1);
            telemetry.update();
            //pid_speed_motor();
        }

        kicker.setPosition(0.5);

        runTime.reset();
        while(runTime.milliseconds() < 125) {}
        kicker.setPosition(0.01);


        //Shoot second ball
        setShooterSpeed(0.9);
        position = secondPos;
        drum.setPosition(position);

        runTime.reset();
        while(runTime.milliseconds() < 800) {
            telemetry.addData("Here1", 1);
            telemetry.update();
            //pid_speed_motor();
        }
        timer.reset();

        kicker.setPosition(0.5);
        runTime.reset();
        while(runTime.milliseconds() < 125) {}
        kicker.setPosition(0.01);


        //Shoot third ball
        setShooterSpeed(0.88);
        position = thirdPos;
        drum.setPosition(position);

        runTime.reset();
        while(runTime.milliseconds() < 800) {
            telemetry.addData("Here2", 1);
            telemetry.update();
            //pid_speed_motor();
        }
        timer.reset();

        kicker.setPosition(0.5);
        runTime.reset();
        while(runTime.milliseconds() < 125) {}
        kicker.setPosition(0.01);

        count = 0;

        //Move back to 0
        position = 0;
        drum.setPosition(position);

        runTime.reset();
        while(runTime.milliseconds() < 125) {}

        //Reset shooter power
        turret1.setPower(0);
        turret2.setPower(0);
    }



    public void shootThreeBalls(double firstPos, double secondPos, double thirdPos)  {
        timer.reset();
        kicker.setPosition(0.01);

        setShooterSpeed(0.9);
        integralSumMotor = 0.0;
        lastErrorMotor = 0.0;

        //Shoot first ball
        position = firstPos;
        drum.setPosition(position);
        runTime.reset();
        while(runTime.milliseconds() < 1300) {
            telemetry.addData("Here0", 1);
            telemetry.update();
            pid_speed_motor();
        }

        kicker.setPosition(0.5);

        runTime.reset();
        while(runTime.milliseconds() < 125) {}
        kicker.setPosition(0.01);


        //Shoot second ball
        setShooterSpeed(0.9);
        position = secondPos;
        drum.setPosition(position);

        runTime.reset();
        while(runTime.milliseconds() < 500) {
            telemetry.addData("Here1", 1);
            telemetry.update();
            pid_speed_motor();
        }
        timer.reset();

        kicker.setPosition(0.5);
        runTime.reset();
        while(runTime.milliseconds() < 125) {}
        kicker.setPosition(0.01);


        //Shoot third ball
        setShooterSpeed(0.88);
        position = thirdPos;
        drum.setPosition(position);

        runTime.reset();
        while(runTime.milliseconds() < 500) {
            telemetry.addData("Here2", 1);
            telemetry.update();
            pid_speed_motor();
        }
        timer.reset();

        kicker.setPosition(1);
        runTime.reset();
        while(runTime.milliseconds() < 400) {}
        kicker.setPosition(0.01);

        count = 0;

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


    private void pid_speed_motor() {
        double TARGET_VELOCITY = expectedPower*5400;
        double dt = timer.seconds();
        if (dt >= LOOP_PERIOD) {
            double currentVelocity = turret1.getVelocity();
            double currentVelocity2 = turret2.getVelocity();
            double power = pidControlForMotorSpeed(TARGET_VELOCITY, currentVelocity);
            double power2 = pidControlForMotorSpeed(TARGET_VELOCITY, currentVelocity2);
            telemetry.addData("power", power);
            telemetry.addData("power2", power2);
            telemetry.update();

            turret1.setPower(power);
            turret2.setPower(-power2);
            timer.reset();
        }
    }

    public double pidControlForMotorSpeed(double reference, double state) {
        double dt = timer.seconds();
        if (dt == 0) dt = 1e-3;

        double error = reference - state;

        // Integral accumulation with clamp

        integralSumMotor +=error * dt;
        if (integralSumMotor > INTEGRAL_MOTOR_MAX) integralSumMotor = INTEGRAL_MOTOR_MAX;
        if (integralSumMotor < -INTEGRAL_MOTOR_MAX) integralSumMotor = -INTEGRAL_MOTOR_MAX;

        // Derivative term
        double derivative = (error - lastErrorMotor) / dt;
        lastErrorMotor = error;

        timer.reset();

        // PIDF output
        return (error * Kpm) + (derivative * Kdm) + (integralSumMotor * Kim) + (reference * Kfm);
    }


    private void setLed(String detectedColor, Servo rgb) {
        if (detectedColor.equalsIgnoreCase("P")) {
            pos = 0.71;//purple
            rgb.setPosition(pos);
        } else if (detectedColor.equalsIgnoreCase("G")) {
            pos = 0.5;//green
            rgb.setPosition(pos);
        } else if (detectedColor.equalsIgnoreCase("R")) {
            pos = 0.2;
            rgb.setPosition(pos);
        }
        else{
            rgb.setPosition(0);
        }
    }

    public void autoalign() {
        cr = false;
        cb = false;


        LLResult result = limelight.getLatestResult();
        telemetry.addLine("hi, limelight sees");
        telemetry.update();


        if (result != null){
            headlighton = true;
            //headlight.setPosition(1);
            if (result.isValid()){
                headlighton = true;
                telemetry.addLine("hi, limelight is seeing a apriltag");
                telemetry.update();
                telemetry.addLine("hi, limelight is seeing a apriltag");
                double tx = result.getTx();

                List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();

                for (LLResultTypes.FiducialResult tag : tags) {
                    telemetry.addLine("plz work now tag idea. string = false");
                    telemetry.update();
                    Pose3D pose = tag.getTargetPoseCameraSpace();
                    Position pos = pose.getPosition();

                    //perp = false;
                    switch ((int)tag.getFiducialId()) {
                        //make the cb = true when you wnat to c blue
                        case 20:
                        {telemetry.addData("Team", "Blue Side");
                            cb = true;
                            txr = result.getTx();
                            break;}
                        case 24: {
                            telemetry.addData("Team", "Red Side");
                            cr = true;
                            txr = result.getTx();
                            break;}
                    }
                }

                if (cr == true){
                    result = limelight.getLatestResult();

                    if (result != null && result.isValid()) {

                        telemetry.update();
                        double txr = result.getTx();

                        double power = 0.5;
                        time = (int)(Math.abs(txr * 5));
                        if (txr >  (error) ){
                            drive.Right(power);
                            runTime.reset();
                            while(runTime.milliseconds() < time) {}
                            drive.afterEnd();
                        } else if (txr < (-1 * error) ){
                            drive.Left(power);
                            runTime.reset();
                            while(runTime.milliseconds() < time) {}
                            drive.afterEnd();
                        }
                    }
                }
                if (cb == true)    {
                    telemetry.update();

                    double txr = result.getTx() + 2;             ;

                    if (result != null && result.isValid()) {

                        double power = 0.5;
                        time = (int)(Math.abs(txr * 5));
                        if (txr < (-1 * error)){
                            drive.Left(power);
                            runTime.reset();
                            while(runTime.milliseconds() < time) {}
                            drive.afterEnd();

                        } else if (txr > (-1.0 * error) ){
                            drive.Right(power);
                            runTime.reset();
                            while(runTime.milliseconds() < time) {}
                            drive.afterEnd();
                        }
                    }
                }
            }
        }
    }

    private void shootThreeBallsSorted() {
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
        if (pattern_real != null && pattern_real.equals("PGP")) {
            //if current pattern is PGP
            if (pattern_current.equals(pattern_real)) {
                shootThreeBalls(0.2, 0.6, 1);
            }
            else if (pattern_current.equals("PPG")) {
                shootThreeBalls(0.2, 1, 0.6);
            }
            else if (pattern_current.equals("GPP")) {
                shootThreeBalls(0.6, 1, 0.2);
            }
        }
        if (pattern_real != null && pattern_real.equals("GPP")) {
            //if current pattern is GPP
            if (pattern_current.equals(pattern_real)) {
                shootThreeBalls(0.2, 0.6, 1);
            }
            else if (pattern_current.equals("PGP")) {
                shootThreeBalls(0.6, 0.2, 1);
            }
            else if (pattern_current.equals("PPG")) {
                shootThreeBalls(0.6, 1, 0.2);
            }
        }
    }

}

