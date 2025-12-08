package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

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

import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp
public class DC extends OpMode {
    //Wheels : FRONT LEFT, FRONT RIGHT,BACK LEFT, BACK RIGHT
    DcMotor FL, FR, BL, BR;
    private Servo drum;
    private DcMotorEx turret1;
    private DcMotorEx turret2;
    private DcMotor intake;
    private DcMotor encoder;
    double total;
    double pos;

    //RGB Lights
    private Servo rgbmid;
    private Servo rgbmid2;
    private Servo rgbmid3;
    private Servo rgbmid4;

    //COLOR SENSORS STARTS
    NormalizedColorSensor front;
    NormalizedColorSensor left;
    NormalizedColorSensor right;
    NormalizedColorSensor bottom;
    //COLOR SENSORS ENDS

    String pattern_real = null; // The Expected Pattern
    String pattern_current = null;//The Actual Pattern robot detects

    //PID TUNING VALUES for motor speed
    public static double Kpm = 0.0005;
    public static double Kim = 0.0;
    public static double Kdm = 0.0;
    public static double Kfm = 0.00034;

    public static double highPower = 0.95;
    public static double midPower = 0.7;
    public static double lowPower = 0.6;
    private double expectedPower = 0.0;

    public static double INTEGRAL_MOTOR_MAX = 0.5;
    private double integralSumMotor = 0;
    private double lastErrorMotor = 0;
    // Target ticks/sec for your shooter motor
    // Anti-windup clamp
    public static double LOOP_PERIOD = 0.05;
    //PID TUNING VALUES ENDS

    ElapsedTime timer = new ElapsedTime();
    //private FtcDashboard dashboard;

    private Limelight3A limelight;
    volatile double sharedTx = 0;
    volatile Position sharedPos = null;
    private double TARGET_DEGREES = 25000.0; //first time the encoder is slow
    ElapsedTime runTime = new ElapsedTime();
    volatile int count = 0;
    final double TICKS_PER_REV = 28.0;
    double position = 0.0;

    private Servo kicker;
    private String detectedColor = null;
    private String detectedColor2 = null;
    private String detectedColor3 = null;
    private String detectedColor4 = null;
    boolean cr = false;
    boolean cb = false;
    boolean perp = false;
    boolean headlighton = false;
    double txr = 0;
    int time = 100;
    boolean alignOn = false;
    double error = 1.0;


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
        rgbmid4 = hardwareMap.get(Servo.class, "rgbmid4");
        turret1 = hardwareMap.get(DcMotorEx.class, "turret1");
        //turret2 = hardwareMap.get(DcMotorEx.class, "right_front_drive");

        turret1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turret2 = hardwareMap.get(DcMotorEx.class, "turret2");
        //turret2 = hardwareMap.get(DcMotorEx.class, "right_front_drive");

        turret2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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

        if (pattern_real != null && (pattern_real.equals("PPG") || pattern_real.equals("PGP") || pattern_real.equals("GPP"))) {
            setLed("R", rgbmid4);
        }

        pattern_current = detectedColor+detectedColor3+detectedColor2;
        telemetry.addData("Current Pattern", pattern_current);

        setManualControls();
        intake();


        setShooterSpeed();

        //Realign the kicker
        kicker.setPosition(0.01);

        //Shoot 3 balls when left bumper is clicked.
        if (gamepad1.dpad_up || gamepad1.dpad_left || gamepad1.dpad_right || gamepad1.dpad_right) {
            shootThreeBalls(0.2, 0.6, 0.99);
        }

        if (gamepad1.left_bumper || gamepad1.right_bumper) {
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
        //Manual setting drum back to 0
        if (gamepad1.x) {
            drum.setPosition(0);
            runTime.reset();
            while(runTime.milliseconds() < 125) {}
        }

        //Manual ball spit out
        if (gamepad1.b) {
            runTime.reset();
            while(runTime.milliseconds() < 750) {
                intake.setPower(-1);
            }
        }
        if (gamepad1.a){
            autoalign();
        }
    }
    private void autoalign() {
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
                            Right(txr, power);
                            runTime.reset();
                            while(runTime.milliseconds() < time) {}
                            afterEnd();
                        } else if (txr < (-1 * error) ){
                            Left(txr, power);
                            runTime.reset();
                            while(runTime.milliseconds() < time) {}
                            afterEnd();
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
                            Left(txr, power);
                            runTime.reset();
                            while(runTime.milliseconds() < time) {}
                            afterEnd();

                        } else if (txr > (-1.0 * error) ){
                            Right(txr, power);
                            runTime.reset();
                            while(runTime.milliseconds() < time) {}
                            afterEnd();
                        }
                    }
                }
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
        } else if (detectedColor.equalsIgnoreCase("R")) {
            pos = 0.2;
            rgb.setPosition(pos);
        }
        else{
            rgb.setPosition(0);
        }
    }


    private void setShooterSpeed() {
        if (gamepad1.left_bumper) {
            turret1.setPower(highPower);
            turret2.setPower(-highPower);
            expectedPower = highPower;
        }
        if (gamepad1.right_bumper) {
            turret1.setPower(lowPower);
            turret2.setPower(-lowPower);
            expectedPower = lowPower;
        }
        if (gamepad1.dpad_up){
            turret1.setPower(highPower);
            turret2.setPower(-highPower);
            expectedPower = highPower;
        }
        if (gamepad1.dpad_right){
            turret1.setPower(midPower);
            turret2.setPower(-midPower);
            expectedPower = midPower;
        }
        if (gamepad1.dpad_left){
            turret1.setPower(midPower);
            turret2.setPower(-midPower);
            expectedPower = midPower;
        }
        if (gamepad1.dpad_down){
            turret1.setPower(lowPower);
            turret2.setPower(-lowPower);
            expectedPower = lowPower;
        }
    }

    // private void setShooterSpeedLowOverride() {
    //     if (gamepad1.dpad_up){
    //         turret1.setPower(highPower-0.1);
    //         turret2.setPower(-highPower-0.1);
    //         expectedPower = highPower-0.1;
    //     }
    //     if (gamepad1.left_bumper) {
    //         turret1.setPower(highPower);
    //         turret2.setPower(-highPower);
    //         expectedPower = highPower;
    //     }
    //     if (gamepad1.right_bumper) {
    //         turret1.setPower(lowPower);
    //         turret2.setPower(-lowPower);
    //         expectedPower = lowPower;
    //     }
    //     if (gamepad1.dpad_right){
    //         turret1.setPower(midPower-0.1);
    //         turret2.setPower(-midPower-0.1);
    //         expectedPower = midPower-0.1;
    //     }
    //     if (gamepad1.dpad_left){
    //         turret1.setPower(midPower-0.1);
    //         turret2.setPower(-midPower-0.1);
    //         expectedPower = midPower-0.1;
    //     }
    //     if (gamepad1.dpad_down){
    //         turret1.setPower(lowPower-0.1);
    //         turret2.setPower(-lowPower-0.1);
    //         expectedPower = lowPower-0.1;
    //     }
    // }

    private void Right(double txr, double power) {
        telemetry.addData("tx", txr);
        telemetry.addData("power", power);
        FL.setPower(power);
        FR.setPower(-power);
        BL.setPower(power);
        BR.setPower(-power);
    }



    private void Left(double txr, double power) {
        telemetry.addData("tx", txr);
        telemetry.addData("power", power);
        FL.setPower(-power);
        FR.setPower(power);
        BL.setPower(-power);
        BR.setPower(power);
    }

    private void afterEnd() {
        FL.setPower(0);
        FR.setPower(0);
        BL.setPower(0);
        BR.setPower(0);
    }
    // private void intake() {
    //     intake.setPower(1);
    //     telemetry.addData("Drum position", drum.getPosition());
    //     telemetry.addData("count", count);
    //     //Rotate the drum while the count is < 3 and there is a ball sucked in and we are not shooting
    //     if ((count < 3) && (detectedColor.equalsIgnoreCase("P") || detectedColor.equalsIgnoreCase("G")) && (!gamepad1.left_bumper || !gamepad1.right_bumper)) {

    //         telemetry.addData("count", count);
    //         telemetry.update();
    //         if (count == 0) {
    //             position = position + 0.2;
    //             drum.setPosition(position);
    //         } else if (count == 1) {
    //             position = position + 0.42;
    //             drum.setPosition(position);
    //         }
    //         telemetry.addData("Drum position", drum.getPosition());

    //         runTime.reset();
    //         while(runTime.milliseconds() < 125) {}
    //         count = count + 1;
    //     }
    // }
    private void intake() {
        intake.setPower(1);
        telemetry.addData("Drum position", drum.getPosition());
        //Rotate the drum while we are not shooting
        if (!(gamepad1.dpad_up
                && gamepad1.dpad_left
                && gamepad1.dpad_right
                && gamepad1.dpad_down
                && gamepad1.left_bumper
                && gamepad1.right_bumper)) {
            //only one ball detected
            if ((detectedColor.equals("P") || detectedColor.equals("G"))
                    && detectedColor2.equals("U") && detectedColor3.equals("U")) {
                position = 0.4;
                drum.setPosition(position);
                runTime.reset();
                while(runTime.milliseconds() < 125) {}
            }

            //two balls detected front and left and right is unknown.
            if ((detectedColor.equals("P") || detectedColor.equals("G"))
                    && (detectedColor2.equals("P") || detectedColor2.equals("G"))
                    && detectedColor3.equals("U")) {
                position = 0.82;
                drum.setPosition(position);
                runTime.reset();
                while(runTime.milliseconds() < 125) {}
            }

            //two balls detected front and right and left is unknown.
            if ((detectedColor.equals("P") || detectedColor.equals("G"))
                    && (detectedColor3.equals("P") || detectedColor3.equals("G"))
                    && detectedColor2.equals("U")) {
                position = 0.2;
                drum.setPosition(position);
                runTime.reset();
                while(runTime.milliseconds() < 125) {}
            }

            //If all three balls are in, dont have to rotate the drum
            telemetry.addData("Drum position", drum.getPosition());
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

    private void shootThreeBalls(double firstPos, double secondPos, double thirdPos) {
        timer.reset();
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
        setShooterSpeed();
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
        setShooterSpeed();
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
} // end of class
