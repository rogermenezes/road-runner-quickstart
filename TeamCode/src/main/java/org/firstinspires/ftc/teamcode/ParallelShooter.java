package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ParallelShooter {

    private Servo drum;
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

    Telemetry telemetry;


    public ParallelShooter(HardwareMap hardwareMap, Telemetry telemetry) {
        drum = hardwareMap.get(Servo.class, "drum");
        this.telemetry = telemetry;
        //drum.setPosition(0);

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

        drum.setPosition(0);

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

    private void setShooterSpeed() {
        //TODO: we need some logic on when to use one of high, mid or low
        turret1.setPower(highPower);
        turret2.setPower(-highPower);
    }

    public void shootThreeBalls(double firstPos, double secondPos, double thirdPos)  {
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


}

