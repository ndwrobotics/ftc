package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * Is also defines all of the methods for the robot.
 * In that case, this is our (10925 Spartacles') robot.
 * (Affectionately dubbed Selectron.)
 */
public class Selectron
{
    /* Public OpMode members. */
    //--1-begin
    DcMotor wheelR = null;
    DcMotor wheelL = null;
    //--1-end
    DcMotor wheelR2 = null;
    DcMotor wheelL2 = null;

    //--2-begin
    Servo leftServo = null;
    Servo rightServo = null;
    //--2-end

    DcMotor jewelDisplacer = null;

    TouchSensor touchSensor = null;

    DcMotor lift = null;

    NormalizedColorSensor colorSensor = null;
    DcMotor someMotors[] = null;
    DcMotor moreMotors[] = null;

    double UP_SPEED = -0.5; //tweak this
    double DOWN_SPEED = 0.8;


    double LEFT_CENTER = 0.56; //tweak this
    double RIGHT_CENTER = 0.35; //tweak this
    double LEFT_SLIGHT_OUT = 0.47;
    double RIGHT_SLIGHT_OUT = 0.48;
    double LEFT_IN = 0.99; //tweak this
    double RIGHT_IN = 0.0; //tweak this
    double LEFT_OUT = 0.34; //tweak this
    double RIGHT_OUT = 0.53; //tweak this //done
    //--2-end

    int JEWEL_OUT = -580; //tweak this
    int JEWEL_IN = 0; //tweak this //done

    double COLOR_THRESHOLD = 0.71;



    VuforiaLocalizer vuforia;
    RelicRecoveryVuMark vuMark = null;
    VuforiaTrackables relicTrackables;
    VuforiaTrackable relicTemplate;




    /* local OpMode members. */
    LinearOpMode op = null;

    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public Selectron(LinearOpMode opMode){
        op = opMode;

        colorSensor = op.hardwareMap.get(NormalizedColorSensor.class, "color");

        //--1-begin
        wheelR = op.hardwareMap.dcMotor.get("motorR");
        wheelL = op.hardwareMap.dcMotor.get("motorL");
        //--1-begin
        wheelR2 = op.hardwareMap.dcMotor.get("motorR2");
        wheelL2 = op.hardwareMap.dcMotor.get("motorL2");

        wheelL.setDirection(DcMotorSimple.Direction.REVERSE);//This motor is pointing the wrong direction
        wheelL2.setDirection(DcMotorSimple.Direction.REVERSE);

        touchSensor = op.hardwareMap.touchSensor.get("touchUp");


        leftServo = op.hardwareMap.servo.get("left");
        rightServo = op.hardwareMap.servo.get("right");

        lift = op.hardwareMap.dcMotor.get("lift");

        jewelDisplacer = op.hardwareMap.dcMotor.get("jeweler");


        someMotors = new DcMotor[2];
        moreMotors = new DcMotor[4];

        someMotors[0] = wheelL;
        someMotors[1] = wheelR;

        moreMotors[0] = wheelL;
        moreMotors[1] = wheelR;
        moreMotors[2] = wheelL2;
        moreMotors[3] = wheelR2;
        for (DcMotor motor: moreMotors){
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        jewelDisplacer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        jewelDisplacer.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftServo.setPosition(LEFT_CENTER);
        rightServo.setPosition(RIGHT_CENTER);
    }


    public void initVuforia(){
        int cameraMonitorViewId = op.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", op.hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // OR...  Do Not Activate the Camera Monitor View, to save power
        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey =
                "Aczwc7j/////AAAAGSh6oYrgWE/wvIHsaTUy6ppNpIawgvZu5xiQAUK+xtcW8Zw8P9GhxqBOsB2RKBAP6cOUkwCiyhCaYHKfrhA7ORLNRka9TTTm36bBHMY/WhR02K5Z5Qa2TaqRNjtpZ5rZ2Q8Ee+vnuiVasqj3uAMCB0ceFPYemYJ8snub+w/8gKVwy09n+ZWJ/5yymbJ9lGIQAwlc8Wo0HogRSK6Yk0Z0CiX/UFKhB+wK+I7Vzcwv0pZVOXAhf7fuDJOUr+TcuCBptm6AlFai2evVFuKT1seKaTsX+Sa2jvP3F0O1gbfLYK1dWjIh5/mqEp6yWgk2D+fAhGGWVjeUkN0ftstV6F1U06+f9P52sOd6w0Fuagpm27TP";

        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary
    }

    public void findPicture(){
        relicTrackables.activate();
        vuMark = RelicRecoveryVuMark.from(relicTemplate);
        ElapsedTime runtime = new ElapsedTime();
        while (op.opModeIsActive() && (vuMark == RelicRecoveryVuMark.UNKNOWN) && runtime.seconds() <= 7){
            vuMark = RelicRecoveryVuMark.from(relicTemplate);
            op.telemetry.addData("VuMark, ", "%s visible", vuMark);
            op.telemetry.update();
        }
    }


    public void setEncoders(DcMotor.RunMode mode){
        for (DcMotor motor:moreMotors){
            motor.setMode(mode);
        }
    }


    public void displaceJewel(double color_threshold, int count, boolean redAlliance) {
        if (count < 4 && op.opModeIsActive()) {
            NormalizedRGBA colors;


            double red = 0;
            double blue = 0;
            float max;
            ElapsedTime r = new ElapsedTime();
            for (int i = 0; i < 10; i++) {
                colors = colorSensor.getNormalizedColors();
                max = Math.max(Math.max(Math.max(colors.red, colors.green), colors.blue), colors.alpha);
                colors.red /= max;
                colors.green /= max;
                colors.blue /= max;
                red += colors.red;
                blue += colors.blue;
                op.telemetry.addLine("normalized color:  ")
                        .addData("a", colors.alpha)
                        .addData("r", colors.red)
                        .addData("g", colors.green)
                        .addData("b", colors.blue);
                op.telemetry.update();
                s(50, r);
            }
            op.telemetry.addData("blue: ", blue);
            op.telemetry.addData("red: ", red);
            op.telemetry.update();

            double power = 0.15;

            if (op.opModeIsActive()){
                if (red > 10 * color_threshold) {
                    if (redAlliance){
                        encoderPowerTurn(30, power);
                        encoderPowerTurn(-30, power);
                    } else {
                        encoderPowerTurn(-30, power);
                        encoderPowerTurn(30, power);
                    }
                } else if (blue > 10 * color_threshold) {
                    if(redAlliance){
                        encoderPowerTurn(-30, power);
                        encoderPowerTurn(30, power);
                    } else {
                        encoderPowerTurn(30, power);
                        encoderPowerTurn(-30, power);
                    }

                } else {
                    if (count == 0){
                        encoderPowerDrive(0.5, 0.15);
                    } else if (count == 1){
                        encoderPowerDrive(-1, 0.15);
                    } else if (count == 2){
                        encoderPowerDrive(-0.5, 0.15);
                    } else if (count == 3){
                        encoderPowerDrive(-0.5, 0.15);
                    }
                    displaceJewel(color_threshold, count + 1, redAlliance);
                }
            }
        }
    }

    public void displaceJewel(double color_threshold, boolean redAlliance){
        displaceJewel(color_threshold, 0, redAlliance);
    }


    public void s(double time, ElapsedTime r){
        double t = r.milliseconds();
        while (op.opModeIsActive() && r.milliseconds() <= t + time){}
    }

    public void encoderPowerDrive(double inches, double power){

        // wheel circumference = 4.0 * pi
        // encoder counts is 1120 per revolution
        // 1120 counts per 4pi inches, so 1120/4pi counts per inch
        double COUNTS_PER_INCH = (280/3.1416); // actually 3.141592653589793238462643383279502... but whatever
        double COMPENSATION_FACTOR = 1.0;
        int counts = (int) (COUNTS_PER_INCH * inches * COMPENSATION_FACTOR);
        if (op.opModeIsActive()) {
            for (DcMotor motor : someMotors){
                motor.setTargetPosition(motor.getCurrentPosition() + counts);
            }
            for (DcMotor motor : someMotors){
                motor.setPower(power);
            }
            while (op.opModeIsActive() && (wheelL.isBusy() || wheelR.isBusy())){
                op.telemetry.addData("Target position: ", inches);
                op.telemetry.addData("Encoder Left 1 position: ", wheelL.getCurrentPosition());
                op.telemetry.addData("Encoder Right 1 position: ", wheelR.getCurrentPosition());
                op.telemetry.update();
            }
            for (DcMotor motor : someMotors){
                motor.setPower(0);
            }
            s(250, new ElapsedTime());

        }
    }

    public void encoderDrive(double inches){
        encoderPowerDrive(inches, 0.25);
    }


    public void encoder4WDrive(double inches){

        // wheel circumference = 4.0 * pi
        // encoder counts is 1120 per revolution
        // 1120 counts per 4pi inches, so 1120/4pi counts per inch
        double COUNTS_PER_INCH = (280/3.1416); // actually 3.141592653589793238462643383279502... but whatever
        double COMPENSATION_FACTOR = 1.0;
        int counts = (int) (COUNTS_PER_INCH * inches * COMPENSATION_FACTOR);

        if (op.opModeIsActive()) {
            for (DcMotor motor : moreMotors){
                motor.setTargetPosition(motor.getCurrentPosition() + counts);
            }
            for (DcMotor motor : moreMotors){
                motor.setPower(0.25);
            }
            while (op.opModeIsActive() && (wheelL.isBusy() || wheelR.isBusy() || wheelL2.isBusy() || wheelR2.isBusy())){
                op.telemetry.addData("Target position: ", inches);
                op.telemetry.addData("Encoder Left 1 position: ", wheelL.getCurrentPosition());
                op.telemetry.addData("Encoder Right 1 position: ", wheelR.getCurrentPosition());
                op.telemetry.update();
            }
            for (DcMotor motor : moreMotors){
                motor.setPower(0);
            }
            s(250, new ElapsedTime());

        }
    }

    public void encoderPowerTurn(double degrees, double power){
        // degrees positive goes counterclockwise
        // degrees negative goes clockwise

        // wheel circumference = 4.0 * pi
        // encoder counts is 1120 per revolution
        // 1120 counts per 4pi inches, so 280/pi counts per inch
        // turn diameter is 13.75 inches, so a 360 degree turn is 13.75pi inches
        // so one degree is 13.75pi/360 = 55pi/1440 = 11pi/288
        // 280/pi counts per inch times 11pi/288 inches per degree is 11*280/288 counts per degree
        // 11*280/288 = 11*35/36 = 385/36
        double COUNTS_PER_DEGREE = (385.0/36.0);
        double COMPENSATION_FACTOR = 17.0/16.0;
        int counts = (int) (COUNTS_PER_DEGREE * degrees * COMPENSATION_FACTOR);
        if (op.opModeIsActive()) {
            wheelL.setTargetPosition(wheelL.getCurrentPosition() - counts);
            wheelR.setTargetPosition(wheelR.getCurrentPosition() + counts);
            // should go counterclockwise
            for (DcMotor motor : someMotors){
                motor.setPower(power);
            }
            while (op.opModeIsActive() && (wheelL.isBusy() || wheelR.isBusy())){
                op.telemetry.addData("Target position: ", counts);
                op.telemetry.addData("Encoder Left 1 position: ", wheelL.getCurrentPosition());
                op.telemetry.addData("Encoder Right 1 position: ", wheelR.getCurrentPosition());
                op.telemetry.update();
            }
            for (DcMotor motor : someMotors){
                motor.setPower(0);
            }
            s(250, new ElapsedTime());
        }
    }

    public void encoderTurn(double degrees){
        encoderPowerTurn(degrees, 0.25);
    }

    public void timePowerDrive(double milliseconds, boolean forwards, double power, ElapsedTime r){
        for (DcMotor motor : someMotors){
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        for (DcMotor motor : someMotors){
            if (forwards){
                motor.setPower(power);
            } else {
                motor.setPower(-power);
            }
        }
        s(milliseconds, r);
        for (DcMotor motor : someMotors){
            motor.setPower(0);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        s(250, r);
    }

    public void timeDrive(double milliseconds, boolean forwards, ElapsedTime r){
        timePowerDrive(milliseconds, forwards, 0.25, r);
    }

    public void timePower4WDrive(double milliseconds, boolean forwards, double power, ElapsedTime r){
        for (DcMotor motor : moreMotors){
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        for (DcMotor motor : moreMotors){
            if (forwards){
                motor.setPower(power);
            } else {
                motor.setPower(-power);
            }
        }
        s(milliseconds, r);
        for (DcMotor motor : moreMotors){
            motor.setPower(0);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        s(250, r);
    }
    public void time4WDrive(double milliseconds, boolean forwards, ElapsedTime r){
        timePower4WDrive(milliseconds, forwards, 0.25, r);
    }

    public void square_up(ElapsedTime r, boolean redAlliance){

        if (redAlliance){
            time4WDrive(1000, true, r);
            timePowerDrive(1200, false, 0.15, r);
        } else {
            time4WDrive(1000, false, r);
            timePowerDrive(1200, true, 0.15, r);
        }
        s(250, r);
    }


    // None of the methods from here on have been extensively tested. Use at your own peril.

    public void newAndImprovedEncoderDrive(double inches, double power, int stuckCount){
        ElapsedTime r = new ElapsedTime();
        double COUNTS_PER_INCH = (280/3.1416); // actually 3.141592653589793238462643383279502... but whatever
        double COMPENSATION_FACTOR = 1.0;
        int counts = (int) (COUNTS_PER_INCH * inches * COMPENSATION_FACTOR);
        int curPosL = wheelL.getCurrentPosition();
        int curPosR = wheelR.getCurrentPosition();

        if (op.opModeIsActive()) {
            for (DcMotor motor : someMotors) {
                motor.setTargetPosition(motor.getCurrentPosition() + counts);
            }
            for (DcMotor motor : someMotors) {
                motor.setPower(0.25);
            }

            int stuckCode = 0;
            boolean stuck = false;
            while (op.opModeIsActive() && !stuck && (wheelL.isBusy() || wheelR.isBusy())) {
                stuckCode = isStuck(stuckCode);
                if (stuckCode > 2){
                    stuck = true;
                }
                if (stuck){
                    op.telemetry.addLine("STUCK!!!!");
                } else {
                    op.telemetry.addData("Target position: ", inches);
                    op.telemetry.addData("Encoder Left 1 position: ", wheelL.getCurrentPosition());
                    op.telemetry.addData("Encoder Right 1 position: ", wheelR.getCurrentPosition());
                    op.telemetry.update();
                }
                op.telemetry.update();
            }
            for (DcMotor motor : someMotors) {
                motor.setPower(0);
            }
            if (stuck){
                op.telemetry.addData("Stuck count: ", stuckCount+1);
                op.telemetry.update();
                wheelL.setTargetPosition(wheelR.getCurrentPosition() - curPosR + curPosL);
                wheelL.setPower(0.25);
                while(wheelL.isBusy()){}
                wheelL.setPower(0);
                s(250, r);
                int currentError = curPosL + counts - wheelL.getCurrentPosition();
                newAndImprovedEncoderDrive(currentError/COUNTS_PER_INCH, 0.25, stuckCount+1);
            } else {
                op.telemetry.addData("Stuck count: ", stuckCount);
                op.telemetry.update();
            }
            s(250, r);
        }
    }

    public void newAndImprovedEncoderTurn(double degrees, double power, int stuckCount){
        ElapsedTime r = new ElapsedTime();
        double COUNTS_PER_DEGREE = (385.0/36.0);
        double COMPENSATION_FACTOR = 17.0/16.0;
        int counts = (int) (COUNTS_PER_DEGREE * degrees * COMPENSATION_FACTOR);
        int curPosL = wheelL.getCurrentPosition();
        int curPosR = wheelR.getCurrentPosition();

        if (op.opModeIsActive()) {
            wheelL.setTargetPosition(wheelL.getCurrentPosition() - counts);
            wheelR.setTargetPosition(wheelR.getCurrentPosition() + counts);
            for (DcMotor motor : someMotors) {
                motor.setPower(0.25);
            }

            int stuckCode = 0;
            boolean stuck = false;
            while (op.opModeIsActive() && !stuck && (wheelL.isBusy() || wheelR.isBusy())) {
                stuckCode = isStuck(stuckCode);
                if (stuckCode > 2){
                    stuck = true;
                }
                if (stuck){
                    op.telemetry.addLine("STUCK!!!!");
                } else {
                    op.telemetry.addData("Target position: ", degrees);
                    op.telemetry.addData("Encoder Left 1 position: ", wheelL.getCurrentPosition());
                    op.telemetry.addData("Encoder Right 1 position: ", wheelR.getCurrentPosition());
                    op.telemetry.update();
                }
                op.telemetry.update();
            }
            for (DcMotor motor : someMotors) {
                motor.setPower(0);
            }
            if (stuck){
                op.telemetry.addData("Stuck count: ", stuckCount+1);
                op.telemetry.update();
                wheelL.setTargetPosition(wheelR.getCurrentPosition() - curPosR + curPosL);
                wheelL.setPower(0.25);
                while(wheelL.isBusy()){}
                wheelL.setPower(0);
                s(250, r);
                int currentError = curPosL + counts - wheelL.getCurrentPosition();
                newAndImprovedEncoderTurn(currentError/COUNTS_PER_DEGREE, 0.25, stuckCount+1);
            } else {
                op.telemetry.addData("Stuck count: ", stuckCount);
                op.telemetry.update();
            }
            s(250, r);
        }
    }

    public int isStuck(int prevCode){
        //NOTE: MODIFY TO TEST WHEELS SEPARATELY SO IF ONE WHEEL IS ON TARGET AND THE OTHER IS NOT IT WON'T SAY IT IS STUCK
        int newCode;
        if (Math.abs(wheelL.getCurrentPosition()-wheelL.getTargetPosition())>30 || Math.abs(wheelR.getCurrentPosition()-wheelR.getTargetPosition())>30) {
            int curPosL = wheelL.getCurrentPosition();
            int curPosR = wheelR.getCurrentPosition();
            ElapsedTime r = new ElapsedTime();
            while (op.opModeIsActive() && r.milliseconds() < 100) {
            }
            if ((Math.abs(wheelL.getCurrentPosition() - curPosL) < 10) || (Math.abs(wheelR.getCurrentPosition() - curPosR) < 10)) {
                newCode = prevCode + 1;
            } else {
                newCode = 0;
            }
        } else {
            newCode = 0;
        }
        return newCode;
    }
 }









