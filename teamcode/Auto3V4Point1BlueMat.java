package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

@Autonomous(name="3V4.1 Jewel -Blue Mat", group="Autonomous")
//@Disabled
public class Auto3V4Point1BlueMat extends LinearOpMode
{


    Selectron bot = null;
    boolean RED_ALLIANCE = false;

    @Override
    public void runOpMode() {

        bot = new Selectron(this);
        bot.setEncoders(DcMotor.RunMode.RUN_TO_POSITION);
        bot.resetJewelArm();
        bot.initVuforia();

        waitForStart();
        ElapsedTime r = new ElapsedTime();

        bot.resetJewelArm();
        bot.jewelDisplacer.setTargetPosition(bot.JEWEL_OUT);
        bot.jewelDisplacer.setPower(0.25);

        bot.findPicture();

        if(opModeIsActive()){
            bot.lift.setPower(-bot.DOWN_SPEED);
        }

        bot.s(800, r);
        bot.lift.setPower(0);

        bot.displaceJewel(bot.COLOR_THRESHOLD, RED_ALLIANCE);




        bot.square_up(r, RED_ALLIANCE);

        if(bot.vuMark == RelicRecoveryVuMark.LEFT){
            bot.encoderDrive(-17.5);
            bot.placeGlyphVuforiaEtc(RED_ALLIANCE);

        } else if (bot.vuMark == RelicRecoveryVuMark.RIGHT){
            bot.encoderDrive(-19);
            bot.encoderTurn(-45);
            bot.specialPlaceGlyphVuforiaEtc(RED_ALLIANCE);

        } else {
            bot.encoderDrive(-25);
            bot.placeGlyphVuforiaEtc(RED_ALLIANCE);

        }

    }
}