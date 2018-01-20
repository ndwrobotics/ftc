package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

@Autonomous(name="4V4 Jewel -Blue Mat", group="Autonomous")
//@Disabled
public class Auto4V4BlueMat extends LinearOpMode
{


    SelectronV3 bot = null;
    boolean RED_ALLIANCE = false;

    @Override
    public void runOpMode() {

        bot = new SelectronV3(this);
        bot.setEncoders(DcMotor.RunMode.RUN_TO_POSITION);
        bot.resetJewelArm();
        bot.initVuforia();

        waitForStart();
        ElapsedTime r = new ElapsedTime();

        bot.resetJewelArm();
        bot.jewelDisplacer.setTargetPosition(bot.JEWEL_OUT);
        bot.jewelDisplacer.setPower(0.25);

        bot.findPicture();

        bot.autoRaiseLift();

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