package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

@Autonomous(name="4V4 Jewel -Red Mat", group="Autonomous")
//@Disabled
public class Auto4V4RedMat extends LinearOpMode
{

    SelectronV3 bot = null;
    boolean RED_ALLIANCE = true;

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
            bot.encoderDrive(18);
        } else if (bot.vuMark == RelicRecoveryVuMark.RIGHT){
            bot.encoderDrive(3);
        } else {
            bot.encoderDrive(10.5);
        }

        bot.placeGlyphVuforiaEtc(RED_ALLIANCE);
        // half inch less left?
        // inch less center?

    }
}