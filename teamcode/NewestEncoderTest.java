package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name="Newest Encoder Test", group="test")
//@Disabled
public class NewestEncoderTest extends LinearOpMode {
    SelectronV3 bot = null;
    ElapsedTime r = null;
    @Override
    public void runOpMode() {
        bot = new SelectronV3(this);
        bot.setEncoders(DcMotor.RunMode.RUN_TO_POSITION);
        r = new ElapsedTime();

        waitForStart();

        bot.newEncoderInchesPowerDrive(24, 0.35, bot.getInchesTimeout(24));
        bot.newEncoderInchesPowerDrive(-18, 0.35, bot.getInchesTimeout(-18));
        bot.newEncoderInchesPowerDrive(12, 0.35, bot.getInchesTimeout(12));
        bot.newEncoderInchesPowerDrive(-6, 0.35, bot.getInchesTimeout(-6));

        //1742 1489 913 520
        //146 + 68 * inches
        // 1756 1310 920 533
        //125 + 68 * inches
        // say, b = 500, a = 70 (for inches)



        bot.newEncoderDegreesPowerTurn(180, 0.25, bot.getDegreesTimeout(180));
        bot.newEncoderDegreesPowerTurn(-135, 0.25, bot.getDegreesTimeout(-135));
        bot.newEncoderDegreesPowerTurn(90, 0.25, bot.getDegreesTimeout(90));
        bot.newEncoderDegreesPowerTurn(-45, 0.25, bot.getDegreesTimeout(-45));
    }
}