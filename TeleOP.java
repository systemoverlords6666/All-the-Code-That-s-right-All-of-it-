package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "TeleOP_6666_wheels")
public class TeleOP_6666 extends OpMode{

    // Robot  start

    DcMotor lm;
    DcMotor rm;
    DcMotor up;

    double leftwheelpower;
    double rightwheelpower;
    double goingup;





    @Override
    public void init() {

        lm = hardwareMap.dcMotor.get("lm");
        rm = hardwareMap.dcMotor.get("rm");
        up = hardwareMap.dcMotor.get("up");

    }

    @Override
    public void loop() {

        leftwheelpower = gamepad1.left_stick_y;
        rightwheelpower = gamepad1.right_stick_y;
        goingup =  gamepad2.left_stick_y;

        lm.setPower(leftwheelpower);
        rm.setPower(rightwheelpower);
        up.setPower(goingup);

    }

    //Robot Motion Ends

}
