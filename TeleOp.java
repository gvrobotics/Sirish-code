/*FEEL FREE TO DELETE/ADD COMMENTS*/

/* never forget the package! this supplies most of your code*/
package org.firstinspires.ftc.teamcode;

/* all other libraries required, any unused will appear gray*/
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/* class must match name of file*/
public class TeleOp extends OpMode {
    private double powerLY;
    private double powerRY;
    public DcMotor FR, FL, BR, BL;
    /* easier to adjust if you just set it as a variable*/

    @Override
   /* basic set up code
   (this may be adjusted once we start testing so make sure you understand it)*/
    public void init() {
        // "FR" is what you have to type into the phone


        FR = hardwareMap.get(DcMotor.class, "FR");
        FL = hardwareMap.get(DcMotor.class, "FL");
        BR = hardwareMap.get(DcMotor.class, "BR");
        BL = hardwareMap.get(DcMotor.class, "BL");

        FR.setDirection(DcMotorSimple.Direction.FORWARD);
        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        BR.setDirection(DcMotorSimple.Direction.FORWARD);
        BL.setDirection(DcMotorSimple.Direction.REVERSE);

        /* max power is 1*/
        /* going up on joystick makes the value negative */
        FR.setPower(0);
        FL.setPower(0);
        BR.setPower(0);
        BL.setPower(0);
    }

    @Override
    public void loop() {

        //double powerLX = gamepad1.left_stick_x/2;
         powerLY = gamepad1.left_stick_y/2;
        //double powerRX = gamepad1.right_stick_x/2;
         powerRY = gamepad1.right_stick_y/2;

        if(powerLY > 0.03 || powerLY < -0.03)
        {
            FL.setPower(powerLY);
            BL.setPower(powerLY);
        } else
        {
            FL.setPower(0);
            BL.setPower(0);
        }

        if(powerRY > 0.03 || powerRY < -0.03)
        {
            FR.setPower(powerRY);
            BR.setPower(powerRY);
        } else
        {
            FR.setPower(0);
            BR.setPower(0);
        }

        telemetry.addData("x", "%.2f", gamepad1.left_stick_x);
        telemetry.addData("y", "%.2f", gamepad1.left_stick_y);
        telemetry.update();

    }
}



