/*ATTENTION: THIS PROGRAM HAS NOT BEEN TRIED OUT ON THE ROBOT!
HAVE THIS REVIEWED BY SOMEONE ELSE BEFORE RUNNING THE PROGRAM
TO MAKE SURE MOTORS, ENCODERS, AND HUB DON'T GET DAMAGED*/

// anything with "added in" refers to important things that were added
// No methods created in this program were called, see end for more info

// pkg
package org.firstinspires.ftc.teamcode;
// imports
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import static org.firstinspires.ftc.teamcode.Encoders_SD.robotMotion.armDown;
import static org.firstinspires.ftc.teamcode.Encoders_SD.robotMotion.armUp;


@Autonomous
//file's main class inheriting LinearOpMode
public class Encoders_SD extends LinearOpMode
{
    // define the DC motors
    public DcMotor BR, BL, FR, FL, Arm;


    // clicks per degree
    // double cpd = 21.94;
    // clicks per inch
    double cpi = 7.5;
    // double lcpi = 68;

    // NOTE: armUp and armDown is for intake/outtake
    public enum robotMotion
    {
        right, left, armUp, armDown

    }

    @Override
    public void runOpMode() throws InterruptedException
    {

        // hardware map
        BR = hardwareMap.get(DcMotor.class, "BR");
        BL = hardwareMap.get(DcMotor.class, "BL");
        FR = hardwareMap.get(DcMotor.class, "FR");
        FL = hardwareMap.get(DcMotor.class, "FL");
        Arm = hardwareMap.get(DcMotor.class, "Arm");

        // set direction
        BR.setDirection(DcMotorSimple.Direction.FORWARD);
        BL.setDirection(DcMotorSimple.Direction.REVERSE);
        FR.setDirection(DcMotorSimple.Direction.FORWARD);
        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        // Arm.setDirection() (to be determined!)
        Arm.setDirection(DcMotorSimple.Direction.REVERSE);

        // set mode for EACH motor, TODO: setMode for Arm
        BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //added in arm initializer
        Arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // set initial power/position
        FR.setPower(0);
        FL.setPower(0);
        BR.setPower(0);
        BL.setPower(0);
        //added in arm set power
        Arm.setPower(0);
        //wait until the main thing starts
        waitForStart();

        // TODO: AUTON CODE GOES HERE!!
        // added in some sample auton
        forward(36,0.35);
        //drop spike
        movementRL(robotMotion.right, 28, 0.35);
        //drop spike
        robotsleep(0,0.0);

    }

    private void forward(double inch,  double power)
    {
        // calculates new position for motors
        // use getCurrentPosition(), inch, cpi
        // make sure you cast to int!
        int current_FR = FR.getCurrentPosition();
        int current_FL = FL.getCurrentPosition();
        int current_BR = BR.getCurrentPosition();
        int current_BL = BL.getCurrentPosition();

        int new_FR = current_FR + (int)(cpi*inch);
        int new_FL = current_FL + (int)(cpi*inch);
        int new_BR = current_BR + (int)(cpi*inch);
        int new_BL = current_BL + (int)(cpi*inch);




        // sets new position for motors, use setTargetPosition()
        //added in pos for each motor
        FL.setTargetPosition(new_FL);
        FR.setTargetPosition(new_FR);
        BL.setTargetPosition(new_BL);
        BR.setTargetPosition(new_BR);

        // sets desired power for motors
        //added in pwr for each motor
        FL.setPower(power);
        FR.setPower(power);
        BL.setPower(power);
        BR.setPower(power);

        // sets motors to RUN_TO_POSITION
        // added in run to position for each motor
        FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);



        //loop to run encoders method (do not delete!)
        // ex. FL.isBusy() returns true is FL motor is “headed”         towards target position

        while (FL.isBusy() && FR.isBusy() && BR.isBusy() && BL.isBusy())
        {
            telemetry.addLine("Moving Forward...");
            telemetry.addData("FR current: ", FR.getCurrentPosition());
            telemetry.addData("FR target: ", new_FR);
            telemetry.addData("FL current: ", FL.getCurrentPosition());
            telemetry.addData("FL target: ", new_FL);
            telemetry.addData("BR current: ", BR.getCurrentPosition());
            telemetry.addData("BR target: ", new_BR);
            telemetry.addData("BL current: ", BL.getCurrentPosition());
            telemetry.addData("BL target: ", new_BL);

        }


        // stop motors


    }

    private void backward(int inch,  double power)
    {
        // calculates new position for motors
        // use getCurrentPosition(), inch, cpi
        // make sure you cast to int!
        int current_FR = FL.getCurrentPosition();
        int current_FL = FR.getCurrentPosition();
        int current_BR = BR.getCurrentPosition();
        int current_BL = BL.getCurrentPosition();

        int new_FR = current_FR - (int)(cpi*inch);
        int new_FL = current_FL - (int)(cpi*inch);
        int new_BR = current_BR - (int)(cpi*inch);
        int new_BL = current_BL - (int)(cpi*inch);


        // sets new position for motors, use setTargetPosition()
        //added in pos for each motor

        FL.setTargetPosition(new_FL);
        FR.setTargetPosition(new_FR);
        BL.setTargetPosition(new_BL);
        BR.setTargetPosition(new_BR);

        // sets desired power for motors
        //added in pwr for each motor

        FL.setPower(power*-1);
        FR.setPower(power*-1);
        BL.setPower(power*-1);
        BR.setPower(power*-1);

        // sets motors to RUN_TO_POSITION
        // added in run to position for each motor
        FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);



        //loop to run encoders method (do not delete!)
        // ex. FL.isBusy() returns true is FL motor is “headed”         towards target position

        while (FL.isBusy() && FR.isBusy() && BL.isBusy() && BR.isBusy())
        {
            telemetry.addLine("Moving Backward...");
            telemetry.addData("FR current: ", FR.getCurrentPosition());
            telemetry.addData("FR target: ", new_FR);
            telemetry.addData("FL current: ", FL.getCurrentPosition());
            telemetry.addData("FL target: ", new_FL);
            telemetry.addData("BR current: ", BR.getCurrentPosition());
            telemetry.addData("BR target: ", new_BR);
            telemetry.addData("BL current: ", BL.getCurrentPosition());
            telemetry.addData("BL target: ", new_BL);
        }

        // stop motors

    }

    private void movementRL(robotMotion action, double inch,  double power)
    {
        // calculates new position for motors
        // use getCurrentPosition(), inch, cpi
        // make sure you cast to int!
        // TODO: complete for each WHEEL motor
        int current_FR = FL.getCurrentPosition();
        int current_FL = FR.getCurrentPosition();
        int current_BR = BR.getCurrentPosition();
        int current_BL = BL.getCurrentPosition();


        // sets new position for motors, use setTargetPosition()
        // TODO: complete for each WHEEL motor
        //added in pos for each motor


        if(action == robotMotion.left)
        {
            int new_FR = current_FR + (int)(cpi*inch);
            int new_FL = current_FL - (int)(cpi*inch);
            int new_BR = current_BR - (int)(cpi*inch);
            int new_BL = current_BL + (int)(cpi*inch);
            // sets desired power for motors
            // added in pwr for each motor
            FR.setPower(power);
            FL.setPower(power*-1);
            BR.setPower(power*-1);
            BL.setPower(power);

            // sets motors to RUN_TO_POSITION
            // added in run to position for each motor
            FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);



            //loop to run encoders method (do not delete!)
            // ex. FL.isBusy() returns true is FL motor is “headed”         towards target position

            while (FL.isBusy() && FR.isBusy() && BR.isBusy() && BL.isBusy())
            {
                telemetry.addLine("Moving Left...");
                telemetry.addData("FR current: ", FR.getCurrentPosition());
                telemetry.addData("FR target: ", new_FR);
                telemetry.addData("FL current: ", FL.getCurrentPosition());
                telemetry.addData("FL target: ", new_FL);
                telemetry.addData("BR current: ", BR.getCurrentPosition());
                telemetry.addData("BR target: ", new_BR);
                telemetry.addData("BL current: ", BL.getCurrentPosition());
                telemetry.addData("BL target: ", new_BL);

            }


            // stop motors


        }

        if(action == robotMotion.right)
        {
            // sets desired power for motors
            //added in pwr for each motor
            int new_FR = current_FR - (int)(cpi*inch);
            int new_FL = current_FL + (int)(cpi*inch);
            int new_BR = current_BR + (int)(cpi*inch);
            int new_BL = current_BL - (int)(cpi*inch);
            // sets desired power for motors
            // added in pwr for each motor
            FR.setPower(power*-1);
            FL.setPower(power);
            BR.setPower(power);
            BL.setPower(power*-1);

            // sets motors to RUN_TO_POSITION
            // added in run to position for each motor
            FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);



            //loop to run encoders method (do not delete!)
            // ex. FL.isBusy() returns true is FL motor is “headed”         towards target position

            while (FL.isBusy() && FR.isBusy() && BR.isBusy() && BL.isBusy())
            {
                telemetry.addLine("Moving Right...");
                telemetry.addData("FR current: ", FR.getCurrentPosition());
                telemetry.addData("FR target: ", new_FR);
                telemetry.addData("FL current: ", FL.getCurrentPosition());
                telemetry.addData("FL target: ", new_FL);
                telemetry.addData("BR current: ", BR.getCurrentPosition());
                telemetry.addData("BR target: ", new_BR);
                telemetry.addData("BL current: ", BL.getCurrentPosition());
                telemetry.addData("BL target: ", new_BL);

            }

        }

    }
    // TODO: function for arm attachment
    private void armMovement (robotMotion action, double degree, double power)
    {
        if (action == armUp){
            //open arm
        }
        else if (action == armDown){
            // close arm
        }
    }

    // TODO: function for claw movement
    private void clawMovement (boolean state)
    {
        if (state == true)
        {
            // open claw
        }
        else
        {
            // close claw
        }
    }

    // NOTE: This is important, do NOT delete!
    private void robotsleep(int sleep, double power)
    {
        BR.setPower(0);
        FR.setPower(0);
        BL.setPower(0);
        FL.setPower(0);
        // ADDED IN ARM PWR TO 0
        Arm.setPower(0);
    }
}

/* to add in:
when joystick goes front, back, left, right, match movement with robot using the defined methods
add in arm movement directions (check beginning of code) and a way to maneuver, lift/drop the arm using joystick+buttons.
 */
