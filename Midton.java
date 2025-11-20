package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous
// don't change the program name trust
// new PID function not used for shooter or when setting power to 0

public class Midton extends LinearOpMode {

    double cpi = 0.5; // cycles per inch
    double cpd = 11; // clicks per degree

    public DcMotor BR, BL, FR, FL, Shoot;

    public enum robotMotion
    {
        right, left
    }
        public void setPowerPID(DcMotor motor, int target, double kp, double kl, double kd, OpMode opmode) {
            // kp/kl/kd is input, p/l/d is output
            ElapsedTime timer = new ElapsedTime();

            // IF CHANGING ALGORITHM PARAMETERS
            //MOE = margin of error (in ticks)
            int MOE = 3;
            double previousTime = 0, previousError = 0;
            double p = 0, i = 0, d = 0;
            double max_i = 0.2, min_i = -0.2
            double power;
            // error constant should be within MOE, MOE is like a limit
            while(Math.abs(target - motor.getCurrentPosition()) > 3 && ((LinearOpMode)opmode).opModeIsActive()))
            double currentTime = timer.milliseconds();
            double error = target - motor.getCurrentPosition();

            //proportional error
            p = kp * error; // directly proportional to error

            //integral error
            i += ki * (error * (currentTime - i));
            if (i > max_i) {
                i = max_i
            }
            else if (i < min_i) {
                i = min_i;
            }

            //derivative error
            d = kd * (error - previousError) / (currentTime - previousTime); // directly proportional to rate

            power = p + i + d
            motor.setPower(power);

            //save the values
            previousError = error;
            previousTime = currentTime;
        }

    }

    @Override
    public void runOpMode() throws InterruptedException {

        // hardware map
        BR = hardwareMap.get(DcMotor.class, "BR");
        BL = hardwareMap.get(DcMotor.class, "BL");
        FR = hardwareMap.get(DcMotor.class, "FR");
        FL = hardwareMap.get(DcMotor.class, "FL");
        Shoot = hardwareMap.get(DcMotor.class, "S");
        BR.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.FORWARD);
        FR.setDirection(DcMotorSimple.Direction.REVERSE);
        FL.setDirection(DcMotorSimple.Direction.FORWARD);
        Shoot.setDirection(DcMotorSimple.Direction.FORWARD);

        // set mode
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Shoot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Shoot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Shoot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // set initial power/position
        BR.setPower(0);
        BL.setPower(0);
        FR.setPower(0);
        FL.setPower(0);
        Shoot.setPower(0);


        //motor powers:
        //forward/backward: 0.5
        //strafe: 0.5
        //right/left: 0.3

        waitForStart();
        //TODO: Code Here!
        forward(20, 0.5);
        shootM(2   ,0.5);

    }

    private void shootM(double inch, double power){
        int e = (int)(Shoot.getCurrentPosition() + (inch * cpi));
        Shoot.setTargetPosition(e);
        Shoot.setPower(power);
        Shoot.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (Shoot.isBusy()) {

            telemetry.addData("Shoot Current", Shoot.getCurrentPosition());


            telemetry.update();
        }

        Shoot.setPower(0);
    }
    private void forward(double inch,  double power) {
        // calculate new position for (wheel) motors
        int a = (int)(BR.getCurrentPosition() + (inch * cpi));
        int b = (int)(BL.getCurrentPosition() + (inch * cpi));
        int c = (int)(FR.getCurrentPosition() + (inch * cpi));
        int d = (int)(FL.getCurrentPosition() + (inch * cpi));

        // sets new position for motors
        BR.setTargetPosition(a);
        BL.setTargetPosition(b);
        FL.setTargetPosition(c);
        FR.setTargetPosition(d);

        // sets desired power for motors
        BR.setPowerPID(power);
        BL.setPowerPID(power);
        FR.setPowerPID(power);
        FL.setPowerPID(power);

        // make motors run to position
        BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // loop to get telemetry while motors are running
        while (BR.isBusy() && BL.isBusy() && FR.isBusy() && FL.isBusy()) {
            telemetry.addLine("Forward");

            telemetry.addData("BR Target", a);
            telemetry.addData("BL Target", b);
            telemetry.addData("FR Target", c);
            telemetry.addData("FL Target", d);

            telemetry.addData("BR Current", BR.getCurrentPosition());
            telemetry.addData("BL Current", BL.getCurrentPosition());
            telemetry.addData("FR Current", FR.getCurrentPosition());
            telemetry.addData("FL Current", FL.getCurrentPosition());

            telemetry.update();
        }
        // stop motors
        BR.setPower(0);
        BL.setPower(0);
        FR.setPower(0);
        FL.setPower(0);
    }

    private void backward(double inch,  double power) {
        // calculate new position for (wheel) motors
        int a = (int)(BR.getCurrentPosition() - (inch * cpi));
        int b = (int)(BL.getCurrentPosition() - (inch * cpi));
        int c = (int)(FR.getCurrentPosition() - (inch * cpi));
        int d = (int)(FL.getCurrentPosition() - (inch * cpi));

        // sets new position for motors
        BR.setTargetPosition(a);
        BL.setTargetPosition(b);
        FR.setTargetPosition(c);
        FL.setTargetPosition(d);

        // sets desired power for motors
        BR.setPowerPID(power);
        BL.setPowerPID(power);
        FR.setPowerPID(power);
        FL.setPowerPID(power);

        // make motors run to position
        BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // loop to get telemetry while motors are running
        while (BR.isBusy() && BL.isBusy() && FR.isBusy() && FL.isBusy()) {
            telemetry.addLine("Backward");

            telemetry.addData("BR Target", a);
            telemetry.addData("BL Target", b);
            telemetry.addData("FR Target", c);
            telemetry.addData("FL Target", d);

            telemetry.addData("BR Current", BR.getCurrentPosition());
            telemetry.addData("BL Current", BL.getCurrentPosition());
            telemetry.addData("FR Current", FR.getCurrentPosition());
            telemetry.addData("FL Current", FL.getCurrentPosition());

            telemetry.update();
        }
        // stop motors
        BR.setPower(0);
        BL.setPower(0);
        FR.setPower(0);
        FL.setPower(0);
    }

    private void strafeLeft (double inch,  double power) {
        // FL and BR go backward, FR and BL go forward
        BR.setTargetPosition((int) (BR.getCurrentPosition() - (inch * cpi)));
        BL.setTargetPosition((int) (BL.getCurrentPosition() + (inch * cpi)));
        FR.setTargetPosition((int) (FR.getCurrentPosition() + (inch * cpi)));
        FL.setTargetPosition((int) (FL.getCurrentPosition() - (inch * cpi)));

        // sets desired power for motors
        BR.setPowerPID(-power);
        BL.setPowerPID(power);
        FR.setPowerPID(power);
        FL.setPowerPID(-power);

        // make motors run to position
        BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // loop to get telemetry while motors are running
        while (BR.isBusy() && BL.isBusy() && FR.isBusy() && FL.isBusy()) {
            telemetry.addLine("Strafe Left");

            telemetry.addData("BR Target", BR.getTargetPosition());
            telemetry.addData("BL Target", BL.getTargetPosition());
            telemetry.addData("FR Target", FR.getTargetPosition());
            telemetry.addData("FL Target", FL.getTargetPosition());

            telemetry.addData("BR Current", BR.getCurrentPosition());
            telemetry.addData("BL Current", BL.getCurrentPosition());
            telemetry.addData("FR Current", FR.getCurrentPosition());
            telemetry.addData("FL Current", FL.getCurrentPosition());

            telemetry.update();
        }
        // stop motors
        BR.setPower(0);
        BL.setPower(0);
        FR.setPower(0);
        FL.setPower(0);
    }

    private void strafeRight (double inch,  double power) {
        // FR and BL go backward, FL and BR go forward
        BR.setTargetPosition((int) (BR.getCurrentPosition() + (inch * cpi)));
        BL.setTargetPosition((int) (BL.getCurrentPosition() - (inch * cpi)));
        FR.setTargetPosition((int) (FR.getCurrentPosition() - (inch * cpi)));
        FL.setTargetPosition((int) (FL.getCurrentPosition() + (inch * cpi)));

        // sets desired power for motors
        BR.setPowerPID(power);
        BL.setPowerPID(-power);
        FR.setPowerPID(-power);
        FL.setPowerPID(power);

        // make motors run to position
        BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // loop to get telemetry while motors are running
        while (BR.isBusy() && BL.isBusy() && FR.isBusy() && FL.isBusy()) {
            telemetry.addLine("Strafe Right");

            telemetry.addData("BR Target", BR.getTargetPosition());
            telemetry.addData("BL Target", BL.getTargetPosition());
            telemetry.addData("FR Target", FR.getTargetPosition());
            telemetry.addData("FL Target", FL.getTargetPosition());

            telemetry.addData("BR Current", BR.getCurrentPosition());
            telemetry.addData("BL Current", BL.getCurrentPosition());
            telemetry.addData("FR Current", FR.getCurrentPosition());
            telemetry.addData("FL Current", FL.getCurrentPosition());

            telemetry.update();
        }
        // stop motors
        BR.setPower(0);
        BL.setPower(0);
        FR.setPower(0);
        FL.setPower(0);
    }

    private void movementRL(robotMotion action, double degree,  double power) {

        if(action == robotMotion.left) {
            // left is moving backwards, right is moving forwards
            BR.setTargetPosition((int) (BR.getCurrentPosition() + (degree * cpd)));
            BL.setTargetPosition((int) (BL.getCurrentPosition() - (degree * cpd)));
            FR.setTargetPosition((int) (FR.getCurrentPosition() + (degree * cpd)));
            FL.setTargetPosition((int) (FL.getCurrentPosition() - (degree * cpd)));

            // sets desired power for motors
            BR.setPowerPID(power);
            BL.setPowerPID(-power);
            FR.setPowerPID(power);
            FL.setPowerPID(-power);

            // make motors run to position
            BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // loop to get telemetry while motors are running
            while (BR.isBusy() && BL.isBusy() && FR.isBusy() && FL.isBusy()) {
                telemetry.addLine("Turning Left");

                telemetry.addData("BR Target", BR.getTargetPosition());
                telemetry.addData("BL Target", BL.getTargetPosition());
                telemetry.addData("FR Target", FR.getTargetPosition());
                telemetry.addData("FL Target", FL.getTargetPosition());

                telemetry.addData("BR Current", BR.getCurrentPosition());
                telemetry.addData("BL Current", BL.getCurrentPosition());
                telemetry.addData("FR Current", FR.getCurrentPosition());
                telemetry.addData("FL Current", FL.getCurrentPosition());

                telemetry.update();
            }
            // stop motors
            BR.setPower(0);
            BL.setPower(0);
            FR.setPower(0);
            FL.setPower(0);
        }

        if(action == robotMotion.right) {
            // right is moving backwards, left is moving forwards
            BR.setTargetPosition((int) (BR.getCurrentPosition() - (degree * cpd)));
            BL.setTargetPosition((int) (BL.getCurrentPosition() + (degree * cpd)));
            FR.setTargetPosition((int) (FR.getCurrentPosition() - (degree * cpd)));
            FL.setTargetPosition((int) (FL.getCurrentPosition() + (degree * cpd)));

            // sets desired power for motors
            BR.setPowerPID(-power);
            BL.setPowerPID(power);
            FR.setPowerPID(-power);
            FL.setPowerPID(power);

            // make motors run to position
            BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // loop to get telemetry while motors are running
            while (BR.isBusy() && BL.isBusy() && FR.isBusy() && FL.isBusy()) {
                telemetry.addLine("Turning Right");

                telemetry.addData("BR Target", BR.getTargetPosition());
                telemetry.addData("BL Target", BL.getTargetPosition());
                telemetry.addData("FR Target", FR.getTargetPosition());
                telemetry.addData("FL Target", FL.getTargetPosition());

                telemetry.addData("BR Current", BR.getCurrentPosition());
                telemetry.addData("BL Current", BL.getCurrentPosition());
                telemetry.addData("FR Current", FR.getCurrentPosition());
                telemetry.addData("FL Current", FL.getCurrentPosition());

                telemetry.update();
            }
            // stop motors
            BR.setPower(0);
            BL.setPower(0);
            FR.setPower(0);
            FL.setPower(0);
        }
    }
} // end class
