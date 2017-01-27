package org.firstinspires.ftc.teamcode.Vuforia;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * This is NOT an opmode.
 *
 * This class defines all the specific hardware for a three wheel omni-bot.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Front Left  drive motor:        "FL"
 * Motor channel:  Front Right drive motor:        "FR"
 * Motor channel:  Back Left  drive motor:        "BL"
 * Motor channel:  Back Right drive motor:        "BR"
 *
 * These motors correspond to the four drive motors, two on either side of the robot. This is a normal
 * mecanum drive setup, with the wheels placed in a square pattern.
 *
 * Robot motion is defined in three different axis motions:
 * - Axial    Forward/Backwards      +ve = Forward
 * - Lateral  Side to Side strafing  +ve = Right
 * - Yaw      Rotating               +ve = CCW
 */


public class Robot_MecanumDrive
{
    // Private Members
    private LinearOpMode myOpMode;

    public DcMotor  FR = null;
    public DcMotor  FL = null;
    public DcMotor  BR = null;
    public DcMotor  BL = null;
    public DcMotor Shooter = null;
    public DcMotor Harvester = null;

    public Servo beaconPusher = null;

    ModernRoboticsI2cGyro gyro;
    ColorSensor beaconSensor;

    //sets default Powers
    double shooterPower = 1;
    double harvesterPower = 1;
    int shooterRot = 1100;

    //Needed Values
    double ticsPerInch = 38.5;
    double strafeTicsPerInch = 53;

    double leftPos = .3;
    double rightPos = 0;

    private double  driveAxial      = 0 ;   // Positive is forward
    private double  driveLateral    = 0 ;   // Positive is right
    private double  driveYaw        = 0 ;   // Positive is CCW

    /* Constructor */
    public Robot_MecanumDrive(){

    }


    /* Initialize standard Hardware interfaces */
    public void initDrive(LinearOpMode opMode) {

        // Save reference to Hardware map
        myOpMode = opMode;

        // Define and Initialize Motors
        FR = myOpMode.hardwareMap.get(DcMotor.class, "FR");
        BR = myOpMode.hardwareMap.get(DcMotor.class, "BR");
        FL = myOpMode.hardwareMap.get(DcMotor.class, "FL");
        BL = myOpMode.hardwareMap.get(DcMotor.class, "BL");
        Shooter = myOpMode.hardwareMap.get(DcMotor.class, "Sh");
        Harvester = myOpMode.hardwareMap.get(DcMotor.class, "Ha");
        beaconPusher = myOpMode.hardwareMap.servo.get("beaconPush");
        gyro = (ModernRoboticsI2cGyro)myOpMode.hardwareMap.gyroSensor.get("gyro");
        beaconSensor = myOpMode.hardwareMap.colorSensor.get("beaconSense");
        beaconSensor.setI2cAddress(I2cAddr.create8bit(0x3c));
        beaconPusher.setPosition(rightPos);
        FR.setDirection(DcMotor.Direction.REVERSE); // Positive input rotates counter clockwise
        BR.setDirection(DcMotor.Direction.REVERSE);// Positive input rotates counter clockwise
        FL.setDirection(DcMotor.Direction.FORWARD);
        BL.setDirection(DcMotor.Direction.FORWARD);
        Harvester.setDirection(DcMotor.Direction.REVERSE);
        Shooter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //use RUN_USING_ENCODERS because encoders are installed.
        //setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //ENCODERS NOT CURRENTLY WORKING-----------------------
        setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // Stop all robot motion by setting each axis value to zero
        moveRobot(0,0,0) ;
        gyro.calibrate();
        while(gyro.isCalibrating()){
        }
    }

    public void manualDrive()  {//used in TeleOp
        // In this mode the Left stick moves the robot fwd & back, and Right & Left.
        // The Right stick rotates CCW and CW.

        //  (note: The joystick goes negative when pushed forwards, so negate it)
        setAxial(-myOpMode.gamepad1.left_stick_y);
        setLateral(myOpMode.gamepad1.left_stick_x);
        setYaw(-myOpMode.gamepad1.right_stick_x);
    }


    /***
     * void moveRobot(double axial, double lateral, double yaw)
     * Set speed levels to motors based on axes requests
     * @param axial     Speed in Fwd Direction
     * @param lateral   Speed in lateral direction (+ve to right)
     * @param yaw       Speed of Yaw rotation.  (+ve is CCW)
     */
    public void moveRobot(double axial, double lateral, double yaw) {
        setAxial(axial);
        setLateral(lateral);
        setYaw(yaw);
        moveRobot();
    }

    /***
     * void moveRobot()
     * This method will calculate the motor speeds required to move the robot according to the
     * speeds that are stored in the three Axis variables: driveAxial, driveLateral, driveYaw.
     * This code is setup for a three wheeled OMNI-drive but it could be modified for any sort of omni drive.
     *
     * The code assumes the following conventions.
     * 1) Positive speed on the Axial axis means move FORWARD.
     * 2) Positive speed on the Lateral axis means move RIGHT.
     * 3) Positive speed on the Yaw axis means rotate COUNTER CLOCKWISE.
     *
     * This convention should NOT be changed.  Any new drive system should be configured to react accordingly.
     */
    public void moveRobot() {
        // calculate required motor speeds to acheive axis motions

        double FRP = driveAxial - driveLateral + driveYaw;
        double FLP = driveAxial + driveLateral - driveYaw;
        double BRP = driveAxial + driveLateral + driveYaw;
        double BLP = driveAxial - driveLateral - driveYaw;

        // normalize all motor speeds so no values exceeds 100%.
        double max = Math.max(Math.abs(FRP), Math.abs(FLP));
        max = Math.max(max, Math.abs(BRP));
        max = Math.max(max,Math.abs(BLP));
        if (max > 1.0)
        {
            FRP /= max;
            FLP /= max;
            BRP /= max;
            BLP /= max;
        }

        // Set drive motor power levels.
        FR.setPower(FRP);
        BR.setPower(BRP);
        FL.setPower(FLP);
        BL.setPower(BLP);

        // Display Telemetry
        myOpMode.telemetry.addData("Axes  ", "A[%+5.2f], L[%+5.2f], Y[%+5.2f]", driveAxial, driveLateral, driveYaw);
        //myOpMode.telemetry.addData("Wheels", "L[%+5.2f], R[%+5.2f], B[%+5.2f]", left, right, back);
    }


    public void setAxial(double axial)      {driveAxial = Range.clip(axial, -1, 1);}
    public void setLateral(double lateral)  {driveLateral = Range.clip(lateral, -1, 1); }
    public void setYaw(double yaw)          {driveYaw = Range.clip(yaw, -1, 1); }


    /***
     * void setMode(DcMotor.RunMode mode ) Set all drive motors to same mode.
     * @param mode    Desired Motor mode.
     */
    public void setMode(DcMotor.RunMode mode ) {
        FR.setMode(mode);
        BR.setMode(mode);
        FL.setMode(mode);
        BL.setMode(mode);
    }
}

