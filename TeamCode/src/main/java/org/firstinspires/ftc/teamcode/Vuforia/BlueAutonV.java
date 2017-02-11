/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode.Vuforia;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This example is designed to show how to identify a target, get the robot's position, and then plan
 * and execute an approach path to the target.
 *
 * This OpMode uses two "utility" classes to abstract (hide) the hardware and navigation GUTs.
 * These are:  Robot_OmniDrive and Robot_Navigation.
 *
 * This LinearOpMode uses basic hardware and nav calls to drive the robot in either manual or auto mode.
 * AutoMode is engaged by pressing and holding the Left Bumper.  Release the Bumper to return to Manual Mode.
 *
 *  *ManualMode* simply uses the joysticks to move the robot in three degrees of freedom.
 *  - Left stick X (translate left and right)
 *  - Left Stick Y (translate forward and backwards)
 *  - Right Stick X (rotate CW and CCW)
 *
 *  *AutoMode* will approach the image target and attempt to reach a position directly in front
 *  of the center line of the image, with a predefined stand-off distance.
 *
 *  To simplify this example, a gyro is NOT used.  Therefore there is no attempt being made to stabilize
 *  strafing motions, or to perform field-centric driving.
 *
 */

@Autonomous(name="Blue Vuforia Auton", group="VAuton")
public class BlueAutonV extends LinearOpMode {

    final double TARGET_DISTANCE =  500;    // Hold robot's center 100 mm from target
    final double TARGET_DISTANCE_CLOSER = 300;
    final int WALL_ANGLE = 0;
    /* Declare OpMode members. */
    Robot_MecanumDrive     robot    = new Robot_MecanumDrive();   // Use Omni-Directional drive system
    Robot_Navigation    nav      = new Robot_Navigation();  // Use Image Tracking library

    ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException{
        telemetry.addData(">","INITIALIZING");
        telemetry.update();
        // Initialize the robot and navigation
        robot.initDrive(this);
        nav.initVuforia(this, robot);

        // Activate Vuforia (this takes a few seconds)
        nav.activateTracking();
        telemetry.addData(">","READY");
        telemetry.update();
        waitForStart();
        shootBalls();
        pointToAngle(50,0.8);
        pointToAngle(50,0.35);
        moveBackwardInches(52,0.4);
        pointToAngle(WALL_ANGLE,0.4);
        pointToAngle(WALL_ANGLE);
        startBackward(0.2);
        while(opModeIsActive() && !nav.targetIsVisible(0)){//BACK UNTIL WE SEE BLUE NEAR
            idle();
        }
        stopDrive();
        Thread.sleep(100);
        runtime.reset();
        //moves us into position in front of beacon
        while(opModeIsActive() && nav.targetsAreVisible() && !nav.cruiseControl(TARGET_DISTANCE_CLOSER) && runtime.seconds() < 3){
            nav.cruiseControl(TARGET_DISTANCE_CLOSER);
            robot.moveRobot();
        }
        //pointToAngle(WALL_ANGLE-2);
        pushBeaconBlue();
        moveForwardInches(36,0.3);
        robot.beaconPusher.setPosition(robot.rightPos);
        Thread.sleep(100);
        pointToAngle(WALL_ANGLE + 90, 0.3);
        pointToAngle(WALL_ANGLE + 90);
        Thread.sleep(100);
        moveBackwardInches(50,0.4);
        Thread.sleep(100);
        pointToAngle(WALL_ANGLE,0.3);
        startBackward(0.2);
        while(opModeIsActive() && !nav.targetIsVisible(2)){
            idle();
        }
        stopDrive();
        runtime.reset();
        while(opModeIsActive() && nav.targetsAreVisible() && !nav.cruiseControl(TARGET_DISTANCE) && runtime.seconds() < 3){
            nav.cruiseControl(TARGET_DISTANCE);
            robot.moveRobot();
        }
        pointToAngle(WALL_ANGLE);
        Thread.sleep(100);
        pushBeaconBlue();
        startForward(.6);
        Thread.sleep(500);
        pointToAngle(WALL_ANGLE + 22,0.3);
        moveForwardInches(48,0.8);
    }

    public void shootBalls() throws InterruptedException{
        robot.Shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.Shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.Shooter.setPower(robot.shooterPower);
        while(robot.Shooter.getCurrentPosition() < robot.shooterRot){
            if(robot.Shooter.getCurrentPosition() > robot.shooterRot / 4 && robot.Shooter.getCurrentPosition() < robot.shooterRot){
                robot.Harvester.setPower(robot.harvesterPower);
            }
            else
                robot.Harvester.setPower(0);
        }
        robot.Shooter.setPower(0);
        runtime.reset();
        robot.Harvester.setPower(1);
        while(opModeIsActive() && runtime.seconds() < 1){
            idle();
        }
        robot.Harvester.setPower(0);
        Thread.sleep(100);
        robot.Shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.Shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.Shooter.setPower(robot.shooterPower);
        while(robot.Shooter.getCurrentPosition() < robot.shooterRot){
            if(robot.Shooter.getCurrentPosition() > robot.shooterRot / 5 && robot.Shooter.getCurrentPosition() < robot.shooterRot){
                robot.Harvester.setPower(robot.harvesterPower);
            }
            else
                robot.Harvester.setPower(0);
        }
        runtime.reset();
        robot.Shooter.setPower(0);
        robot.Harvester.setPower(1);
        while(opModeIsActive() && runtime.seconds() < 1){
            idle();
        }
        robot.Harvester.setPower(0);
        Thread.sleep(200);
        int startPos = robot.Shooter.getCurrentPosition();
        robot.Shooter.setPower(robot.shooterPower);
        while(robot.Shooter.getCurrentPosition() < startPos + 500){
            idle();
        }
        robot.Shooter.setPower(0);
        robot.Harvester.setPower(0);
    }

    public void startBackward(double speed){
        robot.FR.setPower(-speed);
        robot.FL.setPower(-speed);
        robot.BR.setPower(-speed);
        robot.BL.setPower(-speed);
    }

    public void startForward(double speed){
        robot.FR.setPower(speed);
        robot.FL.setPower(speed);
        robot.BR.setPower(speed);
        robot.BL.setPower(speed);
    }

    public void startRight(double speed){
        robot.FR.setPower(-speed);
        robot.FL.setPower(speed);
        robot.BR.setPower(speed);
        robot.BL.setPower(-speed);
    }

    public void startLeft(double speed){
        robot.FR.setPower(speed);
        robot.FL.setPower(-speed);
        robot.BR.setPower(-speed);
        robot.BL.setPower(speed);
    }

    public void stopDrive(){
        robot.FR.setPower(0);
        robot.FL.setPower(0);
        robot.BR.setPower(0);
        robot.BL.setPower(0);
    }

    public void startLeftTurn(double speed){
        robot.FR.setPower(speed);
        robot.FL.setPower(-speed);
        robot.BR.setPower(speed);
        robot.BL.setPower(-speed);
    }

    public void rightTurn(double angle, double speed){
        double target = robot.gyro.getIntegratedZValue() - angle;
        robot.FR.setPower(-speed);
        robot.FL.setPower(speed);
        robot.BR.setPower(-speed);
        robot.BL.setPower(speed);
        while(opModeIsActive() && robot.gyro.getIntegratedZValue() > target){
            idle();
        }
        stopDrive();
    }

    public void leftTurn(double angle, double speed){
        double start = robot.gyro.getHeading();
        double target = robot.gyro.getIntegratedZValue() + angle;
        robot.FR.setPower(speed);
        robot.FL.setPower(-speed);
        robot.BR.setPower(speed);
        robot.BL.setPower(-speed);
        while(opModeIsActive() && robot.gyro.getIntegratedZValue() < target){
            idle();
        }
        stopDrive();
    }


    public void moveRightInches(double inches, double speed){
        double turnSpeed = 0.05;
        double target = inches * robot.strafeTicsPerInch;
        int startHeading = robot.gyro.getIntegratedZValue();
        robot.FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        startRight(speed);
        while(opModeIsActive() && robot.FL.getCurrentPosition() < target){
            if(robot.gyro.getIntegratedZValue() > startHeading + 5) {//If we've turned > 5 degrees CCW, turn CW
                robot.FR.setPower(robot.FR.getPower() - turnSpeed);
                robot.BR.setPower(robot.BR.getPower() - turnSpeed);
                robot.FL.setPower(robot.FL.getPower() + turnSpeed);
                robot.BL.setPower(robot.BL.getPower() + turnSpeed);
            }
            else if(robot.gyro.getIntegratedZValue() < startHeading - 5){//If we've turned > 5 degrees CW, turn CCW
                robot.FR.setPower(robot.FR.getPower() + turnSpeed);
                robot.BR.setPower(robot.BR.getPower() + turnSpeed);
                robot.FL.setPower(robot.FL.getPower() - turnSpeed);
                robot.BL.setPower(robot.BL.getPower() - turnSpeed);
            }
            else
                startRight(speed);
            idle();
        }
        stopDrive();
    }

    public void moveForwardInches(double inches, double speed){
        double target = inches * robot.ticsPerInch;
        robot.FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        startForward(speed);
        while(opModeIsActive() && robot.FL.getCurrentPosition() < target){
            idle();
        }
        stopDrive();
    }

    public void moveBackwardInches(double inches, double speed){
        double target = inches*robot.ticsPerInch;
        robot.FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        startBackward(speed);
        while(opModeIsActive() && robot.FL.getCurrentPosition() > -target){
            idle();
        }
        stopDrive();
    }

    public void pointToAngle(int targetAngle) throws InterruptedException{
        int angle = robot.gyro.getIntegratedZValue();
        if (angle < targetAngle) {//if not turned enough, turn left
            telemetry.addData("ANGLE",angle);
            leftTurn(targetAngle-angle, 0.25);
            Thread.sleep(100);
        }
        else {//if overturned, turn right
            rightTurn(angle-targetAngle, 0.25);
            Thread.sleep(100);
        }
    }

    public void pointToAngle(int targetAngle, double speed) throws InterruptedException{
        int angle = robot.gyro.getIntegratedZValue();
        if (angle < targetAngle) {//if not turned enough, turn left
            telemetry.addData("ANGLE",angle);
            leftTurn(targetAngle-angle, speed);
            Thread.sleep(100);
        }
        else {//if overturned, turn right
            rightTurn(angle-targetAngle, speed);
            Thread.sleep(100);
        }
    }

    public void pushBeaconBlue() throws  InterruptedException{
        if(robot.beaconSensor.red() < robot.beaconSensor.blue()) {
            robot.beaconPusher.setPosition(robot.leftPos);
            runtime.reset();
            startBackward(0.2);
            while(opModeIsActive() && runtime.seconds() < 1) {
                idle();
            }
            stopDrive();
            robot.FR.setPower(-0.5);
            robot.BR.setPower(-0.5);
            while(opModeIsActive() && runtime.seconds() < 1.5){
                idle();
            }
            stopDrive();
        }
        else {
            robot.beaconPusher.setPosition(robot.rightPos);
            runtime.reset();
            startBackward(0.2);
            while(opModeIsActive() && runtime.seconds() < 1) {
                idle();
            }
            stopDrive();
            robot.FL.setPower(-0.5);
            robot.BL.setPower(-0.5);
            while(opModeIsActive() && runtime.seconds() < 1.5){
                idle();
            }
            stopDrive();
        }
        runtime.reset();
    }
}
