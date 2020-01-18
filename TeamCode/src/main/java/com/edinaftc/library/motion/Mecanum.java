package com.edinaftc.library.motion;

import com.edinaftc.library.subsystems.IMU;
import com.edinaftc.library.subsystems.Subsystem;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ThreadPool;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.concurrent.ExecutorService;

public class Mecanum {
    private DcMotor _frontLeft;
    private DcMotor _frontRight;
    private DcMotor _backLeft;
    private DcMotor _backRight;
    private Telemetry _telemetry;
    private ExecutorService imuUpdateExecutor;
    private BNO055IMU imu;
    private boolean imuStarted = false;
    public Orientation angles;

    private double _currentPower = 1.0;

    private Runnable imuUpdateRunnable = () -> {
        while (!Thread.currentThread().isInterrupted()) {
            try {
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            } catch (Throwable t) {
                this._telemetry.addData("Exception running thread 2", "");
                this._telemetry.update();
            }
        }
    };

    //
    // This is our class that we use to drive the robot in autonomous and teleop.  It has a bunch
    // of helper method in it to help us.  The ones that end with the number 2 use the mode
    // run with encoders whilethe others use the run to position.  Except for the move, it just
    // moves the robot with some power.
    // The methods are:
    //
    //  SlideLeft2 - Slide left a certain distance using RUN_WITH_ENCODERS
    //  Move - moves the robot with a certain power
    //  SlideRightRunWithEncoders - slide right a certain distance using RUN_WITH_ENCODERS
    //  MoveForwardRunToPosition - move forward a certain distance using RUN_TO_POSITION
    //  MoveForwardRunWithEncoders - move forward a certain distance using RUN_WITH_ENCODERS
    //  MoveBackwardsRunToPosition - move backwards a certain distance using RUN_TO_POSITION
    //  MoveBackwardsRunWithEncoders - move backwards a certain distance using RUN_WITH_ENCODERS
    //  TurnRight - turn right with RUN_TO_POSITION
    //  TurnLeft - turn left with RUN_TO_POSITION
    //  Stop - stops the motors
    //
    public Mecanum(DcMotor fl, DcMotor fr, DcMotor bl, DcMotor br,
                   Telemetry telemetry)
    {
        _frontLeft = fl;
        _frontRight = fr;
        _backLeft = bl;
        _backRight = br;
        _telemetry = telemetry;

        _backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        _frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        _backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        _frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        _backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        _frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void enableIMU(BNO055IMU imu, LinearOpMode opMode) {
        this.imu = imu;
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        this.imu.initialize(parameters);

        while (!imu.isGyroCalibrated()) {
            opMode.idle();
        }

        imuUpdateExecutor = ThreadPool.newSingleThreadExecutor("subsystem update");
    }

    public void startIMU() {
        if (!imuStarted) {
            imuUpdateExecutor.submit(imuUpdateRunnable);
            imuStarted = true;
        }
    }

    public void stopIMU() {
        if (imuStarted) {
            imuUpdateExecutor.shutdownNow();
            imuStarted = false;
        }
    }

    public void SlideLeftRunWithEncoders(double power, int distance, LinearOpMode opMode) {
        // put the motors into run with encoders so they run with even power
        StopResetEncodersRunWithEncoderAndBrakekOn();

        int currentPosition =  Math.abs(_backRight.getCurrentPosition());
        int error = Math.abs((int)(distance * 0.95));
        Move(-power, power, power, -power);

        // keep moving until we get close and the op mode is active.  close is 95% of what we want to get to
        while ((currentPosition < error) && opMode.opModeIsActive()) {
            currentPosition =  Math.abs(_backRight.getCurrentPosition());
            opMode.idle();
        }

        Stop();
    }

    public void SlideRightRunWithEncoders(double power, int distance, LinearOpMode opMode) {
        // put the motors into run with encoders so they run with even power
        StopResetEncodersRunWithEncoderAndBrakekOn();

        int currentPosition =  Math.abs(_frontRight.getCurrentPosition());
        int error = Math.abs((int)(distance * 0.95));
        Move(power, -power, -power, power);

        // keep moving until we get close and the op mode is active.  close is 95% of what we want to get to
        while ((currentPosition < error) && opMode.opModeIsActive()) {
            currentPosition =  Math.abs(_frontRight.getCurrentPosition());
            opMode.idle();
        }

        Stop();
    }

    public void MoveForwardRunWithEncoders(double power, int distance, LinearOpMode opMode) {
        // put the motors into run with encoders so they run with even power
        StopResetEncodersRunWithEncoderAndBrakekOn();

        int error = Math.abs((int)(distance * 0.95));
        int currentPosition =  Math.abs(_frontRight.getCurrentPosition());
        double currentPower = CalculateRampPower(power, distance, currentPosition);
        Move(currentPower, currentPower, currentPower, currentPower);

        while ((currentPosition < error) && opMode.opModeIsActive()) {
            currentPosition =  Math.abs(_frontRight.getCurrentPosition());
            currentPower = CalculateRampPower(power, distance, currentPosition);
            Move(currentPower, currentPower, currentPower, currentPower);
            opMode.idle();
        }

        Stop();
    }

    public void MoveBackwardsRunWithEncoders(double power, int distance, LinearOpMode opMode) {
        // put the motors into run with encoders so they run with even power
        StopResetEncodersRunWithEncoderAndBrakekOn();

        int error = Math.abs((int)(distance * 0.95));
        int currentPosition =  Math.abs(_frontRight.getCurrentPosition());

        // ramp up and down the motor speed based on current position
        double currentPower = CalculateRampPower(power, distance, currentPosition);

        Move(-currentPower, -currentPower, -currentPower, -currentPower);

        // keep moving until we get close and the op mode is active.  close is 95% of what we want to get to
        while ((currentPosition < error) && opMode.opModeIsActive()) {
            currentPosition =  Math.abs(_frontRight.getCurrentPosition());
            currentPower = CalculateRampPower(power, distance, currentPosition);
            Move(-currentPower, -currentPower, -currentPower, -currentPower);
            opMode.idle();
        }

        Stop();
    }

    public void TurnRightRunWithEncoders(double power, int distance, LinearOpMode opMode) {
        StopResetEncodersRunWithEncoderAndBrakekOn();

        int error = Math.abs((int)(distance * 0.95));
        int currentPosition =  Math.abs(_frontRight.getCurrentPosition());

        // ramp up and down the motor speed based on current position
        double currentPower = CalculateRampPower(power, distance, currentPosition);

        Move(currentPower, -currentPower, currentPower, -currentPower);

        // keep moving until we get close and the op mode is active.  close is 95% of what we want to get to
        while ((currentPosition < error) && opMode.opModeIsActive()) {
            currentPosition =  Math.abs(_frontRight.getCurrentPosition());
            currentPower = CalculateRampPower(power, distance, currentPosition);
            Move(currentPower, -currentPower, currentPower, -currentPower);
            opMode.idle();
        }

        Stop();
    }

    public void TurnLeftRunWithEncoders(double power, int distance, LinearOpMode opMode) {
        StopResetEncodersRunWithEncoderAndBrakekOn();

        int error = Math.abs((int)(distance * 0.95));
        int currentPosition =  Math.abs(_frontRight.getCurrentPosition());

        // ramp up and down the motor speed based on current position
        double currentPower = CalculateRampPower(power, distance, currentPosition);

        Move(-currentPower, currentPower, -currentPower, currentPower);

        // keep moving until we get close and the op mode is active.  close is 95% of what we want to get to
        while ((currentPosition < error) && opMode.opModeIsActive()) {
            currentPosition =  Math.abs(_frontRight.getCurrentPosition());
            currentPower = CalculateRampPower(power, distance, currentPosition);
            Move(-currentPower, currentPower, -currentPower, currentPower);
            opMode.idle();
        }

        Stop();
    }

    public void MoveForwardRunToPosition(double power, int distance, LinearOpMode opMode) {
        // run with simple distance encoders as moving forward or backwards
        SetDistance(distance, distance, distance, distance);
        StopResetEncodersAndRunToPosition();

        int error = Math.abs((int)(distance * 0.95));
        int currentPosition =  Math.abs(_frontRight.getCurrentPosition());
        Move(power, power, power, power);

        // keep moving until we get close and the op mode is active.  close is 95% of what we want to get to
        while (_frontLeft.isBusy() && _frontRight.isBusy() && _backLeft.isBusy() && _backRight.isBusy() && (currentPosition < error) && opMode.opModeIsActive()) {
            currentPosition =  Math.abs(_frontRight.getCurrentPosition());
            opMode.idle();
        }

        Stop();
    }

    public void MoveBackwardsRunToPosition(double power, int distance, LinearOpMode opMode) {
        // run with simple distance encoders as moving forward or backwards
        SetDistance(-distance, -distance, -distance, -distance);
        StopResetEncodersAndRunToPosition();

        int error = Math.abs((int)(distance * 0.95));
        int currentPosition =  Math.abs(_frontRight.getCurrentPosition());
        Move(power, power, power, power);

        // keep moving until we get close and the op mode is active.  close is 95% of what we want to get to
        while (_frontLeft.isBusy() && _frontRight.isBusy() && _backLeft.isBusy() && _backRight.isBusy() && (currentPosition < error) && opMode.opModeIsActive()) {
            currentPosition =  Math.abs(_frontRight.getCurrentPosition());
            opMode.idle();
        }

        Stop();
    }

    public void TurnRightRunToPosition(double power, int distance, LinearOpMode opMode) {
        // run with simple distance encoders as moving forward or backwards
        SetDistance(distance, distance, -distance, -distance);
        StopResetEncodersAndRunToPosition();

        int error = Math.abs((int)(distance * 0.95));
        int currentPosition =  Math.abs(_frontRight.getCurrentPosition());
        Move(power, power, power, power);

        // keep moving until we get close and the op mode is active.  close is 95% of what we want to get to
        while (_frontLeft.isBusy() && _frontRight.isBusy() && _backLeft.isBusy() && _backRight.isBusy() && (currentPosition < error) && opMode.opModeIsActive()) {
            currentPosition =  Math.abs(_frontRight.getCurrentPosition());
            opMode.idle();
        }

        Stop();
    }

    public void TurnLeftRunToPosition(double power, int distance, LinearOpMode opMode) {
        // run with simple distance encoders as moving forward or backwards
        SetDistance(-distance, -distance, distance, distance);
        StopResetEncodersAndRunToPosition();

        int error = Math.abs((int)(distance * 0.95));
        int currentPosition =  Math.abs(_frontRight.getCurrentPosition());
        Move(power, power, power, power);

        // keep moving until we get close and the op mode is active.  close is 95% of what we want to get to
        while (_frontLeft.isBusy() && _frontRight.isBusy() && _backLeft.isBusy() && _backRight.isBusy() && (currentPosition < error) && opMode.opModeIsActive()) {
            currentPosition =  Math.abs(_frontRight.getCurrentPosition());
            opMode.idle();
        }

        Stop();
    }

    public void SlideRightRunToPosition(double power, int distance, LinearOpMode opMode) {
        // put the motors into run with encoders so they run with even power

        SetDistance(distance, -distance, -distance, distance);
        StopResetEncodersAndRunToPosition();

        int error = Math.abs((int)(distance * 0.95));
        int currentPosition =  Math.abs(_frontRight.getCurrentPosition());
        Move(power, power, power, power);

        // keep moving until we get close and the op mode is active.  close is 95% of what we want to get to
        while (_frontLeft.isBusy() && _frontRight.isBusy() && _backLeft.isBusy() && _backRight.isBusy() && (currentPosition < error) && opMode.opModeIsActive()) {
            currentPosition =  Math.abs(_frontRight.getCurrentPosition());
            opMode.idle();
        }

        Stop();
    }

    public void SlideLeftRunToPosition(double power, int distance, LinearOpMode opMode) {
        // put the motors into run with encoders so they run with even power

        SetDistance(-distance, distance, distance, -distance);
        StopResetEncodersAndRunToPosition();

        int error = Math.abs((int)(distance * 0.95));
        int currentPosition =  Math.abs(_frontRight.getCurrentPosition());
        Move(power, power, power, power);

        // keep moving until we get close and the op mode is active.  close is 95% of what we want to get to
        while (_frontLeft.isBusy() && _frontRight.isBusy() && _backLeft.isBusy() && _backRight.isBusy() && (currentPosition < error) && opMode.opModeIsActive()) {
            currentPosition =  Math.abs(_frontRight.getCurrentPosition());
            opMode.idle();
        }

        Stop();
    }

    public void DiagonalRightAndUpRunToPosition(double power, int distance, LinearOpMode opMode) {
        // put the motors into run with encoders so they run with even power

        SetDistance(distance, distance, distance, distance);
        StopResetEncodersAndRunToPosition();

        int error = Math.abs((int)(distance * 0.95));
        int currentPosition =  Math.abs(_frontLeft.getCurrentPosition());
        Move(power, 0, 0, power);

        // keep moving until we get close and the op mode is active.  close is 95% of what we want to get to
        while (_frontLeft.isBusy() && _frontRight.isBusy() && _backLeft.isBusy() && _backRight.isBusy() && (currentPosition < error) && opMode.opModeIsActive()) {
            currentPosition =  Math.abs(_frontLeft.getCurrentPosition());
            opMode.idle();
        }

        Stop();
    }

    public void DiagonalRightAndDownRunToPosition(double power, int distance, LinearOpMode opMode) {
        // put the motors into run with encoders so they run with even power

        SetDistance(-distance, -distance, -distance, -distance);
        StopResetEncodersAndRunToPosition();

        int error = Math.abs((int)(distance * 0.95));
        int currentPosition =  Math.abs(_frontRight.getCurrentPosition());
        Move(0, power, power, 0);

        // keep moving until we get close and the op mode is active.  close is 95% of what we want to get to
        while (_frontLeft.isBusy() && _frontRight.isBusy() && _backLeft.isBusy() && _backRight.isBusy() && (currentPosition < error) && opMode.opModeIsActive()) {
            currentPosition =  Math.abs(_frontRight.getCurrentPosition());
            opMode.idle();
        }

        Stop();
    }

    public void DiagonalLeftAndDownRunToPosition(double power, int distance, LinearOpMode opMode) {
        // put the motors into run with encoders so they run with even power

        SetDistance(-distance, -distance, -distance, -distance);
        StopResetEncodersAndRunToPosition();

        int error = Math.abs((int)(distance * 0.95));
        int currentPosition =  Math.abs(_frontLeft.getCurrentPosition());
        Move(power, 0, 0, power);

        // keep moving until we get close and the op mode is active.  close is 95% of what we want to get to
        while (_frontLeft.isBusy() && _frontRight.isBusy() && _backLeft.isBusy() && _backRight.isBusy() && (currentPosition < error) && opMode.opModeIsActive()) {
            currentPosition =  Math.abs(_frontLeft.getCurrentPosition());
            opMode.idle();
        }

        Stop();
    }

    public void DiagonalLeftAndUpRunToPosition(double power, int distance, LinearOpMode opMode) {
        // put the motors into run with encoders so they run with even power

        SetDistance(distance, distance, distance, distance);
        StopResetEncodersAndRunToPosition();

        int error = Math.abs((int)(distance * 0.95));
        int currentPosition =  Math.abs(_frontRight.getCurrentPosition());
        Move(0, power, power, 0);

        // keep moving until we get close and the op mode is active.  close is 95% of what we want to get to
        while (_frontLeft.isBusy() && _frontRight.isBusy() && _backLeft.isBusy() && _backRight.isBusy() && (currentPosition < error) && opMode.opModeIsActive()) {
            currentPosition =  Math.abs(_frontRight.getCurrentPosition());
            opMode.idle();
        }

        Stop();
    }

    public void MoveForwardRunWithEncodersAndIMU(double power, int distance, double correctionPower,
                                                 float targetAngle, LinearOpMode opMode,
                                                 Telemetry telemetry) {
        // put the motors into run with encoders so they run with even power
        StopResetEncodersRunWithEncoderAndBrakekOn();

        int error = Math.abs((int)(distance * 0.95));
        int currentPosition =  Math.abs(_frontRight.getCurrentPosition());
        double leftPower, rightPower;
        leftPower = rightPower = CalculateRampPower(power, distance, currentPosition);
        Move(leftPower, rightPower, leftPower, rightPower);

        while ((currentPosition < error) && opMode.opModeIsActive()) {
            float currentAngle = angles.firstAngle;
            float difference = Math.abs(targetAngle - Math.abs(currentAngle));

            currentPosition =  Math.abs(_frontRight.getCurrentPosition());
            leftPower = rightPower = CalculateRampPower(power, distance, currentPosition);

            if (difference > 2) {
                if (targetAngle > currentAngle) {
                    // turn right
                    // decrease right power
                    leftPower -= difference / 10 * correctionPower;
                } else if (targetAngle < currentAngle) {
                    // turn left
                    // decrease left power
                    rightPower -= difference / 10 * correctionPower;
                }
            }

            Move(leftPower, rightPower, leftPower, rightPower);

            telemetry.addData("left, right, start, current", "%f %f %f %f", leftPower,
                    rightPower, targetAngle, currentAngle);
            telemetry.update();
            opMode.idle();
        }

        Stop();
    }

    public void MoveBackwardRunWithEncodersAndIMU(double power, int distance, double correctionPower,
                                                  float targetAngle, LinearOpMode opMode,
                                                  Telemetry telemetry) {
        // put the motors into run with encoders so they run with even power
        StopResetEncodersRunWithEncoderAndBrakekOn();

        int error = Math.abs((int)(distance * 0.95));
        int currentPosition =  Math.abs(_frontRight.getCurrentPosition());
        double leftPower, rightPower;
        leftPower = rightPower = CalculateRampPower(power, distance, currentPosition);
        Move(-leftPower, -rightPower, -leftPower, -rightPower);

        while ((currentPosition < error) && opMode.opModeIsActive()) {
            float currentAngle = angles.firstAngle;
            float difference = Math.abs(targetAngle - Math.abs(currentAngle));

            currentPosition =  Math.abs(_frontRight.getCurrentPosition());
            leftPower = rightPower = CalculateRampPower(power, distance, currentPosition);

            if (difference > 2) {
                if (targetAngle > currentAngle) {
                    // turn right
                    // decrease right power
                    rightPower -= difference / 10 * correctionPower;
                } else if (targetAngle < currentAngle) {
                    // turn left
                    // decrease left power
                    leftPower -= difference / 10 * correctionPower;
                }
            }

            Move(-leftPower, -rightPower, -leftPower, -rightPower);

            telemetry.addData("left, right, start, current", "%f %f %f %f", leftPower,
                    rightPower, targetAngle, currentAngle);
            telemetry.update();
            opMode.idle();
        }

        Stop();
    }

    public void TurnToHeading(double target_heading, double speedModifier, Telemetry telemetry) {
        boolean goRight;
        double currentHeading;
        double degreesToTurn;
        double wheelPower;
        double prevHeading = 0;
        ElapsedTime timeoutTimer = new ElapsedTime();

        double wheel_encoder_ticks = 2400;
        double wheel_diameter = 2.3622;  // size of wheels
        double ticks_per_inch = wheel_encoder_ticks / (wheel_diameter * Math.PI);

        currentHeading = readCurrentHeading();
        degreesToTurn = Math.abs(target_heading - currentHeading);

        goRight = target_heading > currentHeading;

        if (degreesToTurn > 180) {
            goRight = !goRight;
            degreesToTurn = 360 - degreesToTurn;
        }

        timeoutTimer.reset();
        prevHeading = currentHeading;
        while (degreesToTurn > .5 && timeoutTimer.seconds() < 2) {  // 11/21 changed from .5 to .3

            if (speedModifier < 0) {
                wheelPower = (Math.pow((degreesToTurn + 25) / -speedModifier, 3) + 15) / 100;
            } else {
                if (speedModifier != 0) {
                    wheelPower = (Math.pow((degreesToTurn) / speedModifier, 4) + 35) / 100;
                } else {
                    wheelPower = (Math.pow((degreesToTurn) / 30, 4) + 15) / 100;
                }
            }

            if (goRight) {
                wheelPower = -wheelPower;
            }

            telemetry.addData("degrees, timeout, power", "%f %f %f", degreesToTurn, timeoutTimer.milliseconds(), wheelPower);
            telemetry.update();

            Move(-wheelPower, wheelPower, -wheelPower, wheelPower);

            currentHeading = readCurrentHeading();

            degreesToTurn = Math.abs(target_heading - currentHeading);       // Calculate how far is remaining to turn

            goRight = target_heading > currentHeading;

            if (degreesToTurn > 180) {
                goRight = !goRight;
                degreesToTurn = 360 - degreesToTurn;
            }

            if (Math.abs(currentHeading - prevHeading) > 1) {  // if it has turned at least one degree
                timeoutTimer.reset();
                prevHeading = currentHeading;
            }

        }

        Move(0, 0, 0, 0);

        telemetry.addData("Heading: ", currentHeading);
        telemetry.update();

    }

    private double readCurrentHeading() {
        double currentHeading;
        currentHeading = angles.firstAngle;
        if (currentHeading < 0) {
            currentHeading = -currentHeading;
        } else {
            currentHeading = 360 - currentHeading;
        }
        return currentHeading;
    }


    public void Move(double left, double right){
        _frontLeft.setPower(left);
        _frontRight.setPower(right);
        _backLeft.setPower(left);
        _backRight.setPower(right);
    }

    public void Move(double fl, double fr, double bl, double br) {
        _frontLeft.setPower(fl);
        _frontRight.setPower(fr);
        _backLeft.setPower(bl);
        _backRight.setPower(br);
    }

    public void Stop() {
        _frontLeft.setPower(0);
        _frontRight.setPower(0);
        _backLeft.setPower(0);
        _backRight.setPower(0);
    }

    public void setCurrentPower(double cp){
        _currentPower = Math.min(1.0, Math.abs(cp));
    }

    //
    // This method will calculate a power based on the current position and our maximum distance.
    // This a very simple motion profile method that uses distance to figure out different power
    // levels. More complex systems use derivitaves and time.
    // We want to ramp up the speed to a flat maxPower and then ramp down to zero.  We did this
    // to prevent the robot from losing traction and jerking.  We only use this when the robot motors are in
    // the run with encoders mode as the internal PID is being used for that instead of helping us
    // with distance.  So this is our real simple PID.  To learn more about what PID is, visit
    // https://en.wikipedia.org/wiki/PID_controller and to learn more about motion profiling visit
    //
    //
    private double CalculateRampPower(double maxPower, int distance, double currentDistance) {
        // out cutoffs on distance for this step up are:
        //  0-10% - .6 of power
        // 10-20% - .85 of power
        // 20-80% - full power
        // 80-90% - .85 power
        // 90-100% - .6 power
        if (currentDistance <= (distance * .10)) {
            return .6 * maxPower;
        } else if (currentDistance <= (distance * .20)) {
            return  .85 * maxPower;
        } else if (currentDistance <= (distance * .80)) {
            return maxPower;
        } else if (currentDistance <= (distance * .90)) {
            return .85 * maxPower;
        } else {
            return .6 * maxPower;
        }
    }

    //
    // This is our simple drive method that allows us to drive the robot in teleop
    //
    public void Drive(double leftStickX, double leftStickY, double rightStickY) {
        if (_frontRight.getMode() != DcMotor.RunMode.RUN_USING_ENCODER) {
            _frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            _frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            _backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            _backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        final double x = Math.pow(-leftStickX, 3.0);
        final double y = Math.pow(leftStickY, 3.0);

        final double rotation = Math.pow(-rightStickY, 3.0);
        final double direction = Math.atan2(x, y);
        final double speed = Math.min(1.0, Math.sqrt(x * x + y * y));

        final double fl = speed * Math.sin(direction + Math.PI / 4.0) + rotation;
        final double fr = speed * Math.cos(direction + Math.PI / 4.0) - rotation;
        final double bl = speed * Math.cos(direction + Math.PI / 4.0) + rotation;
        final double br = speed * Math.sin(direction + Math.PI / 4.0) - rotation;

        _frontLeft.setPower(-fl * _currentPower);
        _frontRight.setPower(-fr * _currentPower);
        _backLeft.setPower(-bl * _currentPower);
        _backRight.setPower(-br * _currentPower);
    }

    public DcMotor getFL() {
        return _frontLeft;
    }

    public DcMotor getFR() {
        return _frontRight;
    }

    public DcMotor getBL() {
        return _backLeft;
    }

    public DcMotor getBR() {
        return _backRight;
    }

    public void assistedDrive(double x, double y, double rot, double heading){

//        Reversing the polarities because of the way joysticks work
        y *= -1;
        rot *= -1;
        rot = (rot > 1)? 1: (rot < -1)? -1 : rot;

//        The + Math.PI / 4.0 is to account for the strafing wheels
        final double direction = Math.atan2(y, x) + Math.PI / 4.0;

//        The local direction is to account for the direction the robot is pointing in relative to the field
        final double localDirection = Math.toRadians(heading) - direction;
        final double speed = Math.min(1.0, Math.sqrt(x * x + y * y));

//        motorMax is to have a max requested power value that the code can
//        use to divert power from moving to rotate as well without
//        going past the max limit for power
        final double motorMax = (speed + Math.abs(rot));

//        Setting moving values while regulating them to save power for rotation
        double fl = ((motorMax - Math.abs(rot)) / motorMax) * speed * Math.sin(localDirection);
        double fr = ((motorMax - Math.abs(rot)) / motorMax) * speed * Math.cos(localDirection);
        double bl = ((motorMax - Math.abs(rot)) / motorMax) * speed * Math.cos(localDirection);
        double br = ((motorMax - Math.abs(rot)) / motorMax) * speed * Math.sin(localDirection);

//        Adding rotation values while regulating them to save power for moving
        fl += ((motorMax - speed) / motorMax) * rot;
        fr -= ((motorMax - speed) / motorMax) * rot;
        bl += ((motorMax - speed) / motorMax) * rot;
        br -= ((motorMax - speed) / motorMax) * rot;

        fl *= _currentPower;
        fr *= _currentPower;
        bl *= _currentPower;
        br *= _currentPower;

//        Capping the values to keep them from going over max speed
        fl = fl > 1? 1 : fl < -1? -1 : fl;
        fr = fr > 1? 1 : fr < -1? -1 : fr;
        bl = bl > 1? 1 : bl < -1? -1 : bl;
        br = br > 1? 1 : br < -1? -1 : br;

        _frontLeft.setPower(-fl);
        _frontRight.setPower(-fr);
        _backLeft.setPower(-bl);
        _backRight.setPower(-br);

    }

    //
    // These are our helper method to set the motors to what we need for the other steps
    // They are the three different ways you can run a motor
    //
    public void StopResetEncodersAndRunToPosition() {
        _frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        _frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        _frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        _frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        _frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        _frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        _backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        _backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        _backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        _backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        _backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        _backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void StopResetEncodersRunWithEncoderAndBrakekOn() {
        _frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        _frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        _frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        _frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        _frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        _frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        _backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        _backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        _backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        _backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        _backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        _backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void StopResetEncodersAndRunWithoutEncoders() {
        _frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        _frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        _frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        _frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        _backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        _backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        _backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        _backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    //
    // This method helps us set the distance on all the motors for things like turning and
    // moving forward.
    //
    private void SetDistance(int lf, int lb, int rf, int rb) {
        _frontLeft.setTargetPosition(lf);
        _frontRight.setTargetPosition(rf);
        _backLeft.setTargetPosition(lb);
        _backRight.setTargetPosition(rb);
    }
}
