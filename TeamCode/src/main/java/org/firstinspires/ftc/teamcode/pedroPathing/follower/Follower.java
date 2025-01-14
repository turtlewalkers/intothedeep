//
// Source code recreated from a .class file by IntelliJ IDEA
// (powered by FernFlower decompiler)
//
package org.firstinspires.ftc.teamcode.pedroPathing.follower;
//package com.pedropathing.follower;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.DriveVectorScaler;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;

import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.localization.Localizer;
import com.pedropathing.localization.Pose;
import com.pedropathing.localization.PoseUpdater;
import com.pedropathing.pathgen.BezierPoint;
import com.pedropathing.pathgen.MathFunctions;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathBuilder;
import com.pedropathing.pathgen.PathCallback;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.pathgen.Vector;
import com.pedropathing.util.DashboardPoseTracker;
import com.pedropathing.util.Drawing;
import com.pedropathing.util.FilteredPIDFController;
import com.pedropathing.util.KalmanFilter;
import com.pedropathing.util.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Iterator;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Follower {
    private HardwareMap hardwareMap;
    private DcMotorEx leftFront;
    private DcMotorEx leftRear;
    private DcMotorEx rightFront;
    private DcMotorEx rightRear;
    private List<DcMotorEx> motors;
    private DriveVectorScaler driveVectorScaler;
    public PoseUpdater poseUpdater;
    private DashboardPoseTracker dashboardPoseTracker;
    private Pose closestPose;
    private Path currentPath;
    private PathChain currentPathChain;
    private final int BEZIER_CURVE_BINARY_STEP_LIMIT;
    private final int AVERAGED_VELOCITY_SAMPLE_NUMBER;
    private int chainIndex;
    private long[] pathStartTimes;
    private boolean followingPathChain;
    private boolean holdingPosition;
    private boolean isBusy;
    private boolean reachedParametricPathEnd;
    private boolean holdPositionAtEnd;
    private boolean teleopDrive;
    private double maxPower = 1;
    private double oldMaxPower = 1;
    private double previousSecondaryTranslationalIntegral;
    private double previousTranslationalIntegral;
    private double holdPointTranslationalScaling;
    private double holdPointHeadingScaling;
    public double driveError;
    public double headingError;
    private long reachedParametricPathEndTime;
    private double[] drivePowers;
    private double[] teleopDriveValues;
    private ArrayList<Vector> velocities;
    private ArrayList<Vector> accelerations;
    private double criticalVoltage = 0;
    private double criticalVoltageDrivePower = 0;
    public static double idealVoltage = 12;
    private Vector averageVelocity;
    private Vector averagePreviousVelocity;
    private Vector averageAcceleration;
    private Vector secondaryTranslationalIntegralVector;
    private Vector translationalIntegralVector;
    private Vector teleopDriveVector;
    private Vector teleopHeadingVector;
    public Vector driveVector;
    public Vector headingVector;
    public Vector translationalVector;
    public Vector centripetalVector;
    public Vector correctiveVector;
    private PIDFController secondaryTranslationalPIDF;
    private PIDFController secondaryTranslationalIntegral;
    private PIDFController translationalPIDF;
    private PIDFController translationalIntegral;
    private PIDFController secondaryHeadingPIDF;
    private PIDFController headingPIDF;
    private FilteredPIDFController secondaryDrivePIDF;
    private FilteredPIDFController drivePIDF;
    private KalmanFilter driveKalmanFilter;
    private double[] driveErrors;
    private double rawDriveError;
    private double previousRawDriveError;
    public static boolean drawOnDashboard = true;
    public static boolean useTranslational = true;
    public static boolean useCentripetal = true;
    public static boolean useHeading = true;
    public static boolean useDrive = true;
    VoltageSensor vSensor;

    public Follower(HardwareMap hardwareMap) {
        this.BEZIER_CURVE_BINARY_STEP_LIMIT = FollowerConstants.BEZIER_CURVE_BINARY_STEP_LIMIT;
        this.AVERAGED_VELOCITY_SAMPLE_NUMBER = FollowerConstants.AVERAGED_VELOCITY_SAMPLE_NUMBER;
        this.holdPointTranslationalScaling = FollowerConstants.holdPointTranslationalScaling;
        this.holdPointHeadingScaling = FollowerConstants.holdPointHeadingScaling;
        this.velocities = new ArrayList();
        this.accelerations = new ArrayList();
        this.secondaryTranslationalPIDF = new PIDFController(FollowerConstants.secondaryTranslationalPIDFCoefficients);
        this.secondaryTranslationalIntegral = new PIDFController(FollowerConstants.secondaryTranslationalIntegral);
        this.translationalPIDF = new PIDFController(FollowerConstants.translationalPIDFCoefficients);
        this.translationalIntegral = new PIDFController(FollowerConstants.translationalIntegral);
        this.secondaryHeadingPIDF = new PIDFController(FollowerConstants.secondaryHeadingPIDFCoefficients);
        this.headingPIDF = new PIDFController(FollowerConstants.headingPIDFCoefficients);
        this.secondaryDrivePIDF = new FilteredPIDFController(FollowerConstants.secondaryDrivePIDFCoefficients);
        this.drivePIDF = new FilteredPIDFController(FollowerConstants.drivePIDFCoefficients);
        this.driveKalmanFilter = new KalmanFilter(FollowerConstants.driveKalmanFilterParameters);
        this.hardwareMap = hardwareMap;
        this.initialize();
    }

    public Follower(HardwareMap hardwareMap, Localizer localizer) {
        this.BEZIER_CURVE_BINARY_STEP_LIMIT = FollowerConstants.BEZIER_CURVE_BINARY_STEP_LIMIT;
        this.AVERAGED_VELOCITY_SAMPLE_NUMBER = FollowerConstants.AVERAGED_VELOCITY_SAMPLE_NUMBER;
        this.holdPointTranslationalScaling = FollowerConstants.holdPointTranslationalScaling;
        this.holdPointHeadingScaling = FollowerConstants.holdPointHeadingScaling;
        this.velocities = new ArrayList();
        this.accelerations = new ArrayList();
        this.secondaryTranslationalPIDF = new PIDFController(FollowerConstants.secondaryTranslationalPIDFCoefficients);
        this.secondaryTranslationalIntegral = new PIDFController(FollowerConstants.secondaryTranslationalIntegral);
        this.translationalPIDF = new PIDFController(FollowerConstants.translationalPIDFCoefficients);
        this.translationalIntegral = new PIDFController(FollowerConstants.translationalIntegral);
        this.secondaryHeadingPIDF = new PIDFController(FollowerConstants.secondaryHeadingPIDFCoefficients);
        this.headingPIDF = new PIDFController(FollowerConstants.headingPIDFCoefficients);
        this.secondaryDrivePIDF = new FilteredPIDFController(FollowerConstants.secondaryDrivePIDFCoefficients);
        this.drivePIDF = new FilteredPIDFController(FollowerConstants.drivePIDFCoefficients);
        this.driveKalmanFilter = new KalmanFilter(FollowerConstants.driveKalmanFilterParameters);
        this.hardwareMap = hardwareMap;
        this.initialize(localizer);
    }

    public void initialize() {
        this.poseUpdater = new PoseUpdater(this.hardwareMap);
        this.driveVectorScaler = new DriveVectorScaler(FollowerConstants.frontLeftVector);
        this.leftFront = (DcMotorEx)this.hardwareMap.get(DcMotorEx.class, FollowerConstants.leftFrontMotorName);
        this.leftRear = (DcMotorEx)this.hardwareMap.get(DcMotorEx.class, FollowerConstants.leftRearMotorName);
        this.rightRear = (DcMotorEx)this.hardwareMap.get(DcMotorEx.class, FollowerConstants.rightRearMotorName);
        this.rightFront = (DcMotorEx)this.hardwareMap.get(DcMotorEx.class, FollowerConstants.rightFrontMotorName);
        this.leftFront.setDirection(FollowerConstants.leftFrontMotorDirection);
        this.leftRear.setDirection(FollowerConstants.leftRearMotorDirection);
        this.rightFront.setDirection(FollowerConstants.rightFrontMotorDirection);
        this.rightRear.setDirection(FollowerConstants.rightRearMotorDirection);
        this.vSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");
        this.motors = Arrays.asList(this.leftFront, this.leftRear, this.rightFront, this.rightRear);
        Iterator var1 = this.motors.iterator();

        while(var1.hasNext()) {
            DcMotorEx motor = (DcMotorEx)var1.next();
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
        }

        this.setMotorsToFloat();
        this.dashboardPoseTracker = new DashboardPoseTracker(this.poseUpdater);
        this.breakFollowing();
    }

    public void initialize(Localizer localizer) {
        this.poseUpdater = new PoseUpdater(this.hardwareMap, localizer);
        this.driveVectorScaler = new DriveVectorScaler(FollowerConstants.frontLeftVector);
        this.leftFront = (DcMotorEx)this.hardwareMap.get(DcMotorEx.class, FollowerConstants.leftFrontMotorName);
        this.leftRear = (DcMotorEx)this.hardwareMap.get(DcMotorEx.class, FollowerConstants.leftRearMotorName);
        this.rightRear = (DcMotorEx)this.hardwareMap.get(DcMotorEx.class, FollowerConstants.rightRearMotorName);
        this.rightFront = (DcMotorEx)this.hardwareMap.get(DcMotorEx.class, FollowerConstants.rightFrontMotorName);
        this.leftFront.setDirection(FollowerConstants.leftFrontMotorDirection);
        this.leftRear.setDirection(FollowerConstants.leftRearMotorDirection);
        this.rightFront.setDirection(FollowerConstants.rightFrontMotorDirection);
        this.rightRear.setDirection(FollowerConstants.rightRearMotorDirection);
        this.motors = Arrays.asList(this.leftFront, this.leftRear, this.rightFront, this.rightRear);
        this.vSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");
        Iterator var2 = this.motors.iterator();

        while(var2.hasNext()) {
            DcMotorEx motor = (DcMotorEx)var2.next();
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
        }

        this.setMotorsToFloat();
        this.dashboardPoseTracker = new DashboardPoseTracker(this.poseUpdater);
        this.breakFollowing();
    }

    public void setSlowDownVoltage(double voltage, double setPower) {
        this.criticalVoltage = voltage;
        this.criticalVoltageDrivePower = setPower;
    }

    private void setMotorsToBrake() {
        Iterator var1 = this.motors.iterator();

        while(var1.hasNext()) {
            DcMotorEx motor = (DcMotorEx)var1.next();
            motor.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        }

    }

    private void setMotorsToFloat() {
        Iterator var1 = this.motors.iterator();

        while(var1.hasNext()) {
            DcMotorEx motor = (DcMotorEx)var1.next();
            motor.setZeroPowerBehavior(ZeroPowerBehavior.FLOAT);
        }

    }

    public void setMaxPower(double set) {
        this.driveVectorScaler.setMaxPowerScaling(set);
    }

    public Point getPointFromPath(double t) {
        return this.currentPath != null ? this.currentPath.getPoint(t) : null;
    }

    public Pose getPose() {
        return this.poseUpdater.getPose();
    }

    public void setPose(Pose pose) {
        this.poseUpdater.setPose(pose);
    }

    public Vector getVelocity() {
        return this.poseUpdater.getVelocity();
    }

    public Vector getAcceleration() {
        return this.poseUpdater.getAcceleration();
    }

    public double getVelocityMagnitude() {
        return this.poseUpdater.getVelocity().getMagnitude();
    }

    public void setStartingPose(Pose pose) {
        this.poseUpdater.setStartingPose(pose);
    }

    public void setCurrentPoseWithOffset(Pose set) {
        this.poseUpdater.setCurrentPoseWithOffset(set);
    }

    public void setXOffset(double xOffset) {
        this.poseUpdater.setXOffset(xOffset);
    }

    public void setYOffset(double yOffset) {
        this.poseUpdater.setYOffset(yOffset);
    }

    public void setHeadingOffset(double headingOffset) {
        this.poseUpdater.setHeadingOffset(headingOffset);
    }

    public double getXOffset() {
        return this.poseUpdater.getXOffset();
    }

    public double getYOffset() {
        return this.poseUpdater.getYOffset();
    }

    public double getHeadingOffset() {
        return this.poseUpdater.getHeadingOffset();
    }

    public void resetOffset() {
        this.poseUpdater.resetOffset();
    }

    public void holdPoint(BezierPoint point, double heading) {
        this.breakFollowing();
        this.holdingPosition = true;
        this.isBusy = false;
        this.followingPathChain = false;
        this.currentPath = new Path(point);
        this.currentPath.setConstantHeadingInterpolation(heading);
        this.closestPose = this.currentPath.getClosestPoint(this.poseUpdater.getPose(), 1);
    }

    public void holdPoint(Point point, double heading) {
        this.holdPoint(new BezierPoint(point), heading);
    }

    public void holdPoint(Pose pose) {
        this.holdPoint(new Point(pose), pose.getHeading());
    }

    public void followPath(Path path, boolean holdEnd) {
        this.breakFollowing();
        this.holdPositionAtEnd = holdEnd;
        this.isBusy = true;
        this.followingPathChain = false;
        this.currentPath = path;
        this.closestPose = this.currentPath.getClosestPoint(this.poseUpdater.getPose(), this.BEZIER_CURVE_BINARY_STEP_LIMIT);
    }

    public void followPath(Path path) {
        this.followPath(path, false);
    }

    public void followPath(PathChain pathChain, boolean holdEnd) {
        this.breakFollowing();
        this.holdPositionAtEnd = holdEnd;
        this.pathStartTimes = new long[pathChain.size()];
        this.pathStartTimes[0] = System.currentTimeMillis();
        this.isBusy = true;
        this.followingPathChain = true;
        this.chainIndex = 0;
        this.currentPathChain = pathChain;
        this.currentPath = pathChain.getPath(this.chainIndex);
        this.closestPose = this.currentPath.getClosestPoint(this.poseUpdater.getPose(), this.BEZIER_CURVE_BINARY_STEP_LIMIT);
    }

    public void followPath(PathChain pathChain) {
        this.followPath(pathChain, false);
    }

    public void startTeleopDrive() {
        this.breakFollowing();
        this.teleopDrive = true;
        if (FollowerConstants.useBrakeModeInTeleOp) {
            this.setMotorsToBrake();
        }

    }

    public void updatePose() {
        this.poseUpdater.update();
        if (drawOnDashboard) {
            this.dashboardPoseTracker.update();
        }

    }

    public void update() {
        this.updatePose();
        int i;
        if (!this.teleopDrive) {
            if (this.currentPath != null) {
                if (this.holdingPosition) {
                    double voltage = vSensor.getVoltage();
                    if (voltage <= criticalVoltage) {
                        if (oldMaxPower == 0) {
                            oldMaxPower = maxPower;
                        }
                        setMaxPower(criticalVoltageDrivePower);
                    } else {
                        if (oldMaxPower != 0) {
                            setMaxPower(oldMaxPower);
                            oldMaxPower = 0;
                        }
                    }
                    this.closestPose = this.currentPath.getClosestPoint(this.poseUpdater.getPose(), 1);
                    this.drivePowers = this.driveVectorScaler.getDrivePowers(MathFunctions.scalarMultiplyVector(this.getTranslationalCorrection(), this.holdPointTranslationalScaling), MathFunctions.scalarMultiplyVector(this.getHeadingVector(), this.holdPointHeadingScaling), new Vector(), this.poseUpdater.getPose().getHeading());

                    for(i = 0; i < this.motors.size(); ++i) {
                        if (Math.abs(((DcMotorEx)this.motors.get(i)).getPower() - this.drivePowers[i]) > FollowerConstants.motorCachingThreshold) {
                            ((DcMotorEx)this.motors.get(i)).setPower(this.drivePowers[i] * (idealVoltage / voltage));
                        }
                    }
                } else {
                    if (this.isBusy) {
                        this.closestPose = this.currentPath.getClosestPoint(this.poseUpdater.getPose(), this.BEZIER_CURVE_BINARY_STEP_LIMIT);
                        if (this.followingPathChain) {
                            this.updateCallbacks();
                        }

                        double voltage = vSensor.getVoltage();
                        if (voltage <= criticalVoltage) {
                            if (oldMaxPower == 0) {
                                oldMaxPower = maxPower;
                            }
                            setMaxPower(criticalVoltageDrivePower);
                        } else {
                            if (oldMaxPower != 0) {
                                setMaxPower(oldMaxPower);
                                oldMaxPower = 0;
                            }
                        }

                        this.drivePowers = this.driveVectorScaler.getDrivePowers(this.getCorrectiveVector(), this.getHeadingVector(), this.getDriveVector(), this.poseUpdater.getPose().getHeading());

                        for(i = 0; i < this.motors.size(); ++i) {
                            if (Math.abs(((DcMotorEx)this.motors.get(i)).getPower() - this.drivePowers[i]) > FollowerConstants.motorCachingThreshold) {
                                ((DcMotorEx)this.motors.get(i)).setPower(this.drivePowers[i] * (idealVoltage / voltage));
                            }
                        }
                    }

                    if (this.currentPath.isAtParametricEnd()) {
                        if (this.followingPathChain && this.chainIndex < this.currentPathChain.size() - 1) {
                            this.breakFollowing();
                            this.pathStartTimes[this.chainIndex] = System.currentTimeMillis();
                            this.isBusy = true;
                            this.followingPathChain = true;
                            ++this.chainIndex;
                            this.currentPath = this.currentPathChain.getPath(this.chainIndex);
                            this.closestPose = this.currentPath.getClosestPoint(this.poseUpdater.getPose(), this.BEZIER_CURVE_BINARY_STEP_LIMIT);
                        } else {
                            if (!this.reachedParametricPathEnd) {
                                this.reachedParametricPathEnd = true;
                                this.reachedParametricPathEndTime = System.currentTimeMillis();
                            }

                            if ((double)(System.currentTimeMillis() - this.reachedParametricPathEndTime) > this.currentPath.getPathEndTimeoutConstraint() || this.poseUpdater.getVelocity().getMagnitude() < this.currentPath.getPathEndVelocityConstraint() && MathFunctions.distance(this.poseUpdater.getPose(), this.closestPose) < this.currentPath.getPathEndTranslationalConstraint() && MathFunctions.getSmallestAngleDifference(this.poseUpdater.getPose().getHeading(), this.currentPath.getClosestPointHeadingGoal()) < this.currentPath.getPathEndHeadingConstraint()) {
                                if (this.holdPositionAtEnd) {
                                    this.holdPositionAtEnd = false;
                                    this.holdPoint(new BezierPoint(this.currentPath.getLastControlPoint()), this.currentPath.getHeadingGoal(1.0));
                                } else {
                                    this.breakFollowing();
                                }
                            }
                        }
                    }
                }
            }
        } else {
            this.velocities.add(this.poseUpdater.getVelocity());
            this.velocities.remove(this.velocities.get(this.velocities.size() - 1));
            this.calculateAveragedVelocityAndAcceleration();
            this.drivePowers = this.driveVectorScaler.getDrivePowers(this.getCentripetalForceCorrection(), this.teleopHeadingVector, this.teleopDriveVector, this.poseUpdater.getPose().getHeading());

            for(i = 0; i < this.motors.size(); ++i) {
                if (Math.abs(((DcMotorEx)this.motors.get(i)).getPower() - this.drivePowers[i]) > FollowerConstants.motorCachingThreshold) {
                    ((DcMotorEx)this.motors.get(i)).setPower(this.drivePowers[i]);
                }
            }
        }

    }

    public void setTeleOpMovementVectors(double forwardDrive, double lateralDrive, double heading) {
        this.setTeleOpMovementVectors(forwardDrive, lateralDrive, heading, true);
    }

    public void setTeleOpMovementVectors(double forwardDrive, double lateralDrive, double heading, boolean robotCentric) {
        this.teleopDriveValues[0] = MathFunctions.clamp(forwardDrive, -1.0, 1.0);
        this.teleopDriveValues[1] = MathFunctions.clamp(lateralDrive, -1.0, 1.0);
        this.teleopDriveValues[2] = MathFunctions.clamp(heading, -1.0, 1.0);
        this.teleopDriveVector.setOrthogonalComponents(this.teleopDriveValues[0], this.teleopDriveValues[1]);
        this.teleopDriveVector.setMagnitude(MathFunctions.clamp(this.teleopDriveVector.getMagnitude(), 0.0, 1.0));
        if (robotCentric) {
            this.teleopDriveVector.rotateVector(this.getPose().getHeading());
        }

        this.teleopHeadingVector.setComponents(this.teleopDriveValues[2], this.getPose().getHeading());
    }

    public void calculateAveragedVelocityAndAcceleration() {
        this.averageVelocity = new Vector();
        this.averagePreviousVelocity = new Vector();

        int i;
        for(i = 0; i < this.velocities.size() / 2; ++i) {
            this.averageVelocity = MathFunctions.addVectors(this.averageVelocity, (Vector)this.velocities.get(i));
        }

        this.averageVelocity = MathFunctions.scalarMultiplyVector(this.averageVelocity, 1.0 / ((double)this.velocities.size() / 2.0));

        for(i = this.velocities.size() / 2; i < this.velocities.size(); ++i) {
            this.averagePreviousVelocity = MathFunctions.addVectors(this.averagePreviousVelocity, (Vector)this.velocities.get(i));
        }

        this.averagePreviousVelocity = MathFunctions.scalarMultiplyVector(this.averagePreviousVelocity, 1.0 / ((double)this.velocities.size() / 2.0));
        this.accelerations.add(MathFunctions.subtractVectors(this.averageVelocity, this.averagePreviousVelocity));
        this.accelerations.remove(this.accelerations.size() - 1);
        this.averageAcceleration = new Vector();

        for(i = 0; i < this.accelerations.size(); ++i) {
            this.averageAcceleration = MathFunctions.addVectors(this.averageAcceleration, (Vector)this.accelerations.get(i));
        }

        this.averageAcceleration = MathFunctions.scalarMultiplyVector(this.averageAcceleration, 1.0 / (double)this.accelerations.size());
    }

    public void updateCallbacks() {
        Iterator var1 = this.currentPathChain.getCallbacks().iterator();

        while(true) {
            PathCallback callback;
            do {
                do {
                    while(true) {
                        do {
                            if (!var1.hasNext()) {
                                return;
                            }

                            callback = (PathCallback)var1.next();
                        } while(callback.hasBeenRun());

                        if (callback.getType() == 1) {
                            break;
                        }

                        if (this.chainIndex >= callback.getIndex() && (double)(System.currentTimeMillis() - this.pathStartTimes[callback.getIndex()]) > callback.getStartCondition()) {
                            callback.run();
                        }
                    }
                } while(this.chainIndex != callback.getIndex());
            } while(!(this.getCurrentTValue() >= callback.getStartCondition()) && !MathFunctions.roughlyEquals(this.getCurrentTValue(), callback.getStartCondition()));

            callback.run();
        }
    }

    public void breakFollowing() {
        this.teleopDrive = false;
        this.setMotorsToFloat();
        this.holdingPosition = false;
        this.isBusy = false;
        this.reachedParametricPathEnd = false;
        this.secondaryDrivePIDF.reset();
        this.drivePIDF.reset();
        this.secondaryHeadingPIDF.reset();
        this.headingPIDF.reset();
        this.secondaryTranslationalPIDF.reset();
        this.secondaryTranslationalIntegral.reset();
        this.secondaryTranslationalIntegralVector = new Vector();
        this.previousSecondaryTranslationalIntegral = 0.0;
        this.translationalPIDF.reset();
        this.translationalIntegral.reset();
        this.translationalIntegralVector = new Vector();
        this.previousTranslationalIntegral = 0.0;
        this.driveVector = new Vector();
        this.headingVector = new Vector();
        this.translationalVector = new Vector();
        this.centripetalVector = new Vector();
        this.correctiveVector = new Vector();
        this.driveError = 0.0;
        this.headingError = 0.0;
        this.rawDriveError = 0.0;
        this.previousRawDriveError = 0.0;
        this.driveErrors = new double[2];

        int i;
        for(i = 0; i < this.driveErrors.length; ++i) {
            this.driveErrors[i] = 0.0;
        }

        this.driveKalmanFilter.reset();

        for(i = 0; i < this.AVERAGED_VELOCITY_SAMPLE_NUMBER; ++i) {
            this.velocities.add(new Vector());
        }

        for(i = 0; i < this.AVERAGED_VELOCITY_SAMPLE_NUMBER / 2; ++i) {
            this.accelerations.add(new Vector());
        }

        this.calculateAveragedVelocityAndAcceleration();
        this.teleopDriveValues = new double[3];
        this.teleopDriveVector = new Vector();
        this.teleopHeadingVector = new Vector();

        for(i = 0; i < this.motors.size(); ++i) {
            ((DcMotorEx)this.motors.get(i)).setPower(0.0);
        }

    }

    public boolean isBusy() {
        return this.isBusy;
    }

    public Vector getDriveVector() {
        if (!useDrive) {
            return new Vector();
        } else if (this.followingPathChain && this.chainIndex < this.currentPathChain.size() - 1) {
            return new Vector(this.driveVectorScaler.getMaxPowerScaling(), this.currentPath.getClosestPointTangentVector().getTheta());
        } else {
            this.driveError = this.getDriveVelocityError();
            if (Math.abs(this.driveError) < FollowerConstants.drivePIDFSwitch && FollowerConstants.useSecondaryDrivePID) {
                this.secondaryDrivePIDF.updateError(this.driveError);
                this.driveVector = new Vector(MathFunctions.clamp(this.secondaryDrivePIDF.runPIDF() + FollowerConstants.secondaryDrivePIDFFeedForward * MathFunctions.getSign(this.driveError), -this.driveVectorScaler.getMaxPowerScaling(), this.driveVectorScaler.getMaxPowerScaling()), this.currentPath.getClosestPointTangentVector().getTheta());
                return MathFunctions.copyVector(this.driveVector);
            } else {
                this.drivePIDF.updateError(this.driveError);
                this.driveVector = new Vector(MathFunctions.clamp(this.drivePIDF.runPIDF() + FollowerConstants.drivePIDFFeedForward * MathFunctions.getSign(this.driveError), -this.driveVectorScaler.getMaxPowerScaling(), this.driveVectorScaler.getMaxPowerScaling()), this.currentPath.getClosestPointTangentVector().getTheta());
                return MathFunctions.copyVector(this.driveVector);
            }
        }
    }

    public double getDriveVelocityError() {
        double distanceToGoal;
        Vector distanceToGoalVector;
        if (!this.currentPath.isAtParametricEnd()) {
            distanceToGoal = this.currentPath.length() * (1.0 - this.currentPath.getClosestPointTValue());
        } else {
            distanceToGoalVector = new Vector();
            distanceToGoalVector.setOrthogonalComponents(this.getPose().getX() - this.currentPath.getLastControlPoint().getX(), this.getPose().getY() - this.currentPath.getLastControlPoint().getY());
            distanceToGoal = MathFunctions.dotProduct(this.currentPath.getEndTangent(), distanceToGoalVector);
        }

        distanceToGoalVector = MathFunctions.scalarMultiplyVector(MathFunctions.normalizeVector(this.currentPath.getClosestPointTangentVector()), distanceToGoal);
        Vector velocity = new Vector(MathFunctions.dotProduct(this.getVelocity(), MathFunctions.normalizeVector(this.currentPath.getClosestPointTangentVector())), this.currentPath.getClosestPointTangentVector().getTheta());
        Vector forwardHeadingVector = new Vector(1.0, this.poseUpdater.getPose().getHeading());
        double forwardVelocity = MathFunctions.dotProduct(forwardHeadingVector, velocity);
        double forwardDistanceToGoal = MathFunctions.dotProduct(forwardHeadingVector, distanceToGoalVector);
        double forwardVelocityGoal = MathFunctions.getSign(forwardDistanceToGoal) * Math.sqrt(Math.abs(-2.0 * this.currentPath.getZeroPowerAccelerationMultiplier() * FollowerConstants.forwardZeroPowerAcceleration * (double)(forwardVelocity < 0.0 ? -1 : 1) * forwardDistanceToGoal));
        double forwardVelocityZeroPowerDecay = forwardVelocity - MathFunctions.getSign(forwardDistanceToGoal) * Math.sqrt(Math.abs(Math.pow(forwardVelocity, 2.0) + 2.0 * FollowerConstants.forwardZeroPowerAcceleration * (double)(forwardVelocity < 0.0 ? -1 : 1) * forwardDistanceToGoal));
        Vector lateralHeadingVector = new Vector(1.0, this.poseUpdater.getPose().getHeading() - 1.5707963267948966);
        double lateralVelocity = MathFunctions.dotProduct(lateralHeadingVector, velocity);
        double lateralDistanceToGoal = MathFunctions.dotProduct(lateralHeadingVector, distanceToGoalVector);
        double lateralVelocityGoal = MathFunctions.getSign(lateralDistanceToGoal) * Math.sqrt(Math.abs(-2.0 * this.currentPath.getZeroPowerAccelerationMultiplier() * FollowerConstants.lateralZeroPowerAcceleration * (double)(lateralVelocity < 0.0 ? -1 : 1) * lateralDistanceToGoal));
        double lateralVelocityZeroPowerDecay = lateralVelocity - MathFunctions.getSign(lateralDistanceToGoal) * Math.sqrt(Math.abs(Math.pow(lateralVelocity, 2.0) + 2.0 * FollowerConstants.lateralZeroPowerAcceleration * (double)(lateralVelocity < 0.0 ? -1 : 1) * lateralDistanceToGoal));
        Vector forwardVelocityError = new Vector(forwardVelocityGoal - forwardVelocityZeroPowerDecay - forwardVelocity, forwardHeadingVector.getTheta());
        Vector lateralVelocityError = new Vector(lateralVelocityGoal - lateralVelocityZeroPowerDecay - lateralVelocity, lateralHeadingVector.getTheta());
        Vector velocityErrorVector = MathFunctions.addVectors(forwardVelocityError, lateralVelocityError);
        this.previousRawDriveError = this.rawDriveError;
        this.rawDriveError = velocityErrorVector.getMagnitude() * MathFunctions.getSign(MathFunctions.dotProduct(velocityErrorVector, this.currentPath.getClosestPointTangentVector()));
        double projection = 2.0 * this.driveErrors[1] - this.driveErrors[0];
        this.driveKalmanFilter.update(this.rawDriveError - this.previousRawDriveError, projection);

        for(int i = 0; i < this.driveErrors.length - 1; ++i) {
            this.driveErrors[i] = this.driveErrors[i + 1];
        }

        this.driveErrors[1] = this.driveKalmanFilter.getState();
        return this.driveKalmanFilter.getState();
    }

    public Vector getHeadingVector() {
        if (!useHeading) {
            return new Vector();
        } else {
            this.headingError = MathFunctions.getTurnDirection(this.poseUpdater.getPose().getHeading(), this.currentPath.getClosestPointHeadingGoal()) * MathFunctions.getSmallestAngleDifference(this.poseUpdater.getPose().getHeading(), this.currentPath.getClosestPointHeadingGoal());
            if (Math.abs(this.headingError) < FollowerConstants.headingPIDFSwitch && FollowerConstants.useSecondaryHeadingPID) {
                this.secondaryHeadingPIDF.updateError(this.headingError);
                this.headingVector = new Vector(MathFunctions.clamp(this.secondaryHeadingPIDF.runPIDF() + FollowerConstants.secondaryHeadingPIDFFeedForward * MathFunctions.getTurnDirection(this.poseUpdater.getPose().getHeading(), this.currentPath.getClosestPointHeadingGoal()), -this.driveVectorScaler.getMaxPowerScaling(), this.driveVectorScaler.getMaxPowerScaling()), this.poseUpdater.getPose().getHeading());
                return MathFunctions.copyVector(this.headingVector);
            } else {
                this.headingPIDF.updateError(this.headingError);
                this.headingVector = new Vector(MathFunctions.clamp(this.headingPIDF.runPIDF() + FollowerConstants.headingPIDFFeedForward * MathFunctions.getTurnDirection(this.poseUpdater.getPose().getHeading(), this.currentPath.getClosestPointHeadingGoal()), -this.driveVectorScaler.getMaxPowerScaling(), this.driveVectorScaler.getMaxPowerScaling()), this.poseUpdater.getPose().getHeading());
                return MathFunctions.copyVector(this.headingVector);
            }
        }
    }

    public Vector getCorrectiveVector() {
        Vector centripetal = this.getCentripetalForceCorrection();
        Vector translational = this.getTranslationalCorrection();
        Vector corrective = MathFunctions.addVectors(centripetal, translational);
        if (corrective.getMagnitude() > this.driveVectorScaler.getMaxPowerScaling()) {
            return MathFunctions.addVectors(centripetal, MathFunctions.scalarMultiplyVector(translational, this.driveVectorScaler.findNormalizingScaling(centripetal, translational)));
        } else {
            this.correctiveVector = MathFunctions.copyVector(corrective);
            return corrective;
        }
    }

    public Vector getTranslationalCorrection() {
        if (!useTranslational) {
            return new Vector();
        } else {
            Vector translationalVector = new Vector();
            double x = this.closestPose.getX() - this.poseUpdater.getPose().getX();
            double y = this.closestPose.getY() - this.poseUpdater.getPose().getY();
            translationalVector.setOrthogonalComponents(x, y);
            if (!this.currentPath.isAtParametricEnd() && !this.currentPath.isAtParametricStart()) {
                translationalVector = MathFunctions.subtractVectors(translationalVector, new Vector(MathFunctions.dotProduct(translationalVector, MathFunctions.normalizeVector(this.currentPath.getClosestPointTangentVector())), this.currentPath.getClosestPointTangentVector().getTheta()));
                this.secondaryTranslationalIntegralVector = MathFunctions.subtractVectors(this.secondaryTranslationalIntegralVector, new Vector(MathFunctions.dotProduct(this.secondaryTranslationalIntegralVector, MathFunctions.normalizeVector(this.currentPath.getClosestPointTangentVector())), this.currentPath.getClosestPointTangentVector().getTheta()));
                this.translationalIntegralVector = MathFunctions.subtractVectors(this.translationalIntegralVector, new Vector(MathFunctions.dotProduct(this.translationalIntegralVector, MathFunctions.normalizeVector(this.currentPath.getClosestPointTangentVector())), this.currentPath.getClosestPointTangentVector().getTheta()));
            }

            if (MathFunctions.distance(this.poseUpdater.getPose(), this.closestPose) < FollowerConstants.translationalPIDFSwitch && FollowerConstants.useSecondaryTranslationalPID) {
                this.secondaryTranslationalIntegral.updateError(translationalVector.getMagnitude());
                this.secondaryTranslationalIntegralVector = MathFunctions.addVectors(this.secondaryTranslationalIntegralVector, new Vector(this.secondaryTranslationalIntegral.runPIDF() - this.previousSecondaryTranslationalIntegral, translationalVector.getTheta()));
                this.previousSecondaryTranslationalIntegral = this.secondaryTranslationalIntegral.runPIDF();
                this.secondaryTranslationalPIDF.updateError(translationalVector.getMagnitude());
                translationalVector.setMagnitude(this.secondaryTranslationalPIDF.runPIDF() + FollowerConstants.secondaryTranslationalPIDFFeedForward);
                translationalVector = MathFunctions.addVectors(translationalVector, this.secondaryTranslationalIntegralVector);
            } else {
                this.translationalIntegral.updateError(translationalVector.getMagnitude());
                this.translationalIntegralVector = MathFunctions.addVectors(this.translationalIntegralVector, new Vector(this.translationalIntegral.runPIDF() - this.previousTranslationalIntegral, translationalVector.getTheta()));
                this.previousTranslationalIntegral = this.translationalIntegral.runPIDF();
                this.translationalPIDF.updateError(translationalVector.getMagnitude());
                translationalVector.setMagnitude(this.translationalPIDF.runPIDF() + FollowerConstants.translationalPIDFFeedForward);
                translationalVector = MathFunctions.addVectors(translationalVector, this.translationalIntegralVector);
            }

            translationalVector.setMagnitude(MathFunctions.clamp(translationalVector.getMagnitude(), 0.0, this.driveVectorScaler.getMaxPowerScaling()));
            this.translationalVector = MathFunctions.copyVector(translationalVector);
            return translationalVector;
        }
    }

    public Vector getTranslationalError() {
        Vector error = new Vector();
        double x = this.closestPose.getX() - this.poseUpdater.getPose().getX();
        double y = this.closestPose.getY() - this.poseUpdater.getPose().getY();
        error.setOrthogonalComponents(x, y);
        return error;
    }

    public Vector getCentripetalForceCorrection() {
        if (!useCentripetal) {
            return new Vector();
        } else {
            double curvature;
            if (!this.teleopDrive) {
                curvature = this.currentPath.getClosestPointCurvature();
            } else {
                double yPrime = this.averageVelocity.getYComponent() / this.averageVelocity.getXComponent();
                double yDoublePrime = this.averageAcceleration.getYComponent() / this.averageVelocity.getXComponent();
                curvature = yDoublePrime / Math.pow(Math.sqrt(1.0 + Math.pow(yPrime, 2.0)), 3.0);
            }

            if (Double.isNaN(curvature)) {
                return new Vector();
            } else {
                this.centripetalVector = new Vector(MathFunctions.clamp(FollowerConstants.centripetalScaling * FollowerConstants.mass * Math.pow(MathFunctions.dotProduct(this.poseUpdater.getVelocity(), MathFunctions.normalizeVector(this.currentPath.getClosestPointTangentVector())), 2.0) * curvature, -this.driveVectorScaler.getMaxPowerScaling(), this.driveVectorScaler.getMaxPowerScaling()), this.currentPath.getClosestPointTangentVector().getTheta() + 1.5707963267948966 * MathFunctions.getSign(this.currentPath.getClosestPointNormalVector().getTheta()));
                return this.centripetalVector;
            }
        }
    }

    public Pose getClosestPose() {
        return this.closestPose;
    }

    public boolean atParametricEnd() {
        if (this.followingPathChain) {
            return this.chainIndex == this.currentPathChain.size() - 1 ? this.currentPath.isAtParametricEnd() : false;
        } else {
            return this.currentPath.isAtParametricEnd();
        }
    }

    public double getCurrentTValue() {
        return this.isBusy ? this.currentPath.getClosestPointTValue() : 1.0;
    }

    public double getCurrentPathNumber() {
        return !this.followingPathChain ? 0.0 : (double)this.chainIndex;
    }

    public PathBuilder pathBuilder() {
        return new PathBuilder();
    }

    public void telemetryDebug(MultipleTelemetry telemetry) {
        telemetry.addData("follower busy", this.isBusy());
        telemetry.addData("heading error", this.headingError);
        telemetry.addData("heading vector magnitude", this.headingVector.getMagnitude());
        telemetry.addData("corrective vector magnitude", this.correctiveVector.getMagnitude());
        telemetry.addData("corrective vector heading", this.correctiveVector.getTheta());
        telemetry.addData("translational error magnitude", this.getTranslationalError().getMagnitude());
        telemetry.addData("translational error direction", this.getTranslationalError().getTheta());
        telemetry.addData("translational vector magnitude", this.translationalVector.getMagnitude());
        telemetry.addData("translational vector heading", this.translationalVector.getMagnitude());
        telemetry.addData("centripetal vector magnitude", this.centripetalVector.getMagnitude());
        telemetry.addData("centripetal vector heading", this.centripetalVector.getTheta());
        telemetry.addData("drive error", this.driveError);
        telemetry.addData("drive vector magnitude", this.driveVector.getMagnitude());
        telemetry.addData("drive vector heading", this.driveVector.getTheta());
        telemetry.addData("x", this.getPose().getX());
        telemetry.addData("y", this.getPose().getY());
        telemetry.addData("heading", this.getPose().getHeading());
        telemetry.addData("total heading", this.poseUpdater.getTotalHeading());
        telemetry.addData("velocity magnitude", this.getVelocity().getMagnitude());
        telemetry.addData("velocity heading", this.getVelocity().getTheta());
        this.driveKalmanFilter.debug(telemetry);
        telemetry.update();
//        if (drawOnDashboard) {
//            Drawing.drawDebug(this);
//        }

    }

    public void telemetryDebug(Telemetry telemetry) {
        this.telemetryDebug(new MultipleTelemetry(new Telemetry[]{telemetry}));
    }

    public double getTotalHeading() {
        return this.poseUpdater.getTotalHeading();
    }

    public Path getCurrentPath() {
        return this.currentPath;
    }

    public DashboardPoseTracker getDashboardPoseTracker() {
        return this.dashboardPoseTracker;
    }

    private void resetIMU() throws InterruptedException {
        this.poseUpdater.resetIMU();
    }
}
