package frc.robot.subsystems;

import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.VoltsPerMeterPerSecond;

import frc.robot.SwerveModule;
import frc.lib.math.Conversions;
import frc.lib.util.Limelight;
import frc.lib.util.LimelightHelpers;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class Swerve extends SubsystemBase {
    public SwerveDriveOdometry swerveOdometry;
    public SwerveDrivePoseEstimator poseEstimator; 
    public SwerveModule[] mSwerveMods;
    public Pigeon2 gyro;
    public String limelight_string; 

    private ChassisSpeeds chassisSpeeds;

    private double speedMultiplier = 1;

    private TalonFX m_frontLeftMotor;
    private TalonFX m_frontRightMotor;
    private TalonFX m_backLeftMotor;
    private TalonFX m_backRightMotor;

    // SysID
    private final MutableMeasure<Voltage> m_appliedVoltage = mutable(Volts.of(0));
    private final MutableMeasure<Distance> m_distance = mutable(Meters.of(0));
    private final MutableMeasure<Velocity<Distance>> m_velocity = mutable(MetersPerSecond.of(0));
    private final Measure<Velocity<Voltage>> m_desiredRampRate = Velocity.combine(Volts, Second).of(1);
    private final Measure<Voltage> m_desiredStepVoltage = Volts.of(5);

    private Field2d m_field = new Field2d();

    // private final SysIdRoutine m_sysIdRoutine =
    //   new SysIdRoutine(
    //       // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
    //       new SysIdRoutine.Config(m_desiredRampRate, m_desiredStepVoltage, null),
    //       new SysIdRoutine.Mechanism(
    //           // Tell SysId how to plumb the driving voltage to the motors.
    //           (Measure<Voltage> volts) -> {
    //             m_frontLeftMotor.setVoltage(volts.in(Volts));
    //             m_frontRightMotor.setVoltage(volts.in(Volts));
    //             m_backLeftMotor.setVoltage(volts.in(Volts));
    //             m_backRightMotor.setVoltage(volts.in(Volts));
    //           },
    //           // Tell SysId how to record a frame of data for each motor on the mechanism being
    //           // characterized.
    //           log -> {
    //             // Record a frame for the left motors.  Since these share an encoder, we consider
    //             // the entire group to be one motor.
    //             log.motor("drive-front-left")
    //                 .voltage(
    //                     m_appliedVoltage.mut_replace(
    //                         m_frontLeftMotor.get() * RobotController.getBatteryVoltage(), Volts))
    //                 .linearPosition(m_distance.mut_replace(Conversions.rotationsToMeters(m_frontLeftMotor.getPosition().getValue(), Constants.Swerve.wheelCircumference), Meters))
    //                 .linearVelocity(
    //                     m_velocity.mut_replace(Conversions.rotationsToMeters(m_frontLeftMotor.getVelocity().getValue(), Constants.Swerve.wheelCircumference), MetersPerSecond));
    //             // Record a frame for the right motors.  Since these share an encoder, we consider
    //             // the entire group to be one motor.
    //             log.motor("drive-front-right")
    //                 .voltage(
    //                     m_appliedVoltage.mut_replace(
    //                         m_frontRightMotor.get() * RobotController.getBatteryVoltage(), Volts))
    //                 .linearPosition(m_distance.mut_replace(Conversions.rotationsToMeters(m_frontRightMotor.getPosition().getValue(), Constants.Swerve.wheelCircumference), Meters))
    //                 .linearVelocity(
    //                     m_velocity.mut_replace(Conversions.rotationsToMeters(m_frontRightMotor.getVelocity().getValue(), Constants.Swerve.wheelCircumference), MetersPerSecond));
    //             log.motor("drive-back-left")
    //                 .voltage(
    //                     m_appliedVoltage.mut_replace(
    //                         m_backLeftMotor.get() * RobotController.getBatteryVoltage(), Volts))
    //                 .linearPosition(m_distance.mut_replace(Conversions.rotationsToMeters(m_backLeftMotor.getPosition().getValue(), Constants.Swerve.wheelCircumference), Meters))
    //                 .linearVelocity(
    //                     m_velocity.mut_replace(Conversions.rotationsToMeters(m_backLeftMotor.getVelocity().getValue(), Constants.Swerve.wheelCircumference), MetersPerSecond));
    //             log.motor("drive-back-right")
    //                 .voltage(
    //                     m_appliedVoltage.mut_replace(
    //                         m_backRightMotor.get() * RobotController.getBatteryVoltage(), Volts))
    //                 .linearPosition(m_distance.mut_replace(Conversions.rotationsToMeters(m_backRightMotor.getPosition().getValue(), Constants.Swerve.wheelCircumference), Meters))
    //                 .linearVelocity(
    //                     m_velocity.mut_replace(Conversions.rotationsToMeters(m_backRightMotor.getVelocity().getValue(), Constants.Swerve.wheelCircumference), MetersPerSecond));
    //           },
    //           // Tell SysId to make generated commands require this subsystem, suffix test state in
    //           // WPILog with this subsystem's name ("drive")
    //           this));


    public Swerve() {
        gyro = new Pigeon2(Constants.Swerve.pigeonID);
        gyro.getConfigurator().apply(new Pigeon2Configuration());
        gyro.setYaw(0);

        mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, Constants.Swerve.Mod0.constants),
            new SwerveModule(1, Constants.Swerve.Mod1.constants),
            new SwerveModule(2, Constants.Swerve.Mod2.constants),
            new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };

        swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getGyroYaw(), getModulePositions());

        m_frontLeftMotor = mSwerveMods[0].getDriveMotor();
        m_frontRightMotor = mSwerveMods[1].getDriveMotor();
        m_backLeftMotor = mSwerveMods[2].getDriveMotor();
        m_backRightMotor = mSwerveMods[3].getDriveMotor();

        AutoBuilder.configureHolonomic(
                this::getPose, // Robot pose supplier
                this::setPose, // Method to reset odometry (will be called if your auto has a starting pose)
                this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                        new PIDConstants(10, 0, 0), // Translation PID constants
                        new PIDConstants(0.095, 0.0009, 0.01), // (p 0.85), (p 0.6 d 0.3),  
                        3, // Max module speed, in m/s
                        Constants.Swerve.trackWidth, // Drive base radius in meters. Distance from robot center to furthest module.
                        new ReplanningConfig() // Default path replanning config. See the API for the options here
                ),
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this // Reference to this subsystem to set requirements
        );
        
        Pose2d ampPose = new Pose2d(new Translation2d(1.649644, 5.682691), new Rotation2d());
        Pose2d initPose = new Pose2d(new Translation2d(4.081077, 1.303912), new Rotation2d()); 
        poseEstimator = new SwerveDrivePoseEstimator(Constants.Swerve.swerveKinematics, getGyroYaw(), getModulePositions(), new Pose2d());
        poseEstimator.resetPosition(new Rotation2d(), getModulePositions(), initPose);
        //poseEstimator.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d());

        

    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates =
            Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                    translation.getX() * speedMultiplier, 
                                    translation.getY() * speedMultiplier, 
                                    rotation * speedMultiplier, 
                                    getHeading()
                                )
                                : new ChassisSpeeds(
                                    translation.getX() * speedMultiplier, 
                                    translation.getY() * speedMultiplier, 
                                    rotation * speedMultiplier)
                                );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }

        chassisSpeeds = new ChassisSpeeds(
            translation.getX() * speedMultiplier,
            translation.getY() * speedMultiplier,
            rotation * speedMultiplier
        );
    }    

    public void driveRobotRelative(ChassisSpeeds chassisSpeeds) {
        SwerveModuleState[] swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(chassisSpeeds);

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], false);
        }
    }

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);
        
        for(SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }

    public SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : mSwerveMods){
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : mSwerveMods){
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public void toggleMultiplier() {
        speedMultiplier = speedMultiplier == 1 ? 0.2 : 1;
    }

    public boolean isLowGear() {
        return speedMultiplier == 0.2;
    }

    public Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
    }

    public Pose2d getEstimatedPose() {
        return poseEstimator.getEstimatedPosition(); 
    }

    public void setPose(Pose2d pose) {
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), pose);
    }

    public Rotation2d getHeading(){
        return getPose().getRotation();
    }

    public void setHeading(Rotation2d heading){
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), heading));
    }

    public void zeroHeading(){
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), new Rotation2d()));
    }

    public Rotation2d getGyroYaw() {
        // if(Math.abs(gyro.getYaw().getValue()) < 1) return Rotation2d.fromDegrees(0);i
        return Rotation2d.fromDegrees(Math.round(gyro.getYaw().getValue()));
    }

    public void resetModulesToAbsolute(){
        for(SwerveModule mod : mSwerveMods){
            mod.resetToAbsolute();
        }
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return chassisSpeeds;
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return null;
        // return m_sysIdRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return null;
        // return m_sysIdRoutine.dynamic(direction);
    }


    @Override
    public void periodic(){
        // m_backLeftMotor.setControl(new Follower(Constants.Swerve.Mod0.driveMotorID, false));
        // m_backRightMotor.setControl(new Follower(Constants.Swerve.Mod1.driveMotorID, false));

        swerveOdometry.update(getGyroYaw(), getModulePositions());
        m_field.setRobotPose(poseEstimator.getEstimatedPosition());
        poseEstimator.update(getGyroYaw(), getModulePositions()); //swerve drive pose estimator 
        //SmartDashboard.putData("Robot's estimated pose", poseEstimator.getEstimatedPos);
        
        //fuse apriltag with swerve odometry
        
        limelight_string = "limelight-top";
        LimelightHelpers.getLatestResults(limelight_string);
        if (LimelightHelpers.getTV(limelight_string)) {
            if (LimelightHelpers.getTA(limelight_string) > 0.2) {
                poseEstimator.addVisionMeasurement(LimelightHelpers.getBotPose2d_wpiBlue(limelight_string), LimelightHelpers.getLatency_Capture(limelight_string));
            }

            else {
                //do nothing -> pass
            }
        
        }

        double y_difference = getEstimatedPose().getY() - Constants.MapPoses.subwoofer_location_blue.getY();
        double x_difference = getEstimatedPose().getX() - Constants.MapPoses.subwoofer_location_blue.getX(); 
        double angle = Math.atan(y_difference / x_difference) * Math.PI * 2;
        double turnTo180 = 0;

        //if statements
        if (getEstimatedPose().getRotation().getDegrees() < 0) {
            turnTo180 = -180 - getEstimatedPose().getRotation().getDegrees(); 
        } 
        
        else {
            turnTo180 = 180 - getEstimatedPose().getRotation().getDegrees(); 
        }





        double distance = getEstimatedPose().getTranslation().getDistance(Constants.MapPoses.subwoofer_location_blue.getTranslation());
        Rotation2d turnAngle = getEstimatedPose().getRotation().minus(Constants.MapPoses.subwoofer_location_blue.getRotation());
        SmartDashboard.putNumber("Angle turn to score", turnAngle.getDegrees());
        SmartDashboard.putNumber("Limelight Pose X", LimelightHelpers.getBotPose2d_wpiBlue(limelight_string).getTranslation().getX());
        SmartDashboard.putNumber("Limelight Pose Y", LimelightHelpers.getBotPose2d_wpiBlue(limelight_string).getTranslation().getY());
        SmartDashboard.putNumber("Limelight Pose Heading", LimelightHelpers.getBotPose2d_wpiBlue(limelight_string).getRotation().getDegrees());
        SmartDashboard.putNumber("Calculated distance from transform", distance); 
        SmartDashboard.putNumber("Angle turn to 180", turnTo180);

        //get transform from robot to subwoofer on blue alliance based on just module positions and gyro heading 
        //Transform2d transform = poseEstimator.getEstimatedPosition().trans

        for(SwerveModule mod : mSwerveMods){
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " CANcoder", mod.getCANcoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Angle", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);    
        }

        SmartDashboard.putNumber("Pigeon", gyro.getYaw().getValue());
        SmartDashboard.putData(m_field);
    }

    public void stop() {
        RobotContainer.s_Swerve.drive(
            new Translation2d(0, 0), 
            0, true, true
            
        );
    }
}