package frc.robot;

import java.util.List;

import com.ctre.phoenix.CANifier.LEDChannel;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.LED.LEDColor;
import frc.robot.subsystems.LED.LEDMode;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    private final Joystick driver = new Joystick(0);

    /* Drive Controls */
    private final int translationAxis = 1;
    private final int strafeAxis = 0;
    private final int rotationAxis = 2;

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, 3); // B
    private final JoystickButton robotCentric = new JoystickButton(driver, 5); // LB
    private final JoystickButton slowMode = new JoystickButton(driver, 8); // RT
    private final JoystickButton trackApriltag = new JoystickButton(driver, 1); // x
    private final JoystickButton alignApriltag = new JoystickButton(driver, 7); // LT
    private final POVButton leftPOV = new POVButton(driver, 270);
    private final POVButton rightPOV = new POVButton(driver, 90);
    private final POVButton upPOV = new POVButton(driver, 0);
    private final POVButton downPOV = new POVButton(driver, 180);

    /* Subsystems */
    public final static Swerve s_Swerve = new Swerve();
    public final static Vision vision = new Vision(s_Swerve);
    public static final LED s_Led = new LED(0, 60);

    /* The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> driver.getRawAxis(translationAxis), 
                () -> driver.getRawAxis(strafeAxis), 
                () -> driver.getRawAxis(rotationAxis), 
                () -> robotCentric.getAsBoolean()
            )
        );

        // Register Named Commands
        NamedCommands.registerCommand("shoot", new LedCommand(LEDColor.GREEN, LEDMode.STATIC));
        NamedCommands.registerCommand("intake", new LedCommand(LEDColor.YELLOW, LEDMode.STATIC));

        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));
        slowMode.onTrue(new InstantCommand(() -> s_Swerve.toggleMultiplier()));
        alignApriltag.whileTrue(new AlignApriltagCommand());
        trackApriltag.whileTrue(new ApriltagTracker(s_Swerve));

        // SysID
        // leftPOV.whileTrue(s_Swerve.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        // rightPOV.whileTrue(s_Swerve.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        // downPOV.whileTrue(s_Swerve.sysIdDynamic(SysIdRoutine.Direction.kForward));
        // upPOV.whileTrue(s_Swerve.sysIdDynamic(SysIdRoutine.Direction.kReverse));

        // leftPOV.whileTrue(new InstantCommand(() -> System.out.println("left")));
        // rightPOV.whileTrue(new InstantCommand(() -> System.out.println("right")));
        // downPOV.whileTrue(new InstantCommand(() -> System.out.println("down")));
        // upPOV.whileTrue(new InstantCommand(() -> System.out.println("up")));
    }

    /* 
    private Command alignCommand() {
        Command com = new SequentialCommandGroup(
            new ApriltagTracker(s_Swerve),
            new WaitCommand(1),
            new LedCommand(LEDColor.GREEN, LEDMode.STATIC)
        );
        return com;
    }
    */


    public Command getAutonomousCommand() {
        // return new PathPlannerAuto("Rotate Auto");

        return new PathPlannerAuto("FreddyComboRotationAuton");
        // return new PathPlannerAuto("VerticalAuton");
        // return new PathPlannerAuto("First path test");
        // return Commands.runOnce(() -> {
        //     Pose2d currentPose = s_Swerve.getPose();
            
        //     // The rotation component in these poses represents the direction of travel
        //     Pose2d startPos = new Pose2d(currentPose.getTranslation(), new Rotation2d());
        //     Pose2d endPos = new Pose2d(currentPose.getTranslation().plus(new Translation2d(-2, 0)), Rotation2d.fromDegrees(180));

        //     List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(startPos, endPos);
        //     PathPlannerPath path = new PathPlannerPath(
        //         bezierPoints, 
        //         new PathConstraints(
        //         1, 1, 
        //         Units.degreesToRadians(180), Units.degreesToRadians(90)
        //         ),  
        //         new GoalEndState(0.0, Rotation2d.fromDegrees(180))
        //     );

        //     // Prevent this path from being flipped on the red alliance, since the given positions are already correct
        //     path.preventFlipping = true;

        //     AutoBuilder.followPath(path).schedule();
        // });
    }
}