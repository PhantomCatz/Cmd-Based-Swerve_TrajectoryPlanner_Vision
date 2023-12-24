package frc.robot;

import java.sql.Driver;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.CatzConstants.ManipulatorPoseConstants;
import frc.robot.Utils.CatzAbstractStateUtil;
import frc.robot.Utils.CatzAbstractStateUtil.GamePieceState;
import frc.robot.commands.ManipulatorToPoseCmd;
import frc.robot.commands.DriveCmds.Trajectory.TrajectoryFollowingCmd;
import frc.robot.commands.DriveCmds.Trajectory.Paths.Trajectories;
import frc.robot.subsystems.Arm.SubsystemCatzArm;
import frc.robot.subsystems.Elevator.SubsystemCatzElevator;
import frc.robot.subsystems.Intake.SubsystemCatzIntake;
import frc.robot.subsystems.drivetrain.SubsystemCatzDrivetrain;

public class CatzAutonomous {
    private SubsystemCatzDrivetrain driveTrain = SubsystemCatzDrivetrain.getInstance(); 
    private SubsystemCatzElevator elevator = SubsystemCatzElevator.getInstance();
    private SubsystemCatzArm arm = SubsystemCatzArm.getInstance();
    private SubsystemCatzIntake intake = SubsystemCatzIntake.getInstance();
    
    //private static PathPlannerPath driveStraighFullTurn = PathPlannerPath.fromPathFile("DriveStraightFullTurn");
    //private static PathPlannerPath feildSideDriveBack = PathPlannerPath.fromPathFile("FeildSideDriveBack");

    public static LoggedDashboardChooser<DriverStation.Alliance> chosenAllianceColor = new LoggedDashboardChooser<>("alliance selector");
    private static LoggedDashboardChooser<AutoModes> autoChooser = new LoggedDashboardChooser<>("Auto Routine");

    private enum AutoModes {
        TEST,
        DRIVE_STRAIGHT,
        TRANSLATE_DRIVE_STRAIGHT,
        PARALEL_SCORE_2
    }

    public CatzAutonomous()
    {
        chosenAllianceColor.addDefaultOption("Blue Alliance", DriverStation.Alliance.Blue);
        chosenAllianceColor.addOption       ("Red Alliance",  DriverStation.Alliance.Red);

        autoChooser.addDefaultOption("Do Nothing", null);

        autoChooser.addOption       ("TEST PATH",  AutoModes.TEST);
        autoChooser.addOption       ("Drivestright", AutoModes.DRIVE_STRAIGHT);
        autoChooser.addOption       ("TranslateDriveStraight", AutoModes.TRANSLATE_DRIVE_STRAIGHT);
    }

    public Command getCommand()
    {
        driveTrain.resetForAutonomous();

        switch(autoChooser.get())
        {
            case TEST: return testPath2();
            case DRIVE_STRAIGHT: return driveStraight();
            case PARALEL_SCORE_2: return parallelScoreCube();
            default: 
            return new InstantCommand();
        }
    }

    public Command testPath()
    {
        driveTrain.resetForAutonomous();
        return new SequentialCommandGroup(
            new TrajectoryFollowingCmd(Trajectories.testTrajectoryCurve, Rotation2d.fromDegrees(180)),
            new TrajectoryFollowingCmd(Trajectories.testTrajectoryCurveGoBack, Rotation2d.fromDegrees(180)),
            new TrajectoryFollowingCmd(Trajectories.testTrajectoryStraight, Rotation2d.fromDegrees(0))
        );
    }

    public Command testPath2()
    {
        driveTrain.resetForAutonomous();
        return new TrajectoryFollowingCmd(Trajectories.testTrajectoryBellsCurve, Rotation2d.fromDegrees(0));
    }

    public Command driveStraight() {
        driveTrain.resetForAutonomous();
        return new TrajectoryFollowingCmd(Trajectories.testTrajectoryStraight, Rotation2d.fromDegrees(180));
    }

    public Command startWall()
    {
        return new SequentialCommandGroup();
    }

    public Command startField()
    {
        return new SequentialCommandGroup();
    }

    public Command startCenter()
    {
        return new SequentialCommandGroup();
    }

    public static SequentialCommandGroup parallelScoreCube()
    {
        return new SequentialCommandGroup(
                    new ManipulatorToPoseCmd(ManipulatorPoseConstants.SCORE_HIGH_CONE)
                        .raceWith(Commands.waitSeconds(1.0)),

                    new ParallelCommandGroup(
                       // new PPTrajectoryFollowingCmd(driveStraighFullTurn), 
                        new SequentialCommandGroup(
                            new ManipulatorToPoseCmd(ManipulatorPoseConstants.STOW)
                                .raceWith(Commands.waitSeconds(1.0)),
                           // new StateMachineCmd(SetMechanismState.PICKUP_GROUND)
                          //      .raceWith(Commands.waitSeconds(1.0)),
                            intakeSequenceCommandGroup())
                                            ),

                    new ParallelCommandGroup(
                       // new PPTrajectoryFollowingCmd(feildSideDriveBack), 
                        new SequentialCommandGroup(
                            new ManipulatorToPoseCmd(ManipulatorPoseConstants.STOW)
                                .raceWith(Commands.waitSeconds(1.0)),
                            new ManipulatorToPoseCmd(ManipulatorPoseConstants.SCORE_HIGH_CONE)
                                .raceWith(Commands.waitSeconds(1.0)))
                                            ),
                    intakeSequenceCommandGroup(),
                    new ManipulatorToPoseCmd(ManipulatorPoseConstants.STOW)
                                         );       
    }

    public static SequentialCommandGroup RightScore1HighConeBalance()
    {
        return new SequentialCommandGroup(
                    Commands.runOnce(() -> CatzAbstractStateUtil.newGamePieceState(GamePieceState.CONE)),
                    new ManipulatorToPoseCmd(CatzConstants.ManipulatorPoseConstants.SCORE_HIGH_CONE)

                   //new PPTrajectoryFollowingCmd(driveStraighFullTurn)
            
        );
    }

    public static SequentialCommandGroup intakeSequenceCommandGroup()
    {
        return null;
        // new SequentialCommandGroup(
        //         intake.intakeRollersIn(),
        //         Commands.waitSeconds(0.5),
        //         intake.intakeRollersOff()           
        //                                  );
    }
}