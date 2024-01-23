package frc.robot;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.constraint.SwerveDriveKinematicsConstraint;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ModuleConstants.DriveConstants;
import frc.robot.ModuleConstants.DriveConstants;
public class swerveSubsystemBase extends SubsystemBase {
    private final swerveModule frontleft = new swerveModule(
        DriveConstants.kFrontLeftDriveMotorPort,
        DriveConstants.kFrontLeftTurningMotorPort,
        DriveConstants.kFrontLeftDriveEncoderReversed,
        DriveConstants.kFrontLeftTurningEncoderReversed,
        DriveConstants.kFrontLeftDriveAbsoluteEncoderPort,
        DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetRad,
        DriveConstants.kFrontLeftDriveEncoderReversed);

   private final swerveModule frontRight = new swerveModule(
        DriveConstants.kFrontRightDriveMotorPort,
        DriveConstants.kFrontRightTurningMotorPort,
        DriveConstants.kFrontRightDriveEncoderReversed,
        DriveConstants.kFrontRightTurningEncoderReversed,
        DriveConstants.kFrontRightDriveAbsoluteEncoderPort,
        DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetRad,
        DriveConstants.kFrontRightDriveEncoderReversed);
    
      private final swerveModule backleft = new swerveModule(
        DriveConstants.kBackLeftDriveMotorPort,
        DriveConstants.kBackLeftTurningMotorPort,
        DriveConstants.kBackLeftDriveEncoderReversed,
        DriveConstants.kBackLeftTurningEncoderReversed,
        DriveConstants.kBackLeftDriveAbsoluteEncoderPort,
        DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetRad,
        DriveConstants.kBackLeftDriveEncoderReversed);

      private final swerveModule backRight = new swerveModule(
        DriveConstants.kBackRightDriveMotorPort,
        DriveConstants.kBackRightTurningMotorPort,
        DriveConstants.kBackRightDriveEncoderReversed,
        DriveConstants.kBackRightTurningEncoderReversed,
        DriveConstants.kBackRightDriveAbsoluteEncoderPort,
        DriveConstants.kBackRightDriveAbsoluteEncoderOffsetRad,
        DriveConstants.kBackRightDriveEncoderReversed);

        Pigeon2 gyro = new Pigeon2(0);
        
   public swerveSubsystemBase(){
         
         new Thread(() ->{
          try {
            Thread.sleep(1000);
            zeroHeading();
          } catch (Exception e) { }
        }).start();
        }
        
        public void zeroHeading(){
          gyro.reset();
        }
      public double getHeading(){
        return Math.IEEEremainder(gyro.getAngle(), 360);
      }

    public Rotation2d getRotation2d(){
      return Rotation2d.fromDegrees(getHeading());
    }
    @Override
    public void periodic() {
      SmartDashboard.putNumber("robot Heading", getHeading());
    }
    public void stopModels(){
      frontRight.stop();
      frontleft.stop();
      backRight.stop();
      backleft.stop();
    }

    public void setModulesState(SwerveModuleState[] desiredStates){
      SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
      frontRight.setDesiredState(desiredStates[0]);
      frontleft.setDesiredState(desiredStates[1]);
      backRight.setDesiredState(desiredStates[2]);
      backleft.setDesiredState(desiredStates[3]);
    }
    
}

