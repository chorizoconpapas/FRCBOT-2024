package frc.robot;

import com.fasterxml.jackson.databind.AnnotationIntrospector.ReferenceProperty.Type;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.ModuleConstants.DriveConstants;


public class swerveModule {
    private final CANSparkMax driveMotor;
    private final CANSparkMax turningMotor;

    private final RelativeEncoder driverEncoder;

    private final AbsoluteEncoder absoluteEncoder;
    private final boolean  absoluteEncoderReversed;
    private final double absoluteEncoderOffsetRad;

    private final PIDController turningPidController;

    public swerveModule(int driveMotorId,int turningMotorId,boolean driveMotorReversed,boolean turningMotorReversed,int absoluteEncoderId
    ,double absoluteEncoderOffset,boolean absoluteEncoderReversed){

        this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
        this.absoluteEncoderReversed = absoluteEncoderReversed;

        driveMotor = new CANSparkMax (driveMotorId,MotorType.kBrushless);
        turningMotor = new CANSparkMax(turningMotorId, MotorType.kBrushless);

        absoluteEncoder = turningMotor.getAbsoluteEncoder(com.revrobotics.SparkAbsoluteEncoder.Type.kDutyCycle);

        driveMotor.setInverted(driveMotorReversed);
        turningMotor.setInverted(turningMotorReversed);

        driverEncoder = driveMotor.getEncoder();

        driverEncoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter);
        driverEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderRot2Meter/60);
        absoluteEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderRot2Rad);
        absoluteEncoder.setInverted(absoluteEncoderReversed);
        absoluteEncoder.setZeroOffset(absoluteEncoderOffset);

        turningPidController =new PIDController(ModuleConstants.kPTurning, 0,0);
        turningPidController.enableContinuousInput(-Math.PI,Math.PI);
        
        resetEncoders();
    }
 
    public double getDrivePosition(){
        return driverEncoder.getPosition();
    }

    public double getTurningPosition(){
        return absoluteEncoder.getPosition();
    }

    public double getDriveVelocity(){
        return driverEncoder.getVelocity();
    }

    public double getTurningVelocity(){
        return absoluteEncoder.getVelocity();
    }

    public double getAbsoluteEncoderRad(){
        return absoluteEncoder.getVelocity();
    }

    public void resetEncoders(){
        driverEncoder.setPosition(0);
    }

    public SwerveModuleState getState (){
        return new SwerveModuleState(getDriveVelocity(),new Rotation2d(getTurningPosition()));
    }

    public void  setDesiredState(SwerveModuleState state){

        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop ();
            return;
        }

        state = SwerveModuleState.optimize(state,getState().angle);
        driveMotor.set(state.speedMetersPerSecond/DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        turningMotor.set(turningPidController.calculate(absoluteEncoder.getPosition(), state.angle.getRadians()));
        //SmartDashboard.putString("Swerve["+ absoluteEncoder.getChannel() +"] state",state.toString());
     }

    public void stop() {
        driveMotor.set(0);
        turningMotor.set(0);
    }
    
}