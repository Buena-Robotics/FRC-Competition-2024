package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Motors;
import frc.robot.utils.ControllerUtils;

import static frc.robot.Constants.Swerve.L;
import static frc.robot.Constants.Swerve.W;


public class SwerveDriveSubsystem extends SubsystemBase {
    
    private SwerveModule backRight = new SwerveModule (Motors.BACKRIGHTSTEER, Motors.BACKRIGHTDRIVE, Motors.BACKRIGHTENCODER);
    private SwerveModule backLeft = new SwerveModule (Motors.BACKLEFTSTEER, Motors.BACKLEFTDRIVE, Motors.BACKLEFTENCODER);
    private SwerveModule frontRight = new SwerveModule (Motors.FRONTRIGHTSTEER, Motors.FRONTRIGHTDRIVE, Motors.FRONTRIGHTENCODER);
    private SwerveModule frontLeft = new SwerveModule (Motors.FRONTLEFTSTEER, Motors.FRONTLEFTDRIVE, Motors.FRONTLEFTENCODER);

    public void drive (double x1, double y1, double x2) {
        if(x1 < 0.05) x1 = 0;
        if(y1 < 0.05) y1 = 0;
        if(x2 < 0.05) x2 = 0;


        double r = Math.sqrt ((L * L) + (W * W));
        y1 *= -1;

        double a = x1 - x2 * (L / r);
        double b = x1 + x2 * (L / r);
        double c = y1 - x2 * (W / r);
        double d = y1 + x2 * (W / r);

        double backRightSpeed = Math.sqrt ((a * a) + (d * d));
        double backLeftSpeed = Math.sqrt ((a * a) + (c * c));
        double frontRightSpeed = Math.sqrt ((b * b) + (d * d));
        double frontLeftSpeed = Math.sqrt ((b * b) + (c * c));

        double backRightAngle = Math.atan2 (a, d) / Math.PI;
        double backLeftAngle = Math.atan2 (a, c) / Math.PI;
        double frontRightAngle = Math.atan2 (b, d) / Math.PI;
        double frontLeftAngle = Math.atan2 (b, c) / Math.PI;

        // SmartDashboard.putNumber("BackLeft Angle", backLeftAngle);
        // SmartDashboard.putNumber("BackRight Angle", backRightAngle);
        // SmartDashboard.putNumber("FrontLeft Angle", frontLeftAngle);
        // SmartDashboard.putNumber("FrontRight Angle", frontRightAngle);
        // SmartDashboard.putNumber("BackLeft Speed", backLeftSpeed);
        // SmartDashboard.putNumber("BackRight Speed", backRightSpeed);
        // SmartDashboard.putNumber("FrontLeft Speed", frontLeftSpeed);
        // SmartDashboard.putNumber("FrontRight Speed", frontRightSpeed);

        backRight.drive (backRightSpeed, backRightAngle);
        backLeft.drive (backLeftSpeed, backLeftAngle);
        frontRight.drive (frontRightSpeed, frontRightAngle);
        frontLeft.drive (frontLeftSpeed, frontLeftAngle);
    }

    public void arcadeDrive() {
        drive(ControllerUtils.controller.getLeftX(), ControllerUtils.controller.getLeftY(), ControllerUtils.controller.getRightX());
    }
}
