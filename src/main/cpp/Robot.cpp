// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/Joystick.h>
#include <frc/TimedRobot.h>
#include <frc/SpeedControllerGroup.h>
#include <frc/drive/DifferentialDrive.h>
#include "ctre/Phoenix.h"
#include <frc/DoubleSolenoid.h>
#include <frc/Joystick.h>
#include <frc/PneumaticsControlModule.h>
#include <frc/Solenoid.h>
#include <frc/AddressableLED.h>
#include <cameraserver/CameraServer.h>


/**
 * This is a demo program showing the use of the DifferentialDrive class.
 * Runs the motors with tank steering.
 */

class Robot : public frc::TimedRobot {
  
  //DRIVE MOTORS
	WPI_TalonSRX m_FLMotor{21};
	WPI_TalonSRX m_FRMotor{24};
	WPI_TalonSRX m_BLMotor{23};
	WPI_TalonSRX m_BRMotor{22};
  //HANGER/MECHANISIM MOTORS
  WPI_TalonSRX m_HRMotor{25};
  WPI_TalonSRX m_SHMotor{26};
  WPI_TalonSRX m_EMMotor{27};
 // WPI_TalonSRX m_INMotor{28};
	int autoLoopCounter;
   frc::SpeedControllerGroup m_right{m_FRMotor, m_BRMotor};
	 frc::SpeedControllerGroup m_left{m_FLMotor, m_BLMotor};
   

  frc::DifferentialDrive m_robotDrive{m_left, m_right};
  frc::Joystick m_Stick1{0};
  frc::Joystick m_Stick2{1};
  frc::Joystick m_stick{2};

  // Solenoid corresponds to a single solenoid.
  frc::Solenoid m_solenoid{frc::PneumaticsModuleType::CTREPCM, 0};

  // DoubleSolenoid corresponds to a double solenoid.
  frc::DoubleSolenoid m_doubleSolenoid{frc::PneumaticsModuleType::CTREPCM, 1,
                                       2};
 public:
 
  void RobotInit() override {
    
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_right.SetInverted(true);
   // m_FRMotor.SetInverted(true);
   // m_FLMotor.SetInverted(true);
    m_EMMotor.SetInverted(true);
    m_HRMotor.SetInverted(true);
    m_HRMotor.SetInverted(true);
    m_HRMotor.SetNeutralMode(motorcontrol::Brake);
    m_EMMotor.SetNeutralMode(motorcontrol::Brake);

    frc::CameraServer::StartAutomaticCapture();
        //frc::CameraServer::StartAutomaticCapture(1);


  }

  void TeleopInit() override {
    m_solenoid.Set(false);
    m_doubleSolenoid.Set(frc::DoubleSolenoid::kReverse);
  }

  void TeleopPeriodic() override {
    // Drive with tank style
		m_robotDrive.ArcadeDrive(m_Stick1.GetY()*-1, m_Stick2.GetX());


     m_EMMotor.Set(m_stick.GetRawButton(2));
     m_SHMotor.Set(m_stick.GetRawButton(1));


    if (m_stick.GetRawButton(3)) m_HRMotor.Set(1);
    else if (m_stick.GetRawButton(4)) m_HRMotor.Set(-1);
    else m_HRMotor.Set(0);
   
   
    

    /* In order to set the double solenoid, we will say that if neither button
     * is pressed, it is off, if just one button is pressed, set the solenoid to
     * correspond to that button, and if both are pressed, set the solenoid to
     * Forwards.
     */
    if (m_stick.GetRawButton(kDoubleSolenoidForward)) {
      m_doubleSolenoid.Set(frc::DoubleSolenoid::kForward);
    } else if (m_stick.GetRawButton(kDoubleSolenoidReverse)) {
      m_doubleSolenoid.Set(frc::DoubleSolenoid::kReverse);
    } else {
      m_doubleSolenoid.Set(frc::DoubleSolenoid::kOff);
    }
  }

  void AutonomousInit() override {
    autoLoopCounter = 0;
  }

  void AutonomousPeriodic() override {
    if (autoLoopCounter < 50) {
      m_EMMotor.Set(0.3);
    } else if (autoLoopCounter < 100) {
      m_EMMotor.Set(0);
      m_SHMotor.Set(1);
    } else if (autoLoopCounter < 175) {
      m_SHMotor.Set(0);
      m_robotDrive.ArcadeDrive(-0.3, 0, false);
    } else {
      m_robotDrive.ArcadeDrive(0, 0);
    }
    autoLoopCounter++;
  }

 private:
  

  static constexpr int kSolenoidButton = 1;
  static constexpr int kDoubleSolenoidForward = 5;
  static constexpr int kDoubleSolenoidReverse = 6;
};

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif

