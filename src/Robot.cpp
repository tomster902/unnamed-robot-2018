/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <Commands/Command.h>
#include <Commands/Scheduler.h>
#include <LiveWindow/LiveWindow.h>
#include <SmartDashboard/SendableChooser.h>
#include <SmartDashboard/SmartDashboard.h>
#include <TimedRobot.h>
#include "WPILib.h"

#include "Commands/ExampleCommand.h"
#include "Commands/MyAutoCommand.h"

class Robot : public frc::IterativeRobot {
public:
	class PIDRotate : public PIDOutput {
		public:
			PIDRotate(MecanumDrive& drive, ADXRS450_Gyro& gyro) {
				m_drive = &drive;
				m_gyro = &gyro;
			}

			void SetX(double x) {
				m_x = x;
			}

			void SetY(double y) {
				m_y = y;
			}

			void SetRotate(double rotate) {
				m_rotate = rotate;
			}

			void DriveCartesian(double x, double y, double rotate) {
				m_x = x;
				m_y = y;
				m_rotate = rotate;
			}

			void PIDWrite(double output) override {
				double curve = (-m_gyro->GetAngle())/RATE;
				double rotate = curve*output;
				DriverStation::ReportWarning("Whitman Curve: " + std::to_string(curve) + "\nRot:" + std::to_string(rotate) +
						"\nAngle:" + std::to_string(m_gyro->GetAngle()) + "\nOutput:" + std::to_string(output));
				m_drive->DriveCartesian(m_x, m_y, rotate + m_rotate);
			}
		private:
			MecanumDrive *m_drive;
			ADXRS450_Gyro *m_gyro;
			double m_x = 0.0;
			double m_y = 0.0;
			double m_rotate = 0.0;
			const double RATE = 100;
		};

	void RobotInit() override {
		m_chooser.AddDefault("Default Auto", &m_defaultAuto);
		m_chooser.AddObject("My Auto", &m_myAuto);
		Talon *rightFrontWheel = new Talon(2);
		Talon *leftFrontWheel = new Talon(1);
		Talon *rightBackWheel = new Talon(4);
		Talon *leftBackWheel = new Talon(3);
		m_xbox = new Joystick(0); // Define joystick being used at USB port #0 on the Drivers Station
		m_mecanumDrive = new MecanumDrive(*leftFrontWheel, *leftBackWheel, *rightFrontWheel, *rightBackWheel);
		gyro = new ADXRS450_Gyro();
		rotate_controller = new PIDRotate(*m_mecanumDrive, *gyro);
		gyro_controller = new PIDController(p, i, d, 3, gyro, rotate_controller);
		SmartDashboard::PutNumber("P", p);
		SmartDashboard::PutNumber("I", i);
		SmartDashboard::PutNumber("D", d);
		SmartDashboard::PutNumber("F", f);
		gyro_controller->Enable();
		//gyro_controller->SetContinuous();
		gyro_controller->SetSetpoint(0.0);
		gyro_controller->SetAbsoluteTolerance(30);
		gyro_controller->SetInputRange(-180.0, 180.0);
		gyro_controller->SetOutputRange(-K, K);
		frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
	}

	/**
	 * This function is called once each time the robot enters Disabled
	 * mode.
	 * You can use it to reset any subsystem information you want to clear
	 * when
	 * the robot is disabled.
	 */
	void DisabledInit() override {}

	void DisabledPeriodic() override {
		frc::Scheduler::GetInstance()->Run();
	}

	/**
	 * This autonomous (along with the chooser code above) shows how to
	 * select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * GetString code to get the auto name from the text box below the Gyro.
	 *
	 * You can add additional auto modes by adding additional commands to
	 * the
	 * chooser code above (like the commented example) or additional
	 * comparisons
	 * to the if-else structure below with additional strings & commands.
	 */
	void AutonomousInit() override {
		std::string autoSelected = frc::SmartDashboard::GetString(
				"Auto Selector", "Default");
		if (autoSelected == "My Auto") {
			m_autonomousCommand = &m_myAuto;
		} else {
			m_autonomousCommand = &m_defaultAuto;
		}

		m_autonomousCommand = m_chooser.GetSelected();

		if (m_autonomousCommand != nullptr) {
			m_autonomousCommand->Start();
		}
	}

	void AutonomousPeriodic() override {
		frc::Scheduler::GetInstance()->Run();
	}

	void TeleopInit() override {
		// This makes sure that the autonomous stops running when
		// teleop starts running. If you want the autonomous to
		// continue until interrupted by another command, remove
		// this line or comment it out.
		if (m_autonomousCommand != nullptr) {
			m_autonomousCommand->Cancel();
			m_autonomousCommand = nullptr;
		}
		gyro->Calibrate();
		Wait(5);
	}

	void TeleopPeriodic() override {
		frc::Scheduler::GetInstance()->Run();
		double strafe = m_xbox->GetRawAxis(4);
		double forward = -m_xbox->GetRawAxis(5);
		double rotate = m_xbox->GetRawAxis(0);
		//DriverStation::ReportWarning("on target?:" + std::to_string(gyro_controller->OnTarget()));
		/*if (rotate == 0.0) {
			double angle =
			if (angle != 0.0) {

			}
		}*/
		rotate*=K;
		forward*=K;
		strafe*=K;
		gyro_controller->SetP(SmartDashboard::GetNumber("P", p));
		gyro_controller->SetI(SmartDashboard::GetNumber("I", i));
		gyro_controller->SetD(SmartDashboard::GetNumber("D", d));
		gyro_controller->SetF(SmartDashboard::GetNumber("F", f));
		rotate_controller->DriveCartesian(strafe, forward, rotate);
	}

	void TestPeriodic() override {}

private:
	// Have it null by default so that if testing teleop it
	// doesn't have undefined behavior and potentially crash.
	MecanumDrive *m_mecanumDrive;		// RobotDrive object using PWM 1-4 for drive motors
	Joystick *m_xbox;
	ADXRS450_Gyro *gyro;
	PIDController *gyro_controller;
	PIDRotate *rotate_controller;
	frc::Command* m_autonomousCommand = nullptr;
	ExampleCommand m_defaultAuto;
	MyAutoCommand m_myAuto;
	frc::SendableChooser<frc::Command*> m_chooser;
	const double K = 0.4;
	const double p = 1.0;
	const double i = 0.1;
	const double d = 0.1;
	const double f = 0.1;
};

START_ROBOT_CLASS(Robot)
