#include <chrono>
#include <iostream>
#include <thread>

int main() {
  int port_number_left;
  std::cout << "\nHow many motors are on your LEFT drivetrain? ";
  std::cin >> port_number_left;

  int port_number_right;
  std::cout << "\nHow many motors are on your RIGHT drivetrain? ";
  std::cin >> port_number_right;

  int leftPorts[port_number_left];
  int rightPorts[port_number_right];

  for (int i = 0; i < port_number_left; i++) {
    std::cout << "\nEnter port LEFT number" << i + 1 << "/" << port_number_left
              << " : ";
    std::cin >> leftPorts[i];
  }

  for (int i = 0; i < port_number_right; i++) {
    std::cout << "\nEnter port RIGHT number" << i + 1 << "/"
              << port_number_right << " : ";
    std::cin >> rightPorts[i];
  }

  int vert_odom_count;

  std::cout << "\nHow many vertical rotation sensors do you have for odom? ";
  std::cin >> vert_odom_count;

  float vert_ports[vert_odom_count][4];

  for (int i = 0; i < vert_odom_count; i++) {
    std::cout << "\nEnter port number" << i + 1 << "/" << vert_odom_count
              << " : ";
    std::cin >> vert_ports[i][0];
    std::cout
        << "\nWhat is the distance of this port from the center of the bot? ";
    std::cin >> vert_ports[i][1];
    std::cout
        << "\nWhat is the gear ratio for the sensor to acutal wheel spins? ";
    std::cin >> vert_ports[i][2];
    std::cout << "\nWhat is the diameter of the wheel? ";
    std::cin >> vert_ports[i][3];
  }

  int horz_odom_count;

  std::cout << "\nHow many horizontal rotation sensors do you have for odom? ";
  std::cin >> horz_odom_count;

  float horz_ports[horz_odom_count][4];

  for (int i = 0; i < horz_odom_count; i++) {
    std::cout << "\nEnter port number" << i + 1 << "/" << horz_odom_count
              << " : ";
    std::cin >> horz_ports[i][0];
    std::cout
        << "\nWhat is the distance of this port from the center of the bot? ";
    std::cin >> horz_ports[i][1];
    std::cout
        << "\nWhat is the gear ratio for the sensor to acutal wheel spins? ";
    std::cin >> horz_ports[i][2];
    std::cout << "\nWhat is the diameter of the wheel? ";
    std::cin >> horz_ports[i][3];
  }

  int inertial_port = 0;
  std::string hasIMU;
  std::cout << "\nDo you have an inertial sensor? Y/N : ";
  std::cin >> hasIMU;

  if (hasIMU == "Y") {
    std::cout << "\nEnter IMU port: ";
    std::cin >> inertial_port;
  }

  float trackWidth;

  std::cout
      << "\nWhat is the distance between the center of your left and right "
         "wheels? ";
  std::cin >> trackWidth;

  int RPM;
  std::cout << "\n What is your drivetrain RPM? ";
  std::cin >> RPM;

  int diameter;
  std::cout << "\n What is your drivetrain wheel diameter? ";
  std::cin >> diameter;

  std::cout << "Generating code";
  std::cout.flush();
  std::this_thread::sleep_for(std::chrono::seconds(1));
  std::cout << "\rGenerating code.";
  std::cout.flush();
  std::this_thread::sleep_for(std::chrono::seconds(1));
  std::cout << "\rGenerating code..";
  std::cout.flush();
  std::this_thread::sleep_for(std::chrono::seconds(1));
  std::cout << "\rGenerating code...";
  std::cout.flush();
  std::this_thread::sleep_for(std::chrono::seconds(1));

  std::cout << "\npros::MotorGroup leftMotors({";
  for (int i = 0; i < port_number_left; i++) {
    std::cout << leftPorts[i];
    if (i < port_number_left - 1) {
      std::cout << ", ";
    }
  }
  std::cout << "});";

  std::cout << "\npros::MotorGroup rightMotors({";
  for (int i = 0; i < port_number_right; i++) {
    std::cout << rightPorts[i];
    if (i < port_number_right - 1) {
      std::cout << ", ";
    }
  }
  std::cout << "});";

  std::cout << "\nlemlib::Drivetrain DT(&leftMotors, &rightMotors,"
            << trackWidth << " ," << diameter << ", " << RPM << ", 0);";

  for (int i = 0; i < vert_odom_count; i++) {
    std::cout << "\npros::Rotation vertRotation" << i + 1 << "("
              << vert_ports[i][0] << ");";
  }
  for (int i = 0; i < horz_odom_count; i++) {
    std::cout << "\npros::Rotation horzRotation" << i + 1 << "("
              << horz_ports[i][0] << ");";
  }
  for (int i = 0; i < vert_odom_count; i++) {
    std::cout << "\nlemlib::TrackingWheel vertT" << i + 1 << "(&"
              << "vertRotation" << i + 1 << ", " << vert_ports[i][3] << ","
              << vert_ports[i][1] << "," << vert_ports[i][2] << " );";
  }
  for (int i = 0; i < horz_odom_count; i++) {
    std::cout << "\nlemlib::TrackingWheel horzT" << i + 1 << "(&"
              << "horzRotation" << i + 1 << ", " << vert_ports[i][3] << ","
              << vert_ports[i][1] << "," << vert_ports[i][2] << " );";
  }
  if (hasIMU == "Y") {
    std::cout << "\npros::IMU inertial(" << inertial_port << ");";
    hasIMU = "&inertial";
  } else {
    hasIMU = "nullptr";
  }
  std::cout << "\nlemlib::OdomSensors sensors("
            << ((vert_odom_count > 0) ? "&vertT1, " : "nullptr, ")
            << ((vert_odom_count > 1) ? "&vertT2, " : "nullptr, ")
            << ((horz_odom_count > 0) ? "&horzT1, " : "nullptr, ")
            << ((horz_odom_count > 1) ? "&horzT2, " : "nullptr, ") << hasIMU
            << ");";
  std::cout << "\nlemlib::ControllerSettings lateral_controller(5, 0, 5.5, 3, "
               "0.5, 100, 1.5, 200, 20);";

  std::cout << "\nlemlib::ControllerSettings lateral_controller(5, 0, 5.5, 3, "
               "0.5, 100, 1.5, 200, 0);";
  std::cout << "\nlemlib::chassis chassis(DT, lateral_controller, "
               "angular_controller, sensors);";
}