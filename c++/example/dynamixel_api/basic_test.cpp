#include <fcntl.h>
#include <iostream>
#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <vector>

#include "dynamixel_sdk/dynamixel_api.hpp"


int getch()
{
  struct termios oldt, newt;
  int ch;
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  ch = getchar();
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  return ch;
}

int kbhit(void)
{
  struct termios oldt, newt;
  int ch;
  int oldf;

  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
  fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

  ch = getchar();

  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  fcntl(STDIN_FILENO, F_SETFL, oldf);

  if (ch != EOF)
  {
    ungetc(ch, stdin);
    return 1;
  }

  return 0;
}

int main() {
  int baudrate = 57600;
  std::cout << "Dynamixel API Source Test Code" << std::endl;
  std::cout << "┌─────[Test Process]────┐" << std::endl;
  std::cout << "│ 1. Ping Test          │" << std::endl;
  std::cout << "│ 2. Torque ON/OFF Test │" << std::endl;
  std::cout << "│ 3. Position Test      │" << std::endl;
  std::cout << "│ 4. Velocity Test      │" << std::endl;
  std::cout << "│ 5. LED Test           │" << std::endl;
  std::cout << "│ 6. Reverse Mode Test  │" << std::endl;
  std::cout << "└───────────────────────┘" << std::endl;
  std::cout << "Baudrate set to: " << baudrate << std::endl;
  std::cout << "Scanning all motors..." << std::endl;

  dynamixel::Connector connector("/dev/ttyUSB0",2.0 , baudrate);
  std::vector<std::unique_ptr<dynamixel::Motor>> motors = connector.getAllMotors();

  if (motors.size() == 0) {
    std::cerr << "No motors found!" << std::endl;
    return 1;
  }
  std::cout << "┌───────────[Motor List]──────────┐" << std::endl;
  for (auto & motor : motors) {
    std::cout << "│ ID: " << static_cast<int>(motor->getID())
              << ", Model: " << motor->getModelName() <<"  │"<< std::endl;
  }
  std::cout << "└─────────────────────────────────┘" << std::endl;
  std::unique_ptr<dynamixel::Motor> motor1 = std::move(motors[0]);
  std::cout << "The test is conducted only on the motor with ID " << static_cast<int>(motor1->getID()) << std::endl;

  std::cout << "Press any key to continue! (or press ESC to quit!)" << std::endl;
  if (getch() == 0x1B)
    return 0;
  auto result_uint16_t = motor1->ping();
  if(!result_uint16_t.isSuccess()){
    std::cerr << dynamixel::getErrorMessage(result_uint16_t.error()) << std::endl;
    return 1;
  }
  std::cout << "┌──────[Ping Test]─────┐" << std::endl;
  std::cout << "│Ping result: " << result_uint16_t.value() << "     │"<< std::endl;
  std::cout << "└──────────────────────┘" << std::endl;

  std::cout << "Press any key to continue! (or press ESC to quit!)" << std::endl;
  if (getch() == 0x1B)
    return 0;
  auto result_void = motor1->enableTorque();
  if(!result_void.isSuccess()){
    std::cerr << dynamixel::getErrorMessage(result_void.error()) << std::endl;
    return 1;
  }

  auto result_uint8_t = motor1->isTorqueOn();
  if(!result_uint8_t.isSuccess()){
    std::cerr << dynamixel::getErrorMessage(result_uint8_t.error()) << std::endl;
    return 1;
  }
  std::cout << "┌──[Torque ON/OFF Test]──┐" << std::endl;
  std::cout << "│Torque ON result: " << static_cast<int>(result_uint8_t.value())<< "     │" << std::endl;

  result_void = motor1->disableTorque();
  if(!result_void.isSuccess()){
    std::cerr << dynamixel::getErrorMessage(result_void.error()) << std::endl;
    return 1;
  }

  result_uint8_t = motor1->isTorqueOn();
  if(!result_uint8_t.isSuccess()){
    std::cerr << dynamixel::getErrorMessage(result_uint8_t.error()) << std::endl;
    return 1;
  }

  std::cout << "│Torque OFF result: " << static_cast<int>(result_uint8_t.value()) << "    │" << std::endl;
  std::cout << "└────────────────────────┘" << std::endl;

  std::cout << "Press any key to continue! (or press ESC to quit!)" << std::endl;
  if (getch() == 0x1B)
    return 0;
  result_void = motor1->setPositionControlMode();
  if(!result_void.isSuccess()){
    std::cerr << dynamixel::getErrorMessage(result_void.error()) << std::endl;
    return 1;
  }
  std::cout << "───────────[Position Control Test]─────────────" << std::endl;
  std::cout << "Position Control Mode set." << std::endl;


  result_void = motor1->enableTorque();
  if(!result_void.isSuccess()){
    std::cerr << dynamixel::getErrorMessage(result_void.error()) << std::endl;
    return 1;
  } else {
    std::cout << "Torque ON." << std::endl;
  }

  int32_t current_pos;
  int target_position = motor1 ->getMaxPositionLimit().value();

  result_void = motor1->setGoalPosition(target_position);
  if(!result_void.isSuccess()){
    std::cerr << dynamixel::getErrorMessage(result_void.error()) << std::endl;
    return 1;
  }
  std::cout << "Target Position:" << target_position << "." << std::endl;

  auto result_int32_t = motor1->getPresentPosition();
  if(!result_int32_t.isSuccess()){
    std::cerr << dynamixel::getErrorMessage(result_int32_t.error()) << std::endl;
    return 1;
  }
  current_pos = result_int32_t.value();

  while (abs(target_position - current_pos) > 10) {
    std::cout << "\rCurrent position: " << current_pos << std::flush;;
    result_int32_t = motor1->getPresentPosition();
    if(!result_int32_t.isSuccess()){
      std::cerr << dynamixel::getErrorMessage(result_int32_t.error()) << std::endl;
      return 1;
    }
    current_pos = result_int32_t.value();
    }
  std::cout << "\rTarget position reached: " << current_pos << std::endl;

  target_position = 0;
  usleep(1000000);

  result_void = motor1->setGoalPosition(target_position);
  if(!result_void.isSuccess()){
    std::cerr << dynamixel::getErrorMessage(result_void.error()) << std::endl;
    return 1;
  } else {
    std::cout << "Target Position: " << target_position << "." << std::endl;
  }

  result_int32_t = motor1->getPresentPosition();
  if(!result_int32_t.isSuccess()){
    std::cerr << dynamixel::getErrorMessage(result_int32_t.error()) << std::endl;
    return 1;
  } else {
    current_pos = result_int32_t.value();
  }

  while (abs(target_position - current_pos) > 5) {
    std::cout << "\rCurrent position: " << current_pos <<"    "<< std::flush;
    result_int32_t = motor1->getPresentPosition();
    if(!result_int32_t.isSuccess()){
      std::cerr << dynamixel::getErrorMessage(result_int32_t.error()) << std::endl;
      return 1;
    }
    current_pos = result_int32_t.value();
  }
  std::cout << "\rTarget position reached: " << current_pos << std::endl;

  result_void = motor1->disableTorque();
  if(!result_void.isSuccess()){
    std::cerr << dynamixel::getErrorMessage(result_void.error()) << std::endl;
    return 1;
  }
  std::cout << "Torque OFF." << std::endl;
  std::cout << "───────────────────────────────────────────────" << std::endl;
  std::cout << "Press any key to continue! (or press ESC to quit!)" << std::endl;
  if (getch() == 0x1B)
    return 0;

  std::cout << "───────────[Velocity Control Test]─────────────" << std::endl;
  result_void = motor1->setVelocityControlMode();
  if(!result_void.isSuccess()){
    std::cerr << dynamixel::getErrorMessage(result_void.error()) << std::endl;
    return 1;
  }
  std::cout << "Velocity Control Mode set." << std::endl;

  result_void = motor1->enableTorque();
  if(!result_void.isSuccess()){
    std::cerr << dynamixel::getErrorMessage(result_void.error()) << std::endl;
    return 1;
  }
  std::cout << "Torque ON." << std::endl;

  int32_t current_vel = motor1 ->getPresentVelocity().value();
  int target_velocity = motor1 ->getVelocityLimit().value();
  std::cout << "Target Velocity: " << target_velocity << std::endl;
  result_void = motor1->setGoalVelocity(target_velocity);
  if(!result_void.isSuccess()){
    std::cerr << dynamixel::getErrorMessage(result_void.error()) << std::endl;
    return 1;
  }
  while (abs(target_velocity - current_vel) > 5) {
    std::cout << "\rCurrent velocity: " << current_vel << std::flush;
    result_int32_t = motor1->getPresentVelocity();
    if(!result_int32_t.isSuccess()){
      std::cerr << dynamixel::getErrorMessage(result_int32_t.error()) << std::endl;
      return 1;
    }
    current_vel = result_int32_t.value();
  }
  std::cout << "\rTarget velocity reached." << current_vel << std::endl;
  usleep(3000000);
  target_velocity = -target_velocity;
  result_void = motor1->setGoalVelocity(target_velocity);
  if(!result_void.isSuccess()){
    std::cerr << dynamixel::getErrorMessage(result_void.error()) << std::endl;
    return 1;
  }
  std::cout << "Goal velocity set to " << target_velocity << "." << std::endl;

  while (abs(target_velocity - current_vel) > 5) {
    std::cout << "\rCurrent velocity: " << current_vel << std::flush;
    result_int32_t = motor1->getPresentVelocity();
    if(!result_int32_t.isSuccess()){
      std::cerr << dynamixel::getErrorMessage(result_int32_t.error()) << std::endl;
      return 1;
    }
    current_vel = result_int32_t.value();
  }

  std::cout << "\rTarget velocity reached." << current_vel << std::endl;
  usleep(3000000);
  result_void = motor1->disableTorque();
  if(!result_void.isSuccess()){
    std::cerr << dynamixel::getErrorMessage(result_void.error()) << std::endl;
    return 1;
  }
  std::cout << "Torque OFF." << std::endl;
  std::cout << "───────────────────────────────────────────────" << std::endl;
  std::cout << "Press any key to continue! (or press ESC to quit!)" << std::endl;
  if (getch() == 0x1B)
    return 0;

  std::cout << "───────────[Direction Test]─────────────" << std::endl;
  result_void = motor1->setReverseDirection();
  if(!result_void.isSuccess()){
    std::cerr << dynamixel::getErrorMessage(result_void.error()) << std::endl;
    return 1;
  }
  std::cout << "Reverse Direction set." << std::endl;

  target_velocity = motor1 ->getVelocityLimit().value();
  motor1->enableTorque();
  motor1->setGoalVelocity(target_velocity);
  std::cout << "Set goal velocity :" << target_velocity << std::endl;
  usleep(3000000);
  motor1->setGoalVelocity(0);
  motor1->disableTorque();
  result_void = motor1->setNormalDirection();
  if(!result_void.isSuccess()){
    std::cerr << dynamixel::getErrorMessage(result_void.error()) << std::endl;
    return 1;
  }
  std::cout << "Normal Direction set." << std::endl;

  std::cout << "Press any key to continue! (or press ESC to quit!)" << std::endl;
  if (getch() == 0x1B)
    return 0;

  std::cout << "───────────[LED Test]─────────────" << std::endl;
  result_void = motor1->LEDOn();
  if(!result_void.isSuccess()){
    std::cerr << dynamixel::getErrorMessage(result_void.error()) << std::endl;
    return 1;
  }
  std::cout << "LED ON." << std::endl;

  result_uint8_t = motor1->isLEDOn();
  if(!result_uint8_t.isSuccess()){
    std::cerr << dynamixel::getErrorMessage(result_uint8_t.error()) << std::endl;
    return 1;
  }
  std::cout << "LED ON result: " << static_cast<int>(result_uint8_t.value()) << std::endl;
  sleep(2);
  result_void = motor1->LEDOff();
  if(!result_void.isSuccess()){
    std::cerr << dynamixel::getErrorMessage(result_void.error()) << std::endl;
    return 1;
  } else {
    std::cout << "LED OFF." << std::endl;
  }

  result_uint8_t = motor1->isLEDOn();
  if(!result_uint8_t.isSuccess()){
    std::cerr << dynamixel::getErrorMessage(result_uint8_t.error()) << std::endl;
    return 1;
  }
  std::cout << "LED OFF result: " << static_cast<int>(result_uint8_t.value()) << std::endl;

  return 0;
}
