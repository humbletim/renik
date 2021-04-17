// interim WASM test harness

#include "renik.h"

#undef assert
#include "renTest/renTest.cpp"

int main(int argc, char **argv) {
  RenIKTest test;
  test.test();

  Quat rightAngle = RenIKHelper::align_vectors(Vector3(1, 0, 0), Vector3(0, 1, 0));
  Quat rightAngleCheck = Quat(Vector3(0, 0, 1), Math::deg2rad(90.0));
  
  printf("rightAngle %s\n", glm::to_string(rightAngle).c_str());
  printf("rightAngleCheck %s\n", glm::to_string(rightAngleCheck).c_str());
  printf("Vector3(1,0,0) %s\n", glm::to_string(Vector3(1, 0, 0)).c_str());
}
