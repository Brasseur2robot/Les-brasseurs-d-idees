#include <Arduino.h>
#include "trajectory_pythagora.h"

double pythagoraCalculation(double x1, double y1, double x2, double y2, bool lengthChoice)
{
  // Converting millimeter in meter
  double height = (y2 - y1) / 1000.0;
  double length = (x2 - x1) / 1000.0;

  double pythagoraResult;

  if (lengthChoice == true) {
    // Caculate the length of the hypothenuse if lengthChoice is true
     pythagoraResult = sqrt(pow(height, 2) + pow(length, 2));
  }
  else {
    // Calculate the angle between base and hypothenuse if lengthChoice is false
    pythagoraResult = atan2(height, length) * 57296 / 1000;
  }

  return pythagoraResult;
}
