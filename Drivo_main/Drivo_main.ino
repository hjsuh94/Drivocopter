#include "src/Drivo/Drivo.h"

using namespace Drivocopter; 

Drivo drivo_;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  drivo_.initialize();
}

void loop() {
  // put your main code here, to run repeatedly:;
  drivo_.update_cmd();
  drivo_.get_current();
}
