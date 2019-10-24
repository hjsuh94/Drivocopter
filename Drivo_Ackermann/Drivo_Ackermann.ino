#include "src/Drivo/drivo.h"
#include "src/R12DS_Ack/r12ds_ack.h"

using namespace Drivocopter;

Drivo drivo_;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(57600);
  drivo_.initialize();
}

void loop() {
  // put your main code here, to run repeatedly:
  drivo_.update_cmd();
}
