#include <DynamixelWorkbench.h>

#define BOUDRATE 1000000
#define ACCELERATION 50
#define DXL_ID1 1
#define DXL_ID2 2

uint8_t id1 = DXL_ID1;
uint8_t id2 = DXL_ID2;
int32_t acc = ACCELERATION;

DynamixelWorkbench dx1;
DynamixelWorkbench dx2;

double* goalWheelVel;
double* realWheelVel;

bool turtleMove(DynamixelWorkbench &dx, uint8_t id, float vel);
double getRealWheelVel(DynamixelWorkbench &dx, uint8_t id);

void setup() {

  Serial.begin(9600);
  while(!Serial);
  const char *log;
  bool rez;

  goalWheelVel = new double[2];
  realWheelVel = new double[2];
  
  goalWheelVel[0] = 1.0;        // Здесь задаем скорость колес
  goalWheelVel[1] = 1.0;

  realWheelVel[0] = 0.0;
  realWheelVel[1] = 0.0;
  
  dx1.init("", BOUDRATE);
  dx2.init("", BOUDRATE);

  dx1.ping(id1);
  dx2.ping(id2);
  
  dx1.wheelMode(id1, acc);
  dx2.wheelMode(id2, acc);
  
}

void loop() {
  
  turtleMove(dx1, id1, goalWheelVel[0]);
  turtleMove(dx2, id2, goalWheelVel[1]);
  
  realWheelVel[0] = getRealWheelVel(dx1, id1);
  realWheelVel[1] = getRealWheelVel(dx2, id2);

  Serial.print("Real_Velocity_Wheel_Left: ");
  Serial.print(realWheelVel[0]);
  Serial.print(",Real_Velocity_Wheel_Right: "); 
  Serial.println(realWheelVel[1]);

  dx1.ping(id1);
  dx2.ping(id2);
}


bool turtleMove(DynamixelWorkbench &dx, uint8_t id, float vel)
{
  return dx.goalVelocity(id, vel);
}
double getRealWheelVel(DynamixelWorkbench &dx, uint8_t id)
{ 
  int32_t tempVel = 0;
  dx.itemRead(id, "Present_Velocity", &tempVel);
  delay(5);
  double vel = tempVel * 0.0247;
  return vel;
}
