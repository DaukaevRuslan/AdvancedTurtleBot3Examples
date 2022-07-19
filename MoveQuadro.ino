#include <DynamixelWorkbench.h>


#define BOUDRATE 1000000
#define ACCELERATION 50
#define DXL_ID1 1
#define DXL_ID2 2

uint8_t id1 = DXL_ID1;
uint8_t id2 = DXL_ID2;
int32_t acc = ACCELERATION;
int32_t millRequest;
int32_t millSerial;

DynamixelWorkbench dx;

double* goalWheelVel;
double* realWheelVel;
double* goalRobotVel;
double* realRobotVel;
double* odometry;

double X, Y, A;
int count = 1;


bool turtleMove(uint8_t id, float vel);
double getRealWheelVel(DynamixelWorkbench &dx, uint8_t id);
void setGoalRobotVel(double *goalRobotVel, double *goalWheelVel);
void calcOdometry(double *realWheelVel, double *realRobotVel, double *odometry);
void serialPrint(double *realwheelVel, double *realRobotVel, double *odometry);


void setup() {

  Serial.begin(9600);
  while(!Serial);
  const char *log;
  bool rez;

  millRequest = millis();
  millSerial = millis();

  goalWheelVel = new double[2];
  realWheelVel = new double[2];
  goalRobotVel = new double[2];
  realRobotVel = new double[4];
  odometry = new double[2];
  
  goalWheelVel[0] = 0.0;
  goalWheelVel[1] = 0.0;

  realWheelVel[0] = 0.0;
  realWheelVel[1] = 0.0;

  goalRobotVel[0] = 0.0;
  goalRobotVel[1] = 0.0 ;

  realRobotVel[0] = 0.0;
  realRobotVel[1] = 0.0;
  realRobotVel[2] = 0.0;
  realRobotVel[3] = 0.0;

  odometry[0] = 0.0;
  odometry[1] = 0.0;
  
  dx.init("", BOUDRATE);

  dx.ping(id1);
  dx.ping(id2);
  
  dx.wheelMode(id1, acc);
  dx.wheelMode(id2, acc);
  
}

void loop() {

  switch(count){ //Конечный автомат
    case 1:
    if (X < 0.3){
      goalRobotVel[0] = 0.1;
      goalRobotVel[1] = 0.0;
    } else count = 2;
    break;
    case 2:
    if (A > -1.57){
      goalRobotVel[0] = 0.0;
      goalRobotVel[1] = -0.4;
    } else count = 3;
    break;
    case 3:
    if (Y > -0.3){
      goalRobotVel[0] = 0.1;
      goalRobotVel[1] = 0.0;
    } else count = 4;
    break;
    case 4:
    if (A > -3.14){
      goalRobotVel[0] = 0.0;
      goalRobotVel[1] = -0.4;
    } else count = 5;
    break;
    case 5:
    if (X > 0){
      goalRobotVel[0] = 0.1;
      goalRobotVel[1] = 0.0;
    } else count = 6;
    break;
    case 6:
    if (A > -4.71){
      goalRobotVel[0] = 0.0;
      goalRobotVel[1] = -0.4;
    } else {
      count = 7;
    }
    break;
    case 7:
    if (Y < 0){
      goalRobotVel[0] = 0.1;
      goalRobotVel[1] = 0.0;
    } else count = 8;
    break;
    case 8:
    if (A > -6.28){
      goalRobotVel[0] = 0.0;
      goalRobotVel[1] = -0.4;
    } else count = 0;
    break;
    default:
      goalRobotVel[0] = 0.0;
      goalRobotVel[1] = 0.0;
    break;
  }

  setGoalRobotVel(goalRobotVel, goalWheelVel);
  
  turtleMove(id1, goalWheelVel[0]);
  turtleMove(id2, goalWheelVel[1]);

  if (millis() - millRequest > 100)
  {
    realWheelVel[0] = getRealWheelVel(dx1, id1);
    realWheelVel[1] = getRealWheelVel(dx2, id2);

    calcOdometry(realWheelVel, realRobotVel, odometry);

    millRequest = millis();
  }

  if (millis() - millSerial > 500)
  {
    serialPrint(realWheelVel, realRobotVel, odometry); 
    millSerial = millis();
  }


  dx.ping(id1);
  dx.ping(id2);
  
}


bool turtleMove(uint8_t id, float vel)
{
  return dx.goalVelocity(id, vel);
}
double getRealWheelVel(DynamixelWorkbench &dx, uint8_t id) //Считываение реальной скорости из привода.
{ 
  int32_t tempVel = 0;
  dx.itemRead(id, "Present_Velocity", &tempVel);
  delay(5);
  double vel = tempVel * 0.0247;
  return vel;
}
void setGoalRobotVel(double *goalRobotVel, double *goalWheelVel) //Функция, преобразуюзщая целевую скорость робота в целевые скорости колес
{    
    goalWheelVel[0] = (2 * goalRobotVel[0] - 0.155 * goalRobotVel[1]) / (2 * 0.05);
    goalWheelVel[1] = (2 * goalRobotVel[0] + 0.155 * goalRobotVel[1]) / (2 * 0.05);
}
void calcOdometry(double *realWheelVel, double *realRobotVel, double *odometry) //Функция, считывающая одометрию
{
  realRobotVel[0] = (0.05 * (realWheelVel[0] + realWheelVel[1])) / 2;
  realRobotVel[1] = (0.05 * (realWheelVel[1] - realWheelVel[0])) / 0.155; //ПЗК

  double delta_s = ((realRobotVel[0] + realRobotVel[2]) / 2) * 0.1;
  double delta_a = ((realRobotVel[1] + realRobotVel[3]) / 2) * 0.1;

  odometry[0] += delta_s;
  odometry[1] += delta_a;

  A += delta_a;
  X += (delta_s * cos(A));
  Y += (delta_s * sin(A));

  realRobotVel[2] = realRobotVel[0];
  realRobotVel[3] = realRobotVel[1];
}
void serialPrint(double *realwheelVel, double *realRobotVel, double *odometry) //Вывод данных в сериал порт
{
  Serial.print("Real_Velocity_Wheel_Left: ");
  Serial.print(realWheelVel[0]);
  Serial.print(",Real_Velocity_Wheel_Right: "); 
  Serial.println(realWheelVel[1]);

  Serial.print("Real_Velocity_Robot_Linear: ");
  Serial.print(realRobotVel[0]);
  Serial.print(", Real_Velocity_Robot_Angular: "); 
  Serial.println(realRobotVel[1]); 

  Serial.print("Odometry_Linear: ");
  Serial.print(odometry[0]);
  Serial.print(", Odometry_Angular: "); 
  Serial.println(odometry[1]);

  Serial.print("X: ");
  Serial.print(X);
  Serial.print(", Y: "); 
  Serial.print(Y);
  Serial.print(", A: "); 
  Serial.println(A * 57.29);
  Serial.println(A);
  Serial.println(" ");

}
