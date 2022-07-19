#include <DynamixelWorkbench.h>

#define BOUDRATE 1000000
#define ACCELERATION 50    //Объявление баудрейта общения, ускорения и айдишников динамикселей.
#define DXL_ID1 1
#define DXL_ID2 2

uint8_t id1 = DXL_ID1;
uint8_t id2 = DXL_ID2;
int32_t acc = ACCELERATION;

DynamixelWorkbench dx;    //Создаем объект ДинамиксельВоркбенч для взаимоднйствия с динамикселями.

double* goalWheelVel;
double* realWheelVel;

double L = 0.155;         //Записываем в переменные расстояние между колесами и радиус самих колес
double R = 0.05;

bool turtleMove(uint8_t id, float vel);
double getRealWheelVel(DynamixelWorkbench &dx, uint8_t id);

void setup() {

  Serial.begin(9600);
  Serial.setTimeout(3);
  while(!Serial);

  goalWheelVel = new double[2];
  goalWheelVel[0] = 0.0;        //Выделяем память под двумерный массив и заполняем его нулями
  goalWheelVel[1] = 0.0;

  realWheelVel = new double[2];
  realWheelVel[0] = 0.0;
  realWheelVel[1] = 0.0;
  
  dx.init("", BOUDRATE);

  dx.ping(id1);
  dx.ping(id2);
  
  dx.wheelMode(id1, acc);
  dx.wheelMode(id2, acc);
}

void loop() {
  
  if (Serial.available() > 0) {
    String str = Serial.readString();     //Считываем из сериал порта целевую скорость
    parsing(str);
    setGoalRobotVel();
  }
}

void setGoalRobotVel() {  //Функция, преобразующая целевую скорость робота в целевые скорости колес.
  goalWheelVel[0] = (2 * goalRobotVel[0] - L * goalRobotVel[1]) / (2 * R);
  goalWheelVel[1] = (2 * goalRobotVel[0] + L * goalRobotVel[1]) / (2 * R);

  turtleMove(id1, goalWheelVel[0]);
  turtleMove(id2, goalWheelVel[1]);

  Serial.println(goalWheelVel[0]);
  Serial.println(goalWheelVel[1]);

  dx.ping(id1);
  dx.ping(id2);
}
 
bool turtleMove(uint8_t id, float vel) //Непосредственно задаем скорости.
{
  return dx.goalVelocity(id, vel);
}

void parsing(String str) { //Парсер
  int divider = str.indexOf(';');
  String buf = str.substring(0, divider);
  goalRobotVel[0] = buf.toFloat();
  buf = str.substring(divider + 1);
  goalRobotVel[1] = buf.toFloat();
}