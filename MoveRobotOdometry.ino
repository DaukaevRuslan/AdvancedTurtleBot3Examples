#include <DynamixelWorkbench.h>


#define BOUDRATE 1000000
#define ACCELERATION 50
#define DXL_ID1 1    //Объявление баудрейта общения, ускорения и адйишников динамикселей
#define DXL_ID2 2

uint8_t id1 = DXL_ID1;
uint8_t id2 = DXL_ID2;
int32_t acc = ACCELERATION;
int32_t millRequest;  //Таймеры для опрашивания динамикселей и вывода данных в сериал порт
int32_t millSerial;

DynamixelWorkbench dx;  //Создаем объект ДинамиксельВоркбенч для взаимодействия с динамикселями

double* goalWheelVel;
double* realWheelVel;
double* goalRobotVel;
double* realRobotVel;
double* odometry;

double X, Y, A;


bool turtleMove(uint8_t id, float vel);
double getRealWheelVel(DynamixelWorkbench &dx, uint8_t id);
void setGoalRobotVel(double *goalRobotVel, double *goalWheelVel);
void calcOdometry(double *realWheelVel, double *realRobotVel, double *odometry);
void serialPrint(double *realwheelVel, double *realRobotVel, double *odometry);

double L = 0.155; //Записываем в переменные расстояние между колесами и радиус самих колес
double R = 0.05;

void setup() {

  Serial.begin(9600);
  Serial.setTimeout(3);
  while(!Serial);
  const char *log;
  bool rez;

  millRequest = millis(); //Начинаем отсчет времени
  millSerial = millis();

  goalWheelVel = new double[2];
  realWheelVel = new double[2];
  goalRobotVel = new double[2];
  realRobotVel = new double[4]; //Выделяем память под двумерные массивы и заполняем его нулями
  odometry = new double[2];
  
  goalWheelVel[0] = 0.0;
  goalWheelVel[1] = 0.0;

  realWheelVel[0] = 0.0;
  realWheelVel[1] = 0.0;

  goalRobotVel[0] = 0.0;
  goalRobotVel[1] = 0.0;

  realRobotVel[0] = 0.0;
  realRobotVel[1] = 0.0;
  realRobotVel[2] = 0.0;
  realRobotVel[3] = 0.0;

  odometry[0] = 0.0;
  odometry[1] = 0.0;
  
  dx.init("", BOUDRATE); //Инициализация объекта Воркбенч

  dx.ping(id1); //Пингуем оба динамикселя
  dx.ping(id2);
  
  dx.wheelMode(id1, acc); //Включаем на обоих сервоприводах режим колеса
  dx.wheelMode(id2, acc);
}

void loop() {

  if (Serial.available() > 0) {
    String str = Serial.readString();     //Считываем из сериал порта целевую скорость
    parsing(str);
  }

  setGoalRobotVel(goalRobotVel, goalWheelVel); //Решение ОЗК
  
  turtleMove(id1, goalWheelVel[0]); //Отправка скоростей на привода
  turtleMove(id2, goalWheelVel[1]);

  if (millis() - millRequest > 100) //10 раз в секунду опрашиваем привода, узнаем их реальную скорость
  {
    realWheelVel[0] = getRealWheelVel(dx1, id1);
    realWheelVel[1] = getRealWheelVel(dx2, id2);

    calcOdometry(realWheelVel, realRobotVel, odometry); //и 10 раз в секунду считываем одометрию

    millRequest = millis();
  }

  if (millis() - millSerial > 500) //Дважды в секунду отправляем данные в сериал порт
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

void parsing(String str) { //Парсер
  int divider = str.indexOf(';');
  String buf = str.substring(0, divider);
  goalRobotVel[0] = buf.toFloat();
  buf = str.substring(divider + 1);
  goalRobotVel[1] = buf.toFloat();
}
