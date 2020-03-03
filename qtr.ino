//#include <QTRSensors.h>
#include <ArduinoSort.h>
 const int SensorCount = 6;
 int sensorValues[SensorCount];
  bool turn = false;
  bool obstacle = false;
  bool sensor;
  bool node;
  bool line_following=false;
  bool status1= LOW;
  
  int binary_node [4];
  float node_value;
  float reference1 = 0; //primary
  float reference2 = 0; //secondary
  float error_value=0;
  float area_moment=0;
  float area=0;
  
  int qtr1[8];
  int qtr2[8];
  int qtr1_min [8];
  int qtr1_max [8];
  int qtr2_min [8];
  int qtr2_max [8];
  char str1[7];
  char str2[7];
  

  int en_left = 9;
  int leftPin_1 = 7;
  int leftPin_2 = 8;
  int en_right = 5;
  int rightPin_1 = 3;
  int rightPin_2 = 4;

float binary_converter()
{
  float value = 0; 
  
  for (int i = 0; i < 4; i++)
  {
    value = value + binary_node[i]*pow(2,i);
    Serial.print(binary_node[i]);
    Serial.print("\t");
    Serial.print(binary_node[i]*pow(2,i));
    Serial.print("\t");
    Serial.println(value);
    
  }

  return value;
}

void read_qtr_1 ()
{
  qtr1[0] = analogRead(A0);
  qtr1[1] = analogRead(A1);
  qtr1[2] = analogRead(A2);
  qtr1[3] = analogRead(A3);
  qtr1[4] = analogRead(A4);
  qtr1[5] = analogRead(A5);
  qtr1[6] = analogRead(A6);
  qtr1[7] = analogRead(A7);
  //Serial.print("Front Sensor ");
  for(int i = 0, j = 0; i < 8; i++)
  {
    if(i == 2 || i == 5)
      continue;
    if(qtr1[i] > 600)
     str1[j] = 'B';
    else
     str1[j] = 'W'; 
   //Serial.print(str1[j]);
   j++; 
  }
//  Serial.print(" ");
//  for(int i = 0; i < 8; i++)
//  {
//    if(i == 2 || i == 5)
//      continue;
//     Serial.print(qtr1[i]);  
//     Serial.print(" ");
//  }
//  Serial.println(); 
}

void read_qtr_2 ()
{
  qtr2[7] = analogRead(A8);
  qtr2[6] = analogRead(A9);
  qtr2[5] = analogRead(A10);
  qtr2[4] = analogRead(A11);
  qtr2[3] = analogRead(A12);
  qtr2[2] = analogRead(A13);
  qtr2[1] = analogRead(A14);
  qtr2[0] = analogRead(A15);
   //Serial.print("Back Sensor ");
//  for(int i = 0, j = 0; i <  8; i++)
//  {
//    if(i == 2 || i == 5)
//      continue;
//    if(qtr2[i] > 600)
//     str2[j] = 'B';
//    else
//     str2[j] = 'W'; 
//   j++;
//  }
//   //Serial.print(" ");
//  for(int j = 0; j <=  5; j++)
//  {
//     Serial.print(str2[j]); 
//  }
//   Serial.print(" ");
//  for(int i = 0; i < 8; i++)
//  {
//    if(i == 2||i == 5)
//      continue;
//   Serial.print(qtr2[i]);  
//   Serial.print(" ");
//  }
   //Serial.println();
}

void calibrate_qtr(int sensor)
{
  if (sensor == 1)
  {
    for (int i=0; i<8; i++)
    {
      Serial.print("#####");
      Serial.println(i);
      qtr1_max[i] = 0;
      qtr2_max[i] = 0;
      for (int j=0; j<400; j++)
      {
        read_qtr_1();
        read_qtr_2();
        if (qtr1_max [i] < qtr1 [i]) 
        {
          qtr1_max [i] = qtr1 [i];
          Serial.print("qtr1\t");
          Serial.println(qtr1_max[i]);
        }
        if (qtr2_max [i] < qtr2 [i]) 
        {
          qtr2_max [i] = qtr2 [i];
          Serial.print("qtr2\t");
          Serial.println(qtr2_max[i]);
        }
        delay(5);
      }
    }
  }

  else if (sensor == 0)
  {
    for (int i=0; i<8; i++)
    {
      qtr1_min [i] = 1000;
      qtr2_min [i] = 1000;
      for (int j=0; j<400; j++)
      {
        read_qtr_1();
        read_qtr_2();
        if (qtr1_min [i] > qtr1 [i]) qtr1_min [i] = qtr1 [i];
        if (qtr2_min [i] > qtr2 [i]) qtr2_min [i] = qtr2 [i];
        delay(5);
      }
    }
  }
}

void check_obstacle()      //check obstacle presence
{
  if(digitalRead(sensor) == 0 )
   {
     obstacle = true;
   }
  else
     obstacle = false;
}

void check_turn ()
{
  if (strcmp(str1,"WWWWWB") == 0|| strcmp(str1,"WWWWBB") == 0 || strcmp(str1,"WWWBBB") == 0 || strcmp(str1,"WWBBBB") == 0 || strcmp(str1,"WBBBBB") == 0||strcmp(str1,"BBBBBB") == 0)
   {
     turn = true;
   }
   else if(strcmp(str1,"BBBBBW") == 0|| strcmp(str1,"BBBBWW") == 0 || strcmp(str1,"BBBWWW") == 0 || strcmp(str1,"BBWWWW") == 0||strcmp(str1,"BWWWWW") == 0)
   {
     turn = true; 
   }
}

int max_value_index(float ar[])      //function to return index of maximum valued element from an array
{      
  int index = 0;
  int temp = ar[0];
  for (int i = 0; i < 8; i++)
  {
    if(temp <= ar[i])
    {
      temp = ar[i];
      index = i;
    }
  }
  return index;
}

float error(int sensor_reading[])
{
  int x_coordinate[3];           // storing index of maximum valued sensor
  float duplicate[8];
  float dupinception[8];
  area=0;
  area_moment=0;
  
  for(int i = 0; i < 8; i++)    //duplicating input
  { 
    if ( i == 2 || i== 5) continue;
    if (sensor_reading[i]<500)
    sensor_reading[i]=0;
//    if(i<4)              
//     duplicate[i] = ( log(abs(3.5-(i-1))) * float((sensor_reading[i] > 500)));
//     else
//     duplicate[i] = ( log(abs(3.5-(i+1))) * float((sensor_reading[i] > 500)));

     duplicate[i] = ( pow(abs(3.5-(i)),4.0) * float(sensor_reading[i]));
     
     area_moment += (3.5-i)*duplicate[i];
     area += duplicate[i];
//     Serial.print ("index is ");
//     Serial.println (i);
//     Serial.print ("area moment");
//     Serial.println (abs(area_moment));
//     Serial.print("area is");
//     Serial.println(area);
  }
      
//error calculation
  if (area != 0)
  error_value = area_moment/area;
  //error_value = error - 3.50;  
  else
  error_value = 0;
  
  
  Serial.print ("error is \t");
  Serial.println(error_value);
  Serial.println("************************************************");
  return error_value;
}

void read_qtr()
{
    read_qtr_1();
    qtr1 [0] = map (qtr1[0],qtr1_min[0], qtr1_max[0], 0, 1000);
    qtr1 [1] = map (qtr1[1],qtr1_min[1], qtr1_max[1], 0, 1000);
    qtr1 [2] = map (qtr1[2],qtr1_min[2], qtr1_max[2], 0, 1000);
    qtr1 [3] = map (qtr1[3],qtr1_min[3], qtr1_max[3], 0, 1000);
    qtr1 [4] = map (qtr1[4],qtr1_min[4], qtr1_max[4], 0, 1000);
    qtr1 [5] = map (qtr1[5],qtr1_min[5], qtr1_max[5], 0, 1000);
    qtr1 [6] = map (qtr1[6],qtr1_min[6], qtr1_max[6], 0, 1000);
    qtr1 [7] = map (qtr1[7],qtr1_min[7], qtr1_max[7], 0, 1000);

    read_qtr_2();
    qtr2 [0] = map (qtr2[0],qtr2_min[0], qtr2_max[0], 0, 1000);
    qtr2 [1] = map (qtr2[1],qtr2_min[1], qtr2_max[1], 0, 1000);
    qtr2 [2] = map (qtr2[2],qtr2_min[2], qtr2_max[2], 0, 1000);
    qtr2 [3] = map (qtr2[3],qtr2_min[3], qtr2_max[3], 0, 1000);
    qtr2 [4] = map (qtr2[4],qtr2_min[4], qtr2_max[4], 0, 1000);
    qtr2 [5] = map (qtr2[5],qtr2_min[5], qtr2_max[5], 0, 1000);
    qtr2 [6] = map (qtr2[6],qtr2_min[6], qtr2_max[6], 0, 1000);
    qtr2 [7] = map (qtr2[7],qtr2_min[7], qtr2_max[7], 0, 1000);
 
  Serial.print("front sensor  ");
  for(int i = 0, j = 0; i <  8; i++)
  {
    Serial.print(qtr1[i]);
    Serial.print("  ");
    if(i == 2 || i == 5)
      continue;
      
    if(qtr2[i] > 500)
     str2[j] = 'B';
    else
     str2[j] = 'W'; 
     
    if(qtr1[i] > 500)
     str1[j] = 'B';
    else
     str1[j] = 'W';
     
   j++;
  }
  Serial.print("\t");
  for(int j = 0; j <=  5; j++)
  {
     Serial.print(str1[j]);
     Serial.print("  ");
  }
  Serial.println();
  Serial.print("back sensor  ");
  for(int j = 0; j <  8; j++)
  {
     Serial.print(qtr2[j]);
     Serial.print("  ");
  }
  Serial.print("\t");
  for(int j = 0; j <=  5; j++)
  {
     Serial.print(str2[j]);
     Serial.print("  ");
  }
  Serial.println();
   
}

void setup() 
{
  str1[6] = '\0';
  str2[6] = '\0'; 
 
  delay(500);
  Serial.begin(9600);
  Serial.println ("calibrating sensors. ... ... ....");
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); // turn on led during calibration
  
  Serial.println ("calibrating minimum value of sensor . .. .... .....");
  calibrate_qtr(0);
  Serial.println ("minimum value calibration done !!");
  Serial.print ("front sensor minimum values \t");
  for(int i=0; i<8; i++)
  {
    Serial.print (qtr1_min[i]);
    Serial.print (" ");
  }
  Serial.println();
  Serial.print ("back sensor minimum values \t");
  for(int i=0; i<8; i++)
  {
    Serial.print (qtr2_min[i]);
    Serial.print (" ");
  }
  Serial.println();

  digitalWrite(LED_BUILTIN, LOW);
  delay (300);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(300);
  digitalWrite(LED_BUILTIN, LOW);
  delay (300);
  digitalWrite(LED_BUILTIN, HIGH);
  
  Serial.println ("calibrating maximum value of sensor . .. .... .....");
  calibrate_qtr(1);
  Serial.println ("maximum vaue calibration done !!");
  Serial.print ("front sensor maximum values \t");
  for(int i=0; i<8; i++)
  {
    Serial.print (qtr1_max[i]);
    Serial.print (" ");
  }
  Serial.println();
  Serial.print ("back sensor maximum values \t");
  for(int i=0; i<8; i++)
  {
    Serial.print (qtr2_max[i]);
    Serial.print (" ");
  }
  Serial.println();
  
  digitalWrite(LED_BUILTIN, LOW); // turn off Arduino's LED to indicate we are through with calibration
  Serial.println ("calibration done !!");
  Serial.println ("********************************");
  
  delay(1000);
  
  pinMode(en_left, OUTPUT);
  pinMode(leftPin_1, OUTPUT);
  pinMode(leftPin_2, OUTPUT);
  pinMode(en_right, OUTPUT);
  pinMode(rightPin_1, OUTPUT);
  pinMode(rightPin_2, OUTPUT);
  pinMode(6, INPUT);
// forward
  digitalWrite(7,HIGH);
  digitalWrite(8,LOW);
  digitalWrite(3,HIGH);
  digitalWrite(4,LOW);

  analogWrite(9,150);
  analogWrite(5,150);
}

void loop() {
  status1 = digitalRead (6);
   if (status1 == HIGH)
   line_following = true;
   if (line_following) {

 read_qtr();                     //read qtrs
 //check_obstacle();              //check obstacle presence
 //check_turn();                 //presence of turn before all white
 reference1 = error(qtr1);    //position of black line on primary sensor array
 reference2 = error(qtr2);   //position of black line on secondary sensor array
  
  //Reading first 1x2 node cell
  if(((strcmp(str1,"WWWWWW") == 0 ) && turn == false && obstacle == false || ((strcmp(str1,"WBBBBW") == 0  || strcmp(str1, "BBBBBW") == 0 || strcmp(str1, "WBBBBB") == 0)&& abs(reference1)<2) || (abs(reference1) > 2 && abs(reference2) < 2) && (strcmp(str1, "WWWWWW")!=0|| strcmp(str1,"WBBBBW") != 0  || strcmp(str1, "BBBBBW") != 0 || strcmp(str1, "WBBBBB") != 0 )) && node == false)
    {
      Serial.println("1x2 Node detected 1x2");
      Serial.print(str1);
      Serial.print("\t");
      Serial.println(str2);
      Serial.print(reference1);
      Serial.print("\t");
      Serial.println(reference2);
      Serial.println(reference1-reference2);
      node = true;

  //store binary node values
      if (strcmp(str1,"WWWWWW") == 0)
        {
          binary_node[0] = 0;
          binary_node[1] = 0;
        }
      else if (strcmp(str1,"WBBBBW") == 0 || strcmp(str1, "BBBBBW") == 0 || strcmp(str1, "WBBBBB") == 0)
       {
          binary_node[0] = 1;
          binary_node[1] = 1;
       }
      else if(reference1 > 2)
       {
          binary_node[0] = 0;
          binary_node[1] = 1;
        }
      else if(reference1 < -2)
       {
          binary_node[0] = 1;
          binary_node[1] = 0;
        }
        Serial.print("####################binary values are: \t1: ");
        Serial.print(binary_node[0]);
        Serial.print("\t2: ");
        Serial.println(binary_node[1]);
    }
    
    read_qtr();                  //read qtrs
   //check_obstacle();              //check obstacle presence
// check_turn();                 //presence of turn before all white
 reference1 = error(qtr1);    //position of black line on primary sensor array
 reference2 = error(qtr2);   //position of black line on secondary sensor array

  //Reading second 1x2 node cell
  if (node == true)
    {
      Serial.println("Yo, we are in");
      if ((((strcmp(str2,"WWWWWW") && turn == false && obstacle == false)|| (strcmp(str2,"WBBBBW") == 0 ) && strcmp (str1, "WWBBWW")) || (strcmp(str2, "BBBBBW") == 0 ) || (strcmp(str2, "WBBBBB") == 0)) && abs(reference1) < 2 || (abs(reference2) > 2 && abs(reference1) < 2))
        {
          
          Serial.println("2x2 Node detected 2x2");
          Serial.print(str1);
          Serial.print("\t");
          Serial.println(str2);
          Serial.print(reference1);
          Serial.print("\t");
          Serial.println(reference2);
          Serial.println(reference1-reference2);
          
      //store binary node values
          if (strcmp(str1,"WWWWWW") == 0)
            {
              binary_node[2] = 0;
              binary_node[3] = 0;
            }
         else if (strcmp(str2,"WBBBBW") == 0 || strcmp(str2, "BBBBBW") == 0 || strcmp(str2, "WBBBBB") == 0)
            {
              binary_node[2] = 1;
              binary_node[3] = 1;
            }
          else if(reference2 > 2.0)
            {
              binary_node[2] = 0;
              binary_node[3] = 1;
            }
          else if(reference2 < -2.0)
            {
              binary_node[2] = 1;
              binary_node[3] = 0;
            }
            Serial.print("####################binary values are: \t3: ");
        Serial.print(binary_node[2]);
        Serial.print("\t4: ");
        Serial.println(binary_node[3]);
            node_value = binary_converter();
            Serial.print("The value of the node is ");
            Serial.println(node_value);
            Serial.println("#############################");
            node = false;
        }
     }


     Serial.print(reference1);
     Serial.print("\t");
     Serial.println(reference2);
     Serial.println(reference1-reference2);
     delay (70);
}
}
