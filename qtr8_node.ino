#include <ArduinoSort.h>
  bool turn;
  bool obstacle;
  bool sensor;
  bool node;
  
  int binary_node [4];
  int node_value;
  float reference1=0; //primary
  float reference2=0; //secondary
  
  int qtr1[8];
  int qtr2[8];
  char str1[6];
  char str2[6];

  int en_left = 9;
  int leftPin_1 = 7;
  int leftPin_2 = 8;
  int en_right = 5;
  int rightPin_1 = 3;
  int rightPin_2 = 4;

int binary_converter( int binary[4] )
{
  int value=0; 
  
  for ( int i=0; i<4; i++)
  {
    value+= binary[i]*pow(2,i);
  }
}

void read_qtr_1 ()
{
  qtr1[0]=analogRead(A0);
  qtr1[1]=analogRead(A1);
  qtr1[2]=analogRead(A2);
  qtr1[3]=analogRead(A3);
  qtr1[4]=analogRead(A4);
  qtr1[5]=analogRead(A5);
  qtr1[6]=analogRead(A6);
  qtr1[7]=analogRead(A7);
  Serial.print("Front Sensor ");
  for(int i = 0, j = 0; i < 8; i++)
  {
    if(i == 1||i == 6)
      continue;
    if(qtr1[i] > 500)
     str1[j] = 'B';
    else
     str1[j] = 'W'; 
   Serial.print(str1[j]);  
   j++; 
  }
  Serial.print(" ");
  for(int i = 0; i < 8; i++)
  {
    if(i == 1 || i == 6)
      continue;
   Serial.print(qtr1[i]);  
   Serial.print(" ");
  }
  Serial.println();  
}

void read_qtr_2 ()
{
  qtr2[0]=analogRead(A8);
  qtr2[1]=analogRead(A9);
  qtr2[2]=analogRead(A10);
  qtr2[3]=analogRead(A11);
  qtr2[4]=analogRead(A12);
  qtr2[5]=analogRead(A13);
  qtr2[6]=analogRead(A14);
  qtr2[7]=analogRead(A15);
  Serial.print("Back Sensor ");
  for(int i = 7, j = 5; i >= 0; i--)
  {
    if(i == 1 || i == 6)
      continue;
    if(qtr2[i] > 600)
     str2[j] = 'B';
    else
     str2[j] = 'W'; 
   j--;
  }
  Serial.print(" ");
  for(int j = 0; j <= 5; j++)
  {
    Serial.print(str2[j]); 
  }
  Serial.print(" ");
  for(int i = 0; i < 8; i++)
  {
    if(i == 1||i == 6)
      continue;
   Serial.print(qtr2[i]);  
   Serial.print(" ");
  }
  Serial.println();
}

void check_obstacle()      //check obstacle presence
{
  if(digitalRead(sensor)==0 )
   {
     obstacle = true;
   }
  else
     obstacle = false;
}

void check_turn ()
{
  if (strcmp(str1,"WWWWWB")==0|| strcmp(str1,"WWWWBB")==0 || strcmp(str1,"WWWBBB")==0 || strcmp(str1,"WWBBBB")==0 || strcmp(str1,"WBBBBB")==0||strcmp(str1,"BBBBBB")==0)
   {
     turn=true;
   }
   else if(strcmp(str1,                                                                                                                                "BBBBBW")==0|| strcmp(str1,"BBBBWW")==0 || strcmp(str1,"BBBWWW")==0 || strcmp(str1,"BBWWWW")==0||strcmp(str1,"BWWWWW")==0)
   {
     turn=true; 
   }
}

int max_value_index(int ar[])      //function to return index of maximum valued element from an array
{      
  int index=0;
  int temp = ar[0];
  for (int i=0;i<8;i++)
  {
    if(temp<=ar[i])
    {
      temp=ar[i];
      index=i;
    }
  }
  return index;
}

float error(int sensor_reading[])
{
  int x_coordinate[3];           // storing index of maximum valued sensor
  int duplicate[8];
  
  for(int i =0;i<8;i++)    //duplicating input
  {                 
     duplicate[i] = sensor_reading[i];
  }
    
  for(int j=0; j<3; j++)        // finding index of maximum valued sensor
  {
    x_coordinate[j]= max_value_index(duplicate);
    duplicate[max_value_index(duplicate)]=0;
  }
  sortArray(x_coordinate,3);

//error calculation
  float a = (sensor_reading[x_coordinate[0]] + sensor_reading[x_coordinate[2]] -(2*sensor_reading[x_coordinate[1]]))/3;   //  [a = (y1 + y3 -(2*y2))/3]
  float b = sensor_reading[x_coordinate[1]] - sensor_reading[x_coordinate[0]] - (2*a*x_coordinate[0]) - a;              //  [b = (y2 - y1 - (2*a*x1) - a]
  float x_position = (-1)*b/(2*a);  //  {x = (-1)*b/(2*a)]
  float error_value = x_position - 3.33;  

  return error_value;
}

void setup() 
{
  pinMode(en_left, OUTPUT);
  pinMode(leftPin_1, OUTPUT);
  pinMode(leftPin_2, OUTPUT);
  pinMode(en_right, OUTPUT);
  pinMode(rightPin_1, OUTPUT);
  pinMode(rightPin_2, OUTPUT);
  
// forward
  digitalWrite(7,HIGH);
  digitalWrite(8,LOW);
  digitalWrite(3,HIGH);
  digitalWrite(4,LOW);

  analogWrite(9,150);
  analogWrite(5,150);
  Serial.begin(9600);
  
}

void loop() {

 read_qtr_1();               //read primary QTR
 read_qtr_2();               //read secondary QTR
 check_obstacle();           //check obstacle presence
 check_turn();               //presence of turn before all white
 // reference1 = error(qtr1);    //position of black line on primary sensor array
 // reference2 = error(qtr2);    //position of black line on secondary sensor array
  
  //Reading first 1x2 node cell
  Serial.println("str1 is ");
  Serial.print(str1);
  Serial.println();
  if((strcmp(str1,"WWWWWW")==0 ))//&& turn==false && obstacle==false) || strcmp(str1,"WBBBBW")==0 ) //|| (abs(reference1)>1 && abs(reference2)<1)) 
    {
      Serial.print("Node detected 1x2");
      node=true;

  //store binary node values
      if (strcmp(str1,"WWWWWW")==0);
        {
          binary_node[0] = 0;
          binary_node[1] = 0;
        }
      if (strcmp(str1,"WBBBBW")==0);
       {
          binary_node[0] = 1;
          binary_node[1] = 1;
       }
      if(reference1>0);
       {
          binary_node[0] = 0;
          binary_node[1] = 1;
        }
      if(reference1<0);
       {
          binary_node[0] = 1;
          binary_node[1] = 0;
        }
    }

//Reading second 1x2 node cell
  if (node==true)
    {
      if ((strcmp(str2,"WWWWWW") && turn==false && obstacle==false)|| strcmp(str2,"WBBBBW")==0 || (abs(reference2)>1 && abs(reference1)<1))
        {
          
          Serial.print("Node detected 2x2");
          
      //store binary node values
          if (strcmp(str2,"WWWWWW")==0)
            {
              binary_node[2] = 0;
              binary_node[3] = 0;
            }
          if (strcmp(str2,"WBBBBW")==0);
            {
              binary_node[2] = 1;
              binary_node[3] = 1;
            }
          if(reference2>0);
            {
              binary_node[2] = 0;
              binary_node[3] = 1;
            }
          if(reference2<0);
            {
              binary_node[2] = 1;
              binary_node[3] = 0;
            }
        }
        
       node=false;
     }

     node_value = binary_converter (binary_node);
     delay (500);
}
