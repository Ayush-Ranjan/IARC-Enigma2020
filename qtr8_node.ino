#include  <ArduinoSort.h> 
  bool turn;
  bool obstacle;
  bool sensor;
  bool node;
  
  int binary_node [4];
  int node_value;
  float reference1 = 0; //primary
  float reference2 = 0; //secondary
  float error_value=0;
  
  int qtr1[8];
  int qtr2[8];
  char str1[7];
  char str2[7];
  

  int en_left = 9;
  int leftPin_1 = 7;
  int leftPin_2 = 8;
  int en_right = 5;
  int rightPin_1 = 3;
  int rightPin_2 = 4;

int binary_converter( int binary[4] )
{
  int value = 0; 
  
  for (int i = 0; i < 4; i++)
  {
    value = value + binary[i]*pow(2,i);
    Serial.print(binary[i]);
    Serial.print("\t");
    Serial.print(binary[i]*pow(2,i));
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
  Serial.print("Front Sensor ");
  for(int i = 0, j = 0; i < 8; i++)
  {
    if(i == 2 || i == 5)
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
    if(i == 2 || i == 5)
      continue;
     Serial.print(qtr1[i]);  
     Serial.print(" ");
  }
  Serial.println(); 
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
   Serial.print("Back Sensor ");
  for(int i = 0, j = 0; i <  8; i++)
  {
    if(i == 2 || i == 5)
      continue;
    if(qtr2[i] > 500)
     str2[j] = 'B';
    else
     str2[j] = 'W'; 
   j++;
  }
   Serial.print(" ");
  for(int j = 0; j <=  5; j++)
  {
     Serial.print(str2[j]); 
  }
   Serial.print(" ");
  for(int i = 0; i < 8; i++)
  {
    if(i == 2||i == 5)
      continue;
   Serial.print(qtr2[i]);  
   Serial.print(" ");
  }
   Serial.println();
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

int max_value_index(int ar[])      //function to return index of maximum valued element from an array
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
  int duplicate[8];
  int dupinception[8];
  
  for(int i = 0; i < 8; i++)    //duplicating input
  {                 
     duplicate[i] = (50.0 / pow(abs(3.5 - i), 0.3) * (sensor_reading[i] > 500));
     dupinception[i] = duplicate[i];
  }
    if ( dupinception [0] + dupinception [1] + dupinception [2] + dupinception [3] + dupinception [4] + dupinception [5] + dupinception [6] + dupinception [7] != 0)  {
    for(int j = 0; j < 3; j++)        // finding index of maximum valued sensor
  {
    x_coordinate[j] =  max_value_index(duplicate);
    duplicate[max_value_index(duplicate)] = 0;
  }
  sortArray(x_coordinate,3);

//error calculation
  float a = (dupinception[x_coordinate[0]] + dupinception[x_coordinate[2]] -(2*dupinception[x_coordinate[1]]))/3;   //  [a = (y1 + y3 -(2*y2))/3]
  float b = dupinception[x_coordinate[1]] - dupinception[x_coordinate[0]] - (2*a*x_coordinate[0]) - a;              //  [b = (y2 - y1 - (2*a*x1) - a]
  float x_position = (-1)*b/(2*a);  //  {x = (-1)*b/(2*a)]
  error_value = x_position - 3.5;  }
  else error_value =0;

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
  str1[6] = '\0';
  str2[6] = '\0'; 
}

void loop() {

 read_qtr_1();                    //read primary QTR
 read_qtr_2();                   //read secondary QTR
 check_obstacle();              //check obstacle presence
 check_turn();                 //presence of turn before all white
 reference1 = error(qtr1);    //position of black line on primary sensor array
 reference2 = error(qtr2);   //position of black line on secondary sensor array
  
  //Reading first 1x2 node cell
  if(((strcmp(str1,"WWWWWW") == 0 ) && turn == false && obstacle == false || strcmp(str1,"WBBBBW") == 0  || strcmp(str1, "BBBBBW") == 0 || strcmp(str1, "WBBBBB") == 0 || (abs(reference1) > 1.5 && abs(reference2) < 1.5)) && node == false)
    {
      Serial.println("1x2 Node detected 1x2");
      Serial.print(str1);
      Serial.print("\t");
      Serial.println(str2);
      Serial.print(reference1);
      Serial.print("\t");
      Serial.println(reference2);
      node = true;

  //store binary node values
      if (strcmp(str1,"WWWWWW") == 0)
        {
          binary_node[0] = 0;
          binary_node[1] = 0;
        }
      if (strcmp(str1,"WBBBBW") == 0 || strcmp(str1, "BBBBBW") == 0 || strcmp(str1, "WBBBBB") == 0)
       {
          binary_node[0] = 1;
          binary_node[1] = 1;
       }
      if(reference1 > 0)
       {
          binary_node[0] = 0;
          binary_node[1] = 1;
        }
      if(reference1 < 0)
       {
          binary_node[0] = 1;
          binary_node[1] = 0;
        }
    }
    
    read_qtr_1();                 //read primary QTR
   read_qtr_2();                   //read secondary QTR
   check_obstacle();              //check obstacle presence
 check_turn();                 //presence of turn before all white
 reference1 = error(qtr1);    //position of black line on primary sensor array
 reference2 = error(qtr2);   //position of black line on secondary sensor array
  //Reading second 1x2 node cell
  if (node == true)
    {
      Serial.println("Yo, we are in");
      if (((strcmp(str2,"WWWWWW") && turn == false && obstacle == false)|| (strcmp(str2,"WBBBBW") == 0 ) || (strcmp(str2, "BBBBBW") == 0 ) || (strcmp(str2, "WBBBBB") == 0)) && abs(reference1) < 1 || (abs(reference2) < 1.5 && abs(reference1) > 1.5))
        {
          
          Serial.println("2x2 Node detected 2x2");
          Serial.print(str1);
          Serial.print("\t");
          Serial.println(str2);
          Serial.print(reference1);
          Serial.print("\t");
          Serial.println(reference2);
          
      //store binary node values
          if (strcmp(str1,"WWWWWW") == 0)
            {
              binary_node[2] = 0;
              binary_node[3] = 0;
            }
          if (strcmp(str1,"WBBBBW") == 0 || strcmp(str1, "BBBBBW") == 0 || strcmp(str1, "WBBBBB") == 0)
            {
              binary_node[2] = 1;
              binary_node[3] = 1;
            }
          if(reference2 > 0)
            {
              binary_node[2] = 0;
              binary_node[3] = 1;
            }
          if(reference2 < 0)
            {
              binary_node[2] = 1;
              binary_node[3] = 0;
            }
            node_value = binary_converter(binary_node);
            Serial.print("The value of the node is ");
            Serial.println(node_value);
            node = false;
        }
     }


     Serial.print(reference1);
     Serial.print("\t");
     Serial.println(reference2);
     delay (500);
}
