// #include <Arduino.h>
// #include <ros.h>
// #include <std_msgs/Float64.h>
// ros::NodeHandle nh1;
// int EN1 = 9; // Pin 1 of L293D IC
// int EN2 = 10;
// int speed = 255;
// void messageCb(const std_msgs::Float64 &msg)
// {
//   if (msg.data == 1.0)
//   {
//     analogWrite(EN1, speed); 
//     analogWrite(EN2, speed); 
//     digitalWrite(5, HIGH);
//     digitalWrite(6, LOW);

//     digitalWrite(7, HIGH);
//     digitalWrite(8, LOW);
//   }
//   else if (msg.data == 2.0)
//   {
//     analogWrite(EN1, speed); 
//     analogWrite(EN2, speed); 
//     digitalWrite(5, LOW);
//     digitalWrite(6, HIGH);

//     digitalWrite(7, LOW);
//     digitalWrite(8, HIGH);
//   }
//   else if (msg.data == 3.0)
//   {
//     analogWrite(EN1, speed); 
//     analogWrite(EN2, speed); 
//     digitalWrite(5, HIGH);
//     digitalWrite(6, LOW);

//     digitalWrite(7, LOW);
//     digitalWrite(8, HIGH);
//   }
//   else if (msg.data == 4.0)
//   {
//     analogWrite(EN1, speed); //sets the motors speed
//     analogWrite(EN2, speed); //sets the motors speed
//     digitalWrite(5, LOW);
//     digitalWrite(6, HIGH);

//     digitalWrite(7, HIGH);
//     digitalWrite(8, LOW);
//   }
//   else if (msg.data == 5.0)
//   {
//     analogWrite(EN1, speed); //sets the motors speed
//     analogWrite(EN2, speed); //sets the motors speed
//     digitalWrite(5, LOW);
//     digitalWrite(6, LOW);

//     digitalWrite(7, LOW);
//     digitalWrite(8, LOW);
//   }
//   else
//   {
//     analogWrite(EN1, speed); //sets the motors speed
//     analogWrite(EN2, speed); //sets the motors speed
//     digitalWrite(5, LOW);
//     digitalWrite(6, LOW);

//     digitalWrite(7, LOW);
//     digitalWrite(8, LOW);
//   }
// }

// ros::Subscriber<std_msgs::Float64> sub("cmd_vel", &messageCb);

// void setup()
// {
//   pinMode(EN1, OUTPUT); // where the motor is connected to
//   pinMode(EN2, OUTPUT); // where the motor is connected to
//   pinMode(5, OUTPUT);
//   pinMode(6, OUTPUT);
//   pinMode(7, OUTPUT);
//   pinMode(8, OUTPUT);
//   nh1.initNode();
//   nh1.getHardware()->setBaud(115200);
//   nh1.subscribe(sub);
// }
// void loop()
// {

//   nh1.spinOnce();
//   delay(1);
// }
