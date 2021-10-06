// #include <Arduino.h>


// #include <ros.h>
// #include <std_msgs/Float64.h>

// ros::NodeHandle nh;

// void messageCb(const std_msgs::Float64& msg)
// {
//   if(msg.data > 1.0)
    
//     digitalWrite(13, HIGH);   // blink the led
//     else 
//     digitalWrite(13, LOW);    // Turn off led
// }

// ros::Subscriber<std_msgs::Float64> sub("led_toggle", &messageCb);

// void setup()
// {
//   pinMode(13, OUTPUT);
//   nh.initNode();
//   nh.subscribe(sub);
// }
// void loop()
// {

//  nh.spinOnce();
//  delay(1);
// }