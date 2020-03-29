#include "ros/ros.h"
#include "user_input/person_position.h"
#include "user_input/player_info.h"
#include "user_input/treasure_info.h"


void chatterCallbackPersonPosition(const user_input::person_position::ConstPtr& msg)
{
}

void chatterCallbackAdversaryPosition1(const user_input::person_position::ConstPtr& msg)
{
}

void chatterCallbackAdversaryPosition2(const user_input::person_position::ConstPtr& msg)
{
}

void chatterCallbackAdversaryPosition3(const user_input::person_position::ConstPtr& msg)
{
}
void chatterCallbackPlayerInfo(const user_input::player_info::ConstPtr& msg)
{
}

void chatterCallbackTreasureInfo(const user_input::treasure_info::ConstPtr& msg)
{
}
int main(int argc , char *argv[])
{
  /////////////////////////////////////////////////////////////////
  //  Set up ros node
  /////////////////////////////////////////////////////////////////
  ros::init(argc, argv, "client");
  ros::NodeHandle n;
  ros::Rate rate(10.0);

  // Set up Subscriber
  ros::Subscriber subPlayer = n.subscribe("person_position", 1000, chatterCallbackPersonPosition);
  ros::Subscriber subAdv1 = n.subscribe("adversary_1_position", 1000, chatterCallbackAdversaryPosition1);
  ros::Subscriber subAdv2 = n.subscribe("adversary_2_position", 1000, chatterCallbackAdversaryPosition2);
  ros::Subscriber subAdv3 = n.subscribe("adversary_3_position", 1000, chatterCallbackAdversaryPosition3);
  // ros::Subscriber subGO = n.subscribe("object_position", 1000, chatterCallbackGameObjects);
  ros::Subscriber subTreasure = n.subscribe("treasure_info", 1000, chatterCallbackTreasureInfo);
  ros::Subscriber subLives = n.subscribe("player_info", 1000, chatterCallbackPlayerInfo);

  return 0;
}
