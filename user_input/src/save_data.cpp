#include "ros/ros.h"
#include "user_input/person_position.h"
// #include "user_input/object_position.h"
#include "user_input/player_info.h"
#include "user_input/treasure_info.h"
// #include "user_input/adversary_1_position.h"
// #include "user_input/adversary_2_position.h"
// #include "user_input/adversary_3_position.h"

// // Define global variables for postition of the player
// float xloc = 15;
// float yloc = 15;
// float th = 0;
//
// // Define global variables for postition of adversary
// float xPosAdversary;
// float yPosAdversary;
// float thRotAdversary;
//
// // Define global variables for detected game objects position messages that
// // are used for updating information distribution
// float idGO;
// float xPosGO;
// float yPosGO;
// float wGO;
//
// // Define global variables for saving treasure info when found
// float xPosTreasure;
// float yPosTreasure;
// int treasureCount;
//
// // Define global variables for saving player info when lives are lost
// float xPosPlayer;
// float yPosPlayer;
// int livesCount;

void chatterCallbackPersonPosition(const user_input::person_position::ConstPtr& msg)
{
  // printf("Updating the person's position\n");
  // xloc = msg->xpos;
  // yloc = msg->ypos;
  // th = msg->theta;
  // location_string = to_string(round(xloc*100)) + "," + to_string(round(yloc*100)) + "," + to_string(round(th*1000)) + "," + "!";
}

void chatterCallbackAdversaryPosition1(const user_input::person_position::ConstPtr& msg)
{
  // printf("Updating the person's position\n");
  // xPosAdversary1 = msg->xpos;
  // yPosAdversary1 = msg->ypos;
  // thRotAdversary1 = msg->theta;
}

void chatterCallbackAdversaryPosition2(const user_input::person_position::ConstPtr& msg)
{
  // printf("Updating the person's position\n");
  // xPosAdversary2 = msg->xpos;
  // yPosAdversary2 = msg->ypos;
  // thRotAdversary2 = msg->theta;
}

void chatterCallbackAdversaryPosition3(const user_input::person_position::ConstPtr& msg)
{
  // printf("Updating the person's position\n");
  // xPosAdversary3 = msg->xpos;
  // yPosAdversary3 = msg->ypos;
  // thRotAdversary3 = msg->theta;
}
// void chatterCallbackGameObjects(const user_input::object_position::ConstPtr& msg)
// {
//   printf("Updating the object's position\n");
//   idGO = msg->id;
//   xPosGO = msg->xpos;
//   yPosGO = msg->ypos;
//   wGO = msg->weight;
// }

void chatterCallbackPlayerInfo(const user_input::player_info::ConstPtr& msg)
{
  // printf("Updating the Player info:\n");
  // xPosPlayer = msg->xpos;
  // yPosPlayer = msg->ypos;
  // livesCount = msg->lives_count;
}

void chatterCallbackTreasureInfo(const user_input::treasure_info::ConstPtr& msg)
{
  // printf("Updating the Treasure information:\n");
  // xPosTreasure = msg->xpos;
  // yPosTreasure = msg->ypos;
  // treasureCount = msg->treasure_count;
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
