
// This ros node recieved strings with coordinate through a TCP socket and
// publishes them to ros topic /input using custom message user_input/input_array

// https://www.binarytides.com/socket-programming-c-linux-tutorial/ used for making TCP socket

// For setting up in ros - make sure to source!
// source devel/setup.bash
// echo $ROS_PACKAGE_PATH

// For compiling not in ros
//$ gcc client.c -o client
//$ ./client

#define PORT 8888
//#define IPv4 "192.168.1.3"
#define IPv4 "192.168.1.3"

#define MAXLEN 100

#include<stdio.h>
#include<string.h>	//strlen
#include<sys/socket.h>
#include<arpa/inet.h>	//inet_addr
#include<unistd.h>
#include<string.h>


#include "ros/ros.h"
#include "user_input/input_array.h"
#include "user_input/person_position.h"
#include "signal.h"

// Define varables for looping
char previous_message[MAXLEN];
char message_from_server[MAXLEN];
int flag = 0;
int count = 0;

// Define variables used for reformatting strings into two arrays
int flag_exclaim = 0;
int flag_comma = 0;
char coordinate;
char temp_char[6];
int temp_number;
float temp_x, temp_y;
int i, i_temp, i_char;

// Set values to 0 to prepare for the start of input
int num_values_in_message=0;
int comma_index = 0;
// Set done and num_index as if "Done" message was just sent
int done = 0;
int num_index = 2;

using namespace std;
// Define global variables for postition of person_location
float xloc = 15;
float yloc = 15;
float th = 0;
int x_middle = 15;
int y_middle = 15;
string location_string = "?";
char location_chararray[MAXLEN];
char input_mode;
int gridsize_unity = 30;

void chatterCallback(const user_input::person_position::ConstPtr& msg)
{
  // printf("Updating the person's position\n");
  xloc = msg->xpos;
  int xloc_int = (int)(xloc*100+0.5); // add 0.5 for rounding because (int) truncates
  yloc = msg->ypos;
  int yloc_int = (int)(yloc*100+0.5);
  th = msg->theta;
  int th_int = (int)(th*1000);
  location_string = to_string(xloc_int) + ","  + to_string(yloc_int) + "," + to_string(th_int) + "," + "!";
  // printf("location_string %s \n",location_string);
}

int main(int argc , char *argv[])
{
  /////////////////////////////////////////////////////////////////
  //  Set up ros node
  /////////////////////////////////////////////////////////////////
  ros::init(argc, argv, "client");
  ros::NodeHandle n;
  ros::Rate rate(10.0);

	// Set up publisher
  ros::Publisher pub = n.advertise<user_input::input_array>("input", 1000);
  user_input::input_array send_array;
  // Send empty array so that the topic can be found even if no inputs have been made
  send_array.droneID = 10;
  send_array.xinput.clear();
  send_array.yinput.clear();
  send_array.datalen = 0;
  pub.publish(send_array);
  ros::spinOnce();

  // Set up Subscriber
  ros::Subscriber sub = n.subscribe("person_position", 1000, chatterCallback);

  /////////////////////////////////////////////////////////////////
  // Setting up TCP socket
  /////////////////////////////////////////////////////////////////
	int socket_desc;
	struct sockaddr_in server;
	char *message, server_reply[MAXLEN];

	//Create socket
	socket_desc = socket(AF_INET , SOCK_STREAM , 0);
	if (socket_desc == -1)
	{
		printf("Could not create socket");
	}

	server.sin_addr.s_addr = inet_addr(IPv4);
	server.sin_family = AF_INET;
	server.sin_port = htons(PORT);

	//Connect to remote server
	if (connect(socket_desc , (struct sockaddr *)&server , sizeof(server)) < 0)
	{
		puts("connect error");
		return 1;
	}
	puts("Connected to Windows Server");

  /////////////////////////////////////////////////////////////////
  // Testing connection
  /////////////////////////////////////////////////////////////////
	//Send data for test
	message = "?Testing connection";
	if( send(socket_desc , message , strlen(message) , 0) < 0)
	{
		puts("Send failed");
		return 1;
	}
	puts("Data Send");
	//Receive a reply from the server
	if( recv(socket_desc, server_reply , MAXLEN , 0) < 0)
	{
		puts("recv failed");
	}
	puts("Reply received");
	puts("Test completed");

  // Define variables for conversion
  int H_tablet = 780;
  int H_unity_global = 30;
  int H_unity_local = 10;
  float zoom_global = H_tablet/H_unity_global;
  float zoom_local = H_tablet/H_unity_local;

  /////////////////////////////////////////////////////////////////
  // Continuously publish recieved messages
  /////////////////////////////////////////////////////////////////
  while (ros::ok())
  {
    // Recieve message from server
		if( recv(socket_desc, message_from_server , MAXLEN , 0) < 0)
		{
			puts("recv failed");
		}
    // Print message
		// puts(message_from_server);

    // Publish message when the server is done sending the coordinates
    if ((message_from_server[0]=='D') &&
        (message_from_server[1]=='o') &&
        (message_from_server[2]=='n') &&
        (message_from_server[3]=='e')) {
            // Ensure that only one message is published when done
            if (done==0) {

              // Set array length and publish to /input
              send_array.datalen = num_values_in_message;
              pub.publish(send_array);
              printf("Input coordinates published w/ size %d \n",num_values_in_message);
              ros::spinOnce(); // Standard line to allow services
              sleep(0.5);

              // Clear send_array values and set values to zero to prepare for new message
    		      send_array.xinput.clear();
              send_array.yinput.clear();
              send_array.droneID = 10;
              comma_index = 0;
              num_values_in_message=0;
              send_array.datalen = num_values_in_message;

              ros::spinOnce(); // Standard line to allow services
              sleep(0.5);

              // Publish the empty array
              pub.publish(send_array);
              printf("Empty array published \n");

            }
            done = 1;
    } else
    // Store coordinates in message from server
    {
      puts(message_from_server);
      // Strings follow format "x-coor,y-coor,....x-coor,y-coor,!"
      // Define variables for unpacking string
      flag_exclaim = 0; // flag for exclaimation mark
      flag_comma = 0; // flag for comma
      coordinate = 'x'; // string starts with x-coordinate
      comma_index = 0; // The string starts with the first number so the imaginary comma is at location -1

      // If it the beginning of a new message
      if (done==1) {
          done = 0; // reset done flag because new set of coordinates are being sent
          num_index = 0; // set the num_index that keeps track of the index of the number in the list, important for knowing which is for droneID and mode
      }
      // Iterate through the characters in latest message from server
      for (i=0;i<MAXLEN;i++)
  		{
        if ((int)message_from_server[i]==33) {
          flag_exclaim = 1;
          // printf("String ends \n");
        }
        if ((int)message_from_server[i]==44) {
          flag_comma = 1;
        }
        // If reached the end of a number and the string has not been terminated, store value
        if (flag_exclaim==0 && flag_comma ==1) {
          // Store characters between commas into temporary string and convert to number
          i_temp=0;
          if (comma_index!=0)
            comma_index=comma_index+1;
          for (i_char=comma_index; i_char<i; i_char++) {
            // it cuts off the first number of the first entree
            temp_char[i_temp]=message_from_server[i_char];
            //printf("%d \n",(int)message_from_server[i2]);
            i_temp++;
          }
          temp_char[i_temp]='.'; // atoi recognized a '.' as indicating the end of a number
          temp_number = atoi(temp_char); // from std package
          //printf( "temp_number: %d \n", temp_number);

          // Determine if the number is a droneID, mode, x-coordinate or y-coordinate
          // After the first two values, values alternate between x and y coordinates
          if (num_index == 0) // the first is the view (arial= 'g', local = 'l' )
          {
            input_mode = temp_char[0];
            num_index++;
          } else if (num_index == 1) // the second number in the set of messages is the droneID
          {
            send_array.droneID = temp_number;
            // printf("input_mode: %c\n", temp_char[0]);
            num_index++;
          } else if (coordinate == 'x') {
            temp_x = temp_number;
            coordinate = 'y';
            //printf("coordinate x found %f \n", temp_x);
          } else if (coordinate == 'y') {
            temp_y = temp_number;
            coordinate = 'x';
            //printf("coordinate y found %f \n", temp_y);

            int unity_x=temp_x, unity_y=temp_y;
            float temp_x2, temp_y2;
    //         // printf("input_mode: %s",input_mode)
            // Perform the rest of the coordinate transform depending on input_mode
            if (input_mode=='l') {

              // Rotate backwards relative to person
              temp_x2 = temp_x*cos(-th)+temp_y*sin(-th);
              temp_y2 = -temp_x*sin(-th)+temp_y*cos(-th);

              // The points recieved here are in relative the the person's position
              // and direction in a tanvas's scale
              temp_x2 = temp_x2/zoom_local;
              temp_y2 = temp_y2/zoom_local;

              // Find absolute coordinates not relative
              unity_x = round(temp_x2 + xloc);
              temp_y2 = temp_y2 + (gridsize_unity - yloc); // finish conversion, uncomment when using UWP touch program
              unity_y = round(gridsize_unity - temp_y2); // finish conversion, uncomment when using UWP touch program

              printf("Transformed coordinates- x: %d y: %d \n",unity_x,unity_y);
            } else if (input_mode=='g') {

              // Rotate backwards relative to person 3.1428/2
              temp_x2 = temp_x*cos(-th)+temp_y*sin(-th);
              temp_y2 = -temp_x*sin(-th)+temp_y*cos(-th);

              temp_x2 = temp_x2/zoom_global;
              temp_y2 = temp_y2/zoom_global;

              // // Find absolute coordinates not relative
              unity_x = round(temp_x2 + x_middle);
              temp_y2 = temp_y2 + (gridsize_unity - y_middle); // finish conversion, uncomment when using UWP touch program
              unity_y = round(gridsize_unity - temp_y2); // finish conversion, uncomment when using UWP touch program
              printf("Transformed coordinates- x: %d y: %d \n",unity_x,unity_y);

            }

            // If within grid, append array message with values
            if ((unity_x>=0) && (unity_x<=29) && (unity_y>=0) && (unity_y<=29)){
                send_array.xinput.push_back(unity_x); // add to message
                send_array.yinput.push_back(unity_y);
                num_values_in_message++;
		            // printf("adds values to message- x: %d y: %d \n ",temp_x3,temp_y3);
            }
          }
          comma_index=i; // Store latest comma location for use later
          flag_comma = 0; // reset flag
        }
      }
		}
    // ROS_INFO_STREAM("looped" << "\n");
    // Reply to server with the person's location
    strcpy(location_chararray,location_string.c_str());
    int stringlen = location_string.length();
    // Send pre-set error message if the the person_position has not been published
    if ((strlen(location_chararray)==0) || (strlen(location_chararray)==1))
    {
       location_chararray[0]='?';
       location_chararray[1]='n';
       location_chararray[2]='o';
       stringlen=5;
       //printf("location set");
    }
		if( send(socket_desc , location_chararray , stringlen , 0) < 0)
		{
			puts("Send failed");
			return 1;
		}
		//puts("Replyed!");
    ros::spinOnce();
    rate.sleep();
    //sleep(1);
	} // Continue looping until ctrl c is pressed
	return 0;
}
