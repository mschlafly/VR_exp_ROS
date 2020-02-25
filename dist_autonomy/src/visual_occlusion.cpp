
// This ros node recieved strings with coordinate through a TCP socket and
// publishes them to ros topic /input using custom message user_input/input_array

// For compiling not in ros
//$ gcc client.c -o client
//$ ./client

using namespace std;

#include<stdio.h>
#include<cmath>

#include "ros/ros.h"
#include "dist_autonomy/Target_dist.h"
#include "user_input/person_position.h"
#include "signal.h"

//////// PROTOTYPES ////////
// Callback function for person position
void chatterCallback(const user_input::person_position::ConstPtr& msg);
void fill_building_array(string complexity);
bool isgridpointinsight(int x, int z, float p_x, float p_z);
////////////////////////////

//////// GLOBAL VARIABLES ////////
// The distribution is defined as follows
  // All grid points that the person can see gets value of 0
  // Other points get values from 1-10 depending on the distance to the person
  // Above and below a threshold, the distance to the person does not affect the
      // weight- those values can be set here.
float mindist = 3;
float maxdist = 17;
float minweight = 1;
float maxweight = 10;

// Person position (gets updated by subscriber callback)
float person_x = 13;
float person_z = 7;
float person_th = 1.57;
bool start_publishing = true;

// Building and visual field arrays
int arraysize = 30;
int building_array[30][30] = { 0 }; // all elements 0
int visual_field_dist[30][30];
/////////////////////////////////

void chatterCallback(const user_input::person_position::ConstPtr& msg)
{
  // printf("Updating the person's position\n");
  person_x = msg->xpos;
  person_z = msg->ypos;
  person_th = msg->theta;
  start_publishing = true;
}

int main(int argc , char *argv[])
{
    // Set up ros node
    ros::init(argc, argv, "visual_occlusion");
    ros::NodeHandle n;
    ros::Rate rate(1.0);

    // Set up publisher
    ros::Publisher pub = n.advertise<dist_autonomy::Target_dist>("visual_dist", 1000);
    dist_autonomy::Target_dist send_array;
    // Send empty array so that the topic can be found even if no inputs have been made
    send_array.target_array.clear();
    pub.publish(send_array);
    ros::spinOnce();

    // Set up Subscriber
    ros::Subscriber sub = n.subscribe("person_position", 1000, chatterCallback);

    // slope is used to define the weight of location linearly according to the person's location
    // locations closer to the person are weighted higher
    float slope = -(maxweight-minweight)/(maxdist-mindist);
    float y0 = (maxweight*maxdist-minweight*mindist)/(maxdist-mindist);
    //printf("slope: %f \n", slope);

    // Get rosparam for whether this is a high or low complexity trial
    string env;
    n.getParam("/complexity_level",env);
    // ROS_INFO_STREAM(env << "\n");

    fill_building_array(env);
    // printf("finished \n");
    // Continuously loops - quits when ctl-C is clicked
    // int count = 0;
    //printf("count %d \n", count);
    while (ros::ok()) {
      if (start_publishing==false) {
        printf("person_position has not been published\n");
      } else {
        // printf(" x-%f y-%f th-%f\n", person_x, person_z, person_th);

        bool checksight = false;
        // Clear array before filling
        send_array.target_array.clear();
        for (int x = 0; x < arraysize; x++)
        {
            for (int z = 0; z < arraysize; z++)
            {
                //send_array.arr.push_back(building_array[x][z]);
                // printf("bld value %d \n",building_array[x][z]);

                // Compute the vector from the person to the grid point x=(Xg-Xp) and z=(Zg-Zp)
                float v2grid_x = (x+0.5)-person_x;
                float v2grid_z = (z+0.5)-person_z;
                float dist = sqrt(pow(v2grid_x,2)+pow(v2grid_z,2)); // units

                // Check if the grid point is in the player's sight (true)
                // printf("checking sight. \n");
                checksight = isgridpointinsight(x,z,person_x,person_z);
                // send_array.target_array.push_back(building_array[x][z]);
                if (checksight==true) {
                    // visual_field_dist[x][z] = 0;
                    send_array.target_array.push_back(0);
                } else {
                    int weight = round(slope*dist+y0);
                    //printf("slope: %f mag: %f weight: %d \n",slope,dist,weight);
                    if (weight < minweight) {
                        weight = minweight;
                    } else if (weight > maxweight) {
                        weight = maxweight;
                    }
                    //count++;
                    //printf("count %d \n", count);
                    // visual_field_dist[x][z] = weight;
                    send_array.target_array.push_back(weight);
                }
            }
        }

        // Publish array

        //send_array.arr = visual_field_dist;
        pub.publish(send_array);
        // printf("Published\n", );
        ros::spinOnce();
        rate.sleep();

        //count++;
        //printf("count %d \n", count);
      }
    }
	  return 0;
}

//
bool isgridpointinsight(int x, int z, float p_x, float p_z) {

    // Both of these need to be true for the the person to be in sight
    bool is_facing_point = false;
    bool is_path_on_building = false;
    bool is_within_line_of_sight = false;

    // To determine whether the person can see the grid point, compute the dot product
    // between a vector to the grid point x=(Xg-Xp) and z=(Zg-Zp) normalized and
    // the vector in the direction the person is looking

    // Compute the vector from the person to the grid point x=(Xg-Xp) and z=(Zg-Zp)
    float v2grid_x = (x+0.5)-p_x;
    float v2grid_z = (z+0.5)-p_z;
    float mag = sqrt(pow(v2grid_x,2)+pow(v2grid_z,2)); // units
    float v2grid_x_normalized = v2grid_x/mag;
    float v2grid_z_normalized = v2grid_z/mag;
    float v2vision_x = cos(-(person_th-(M_PI/2))); // It is already normalized
    float v2vision_z = sin(-(person_th-(M_PI/2)));

    // printf("x-%d z-%d v2vision_x-%f z-%f v2grid_x_normalized-%f z-%f\n", x, z, v2vision_x, v2vision_z, v2grid_x_normalized, v2grid_z_normalized);
    float dotproduct = v2vision_x*v2grid_x_normalized + v2vision_z*v2grid_z_normalized;
    // printf("dotproduct: %f \n", dotproduct);
    if (dotproduct>(cos(M_PI/4))) // Can see 45 degrees right and left
    {
        is_facing_point = true;
    }
    if  (is_facing_point) {  // Only do this test is the first is true

      if (mag < 10.0)
      {
        is_within_line_of_sight = true;

        int num_tests = 50; // Check this many points along vector_to_player

        float v2grid_x_step = v2grid_x/num_tests;
        float v2grid_z_step = v2grid_z/num_tests;
        float prev_position_x = p_x;
        float prev_position_z = p_z;
        float new_position_x, new_position_z;
        // printf("v2grid_x_step \n", );

      // if (x==29) {
        int i = 0;
        // Check the length of the vector for buildings; stop if found
        while ((is_path_on_building == false) && (i <= num_tests)) {
            new_position_x = prev_position_x + v2grid_x_step;
            new_position_z = prev_position_z + v2grid_z_step;

            // Check for buildings only if you have entered a new grid space along the vector.
            if ((floor(new_position_x)==floor(prev_position_x)) && (floor(new_position_z)==floor(prev_position_z))) {
            }
            else {
                // printf("floor of val %f is %d \n", new_position_x, (int)floor(new_position_x));
                int bld_x = (int)floor(new_position_x);
                int bld_z = (int)floor(new_position_z);
                if (building_array[bld_x][bld_z]==1) {
                    is_path_on_building = true;
                    // printf("path on building \n");
                }
            }
            prev_position_x = new_position_x;
            prev_position_z = new_position_z;
            i++;
        }
        }
      // }
    }
    bool iswithinsight = (is_facing_point && is_within_line_of_sight && (is_path_on_building == false));
    return (iswithinsight);
}
// Creates a building array containing 0 where building are located and 1 where they aren't
// To be called once at the beginning
void fill_building_array(string complexity)
{
    printf("Filling building array \n");
    // Add small buildings
    int bld_small_len = 100;
    int bld_small_x_unity[bld_small_len] = {0, 1, 4, 5, 14, 15, 24, 25, 28, 29, // Row 1.1
                     0, 1, 4, 5, 14, 15, 24, 25, 28, 29, // Row 1.2
                     0, 1, 4, 5, 14, 15, 24, 25, 28, 29, // Row 2.1
                     0, 1, 4, 5, 14, 15, 24, 25, 28, 29, // Row 2.2
                     0, 1, 4, 5, 14, 15, 24, 25, 28, 29, // Row 5.1
                     0, 1, 4, 5, 14, 15, 24, 25, 28, 29, // Row 5.2
                     0, 1, 4, 5, 14, 15, 24, 25, 28, 29, // Row 8.1
                     0, 1, 4, 5, 14, 15, 24, 25, 28, 29, // Row 8.2
                     0, 1, 4, 5, 14, 15, 24, 25, 28, 29, // Row 9.1
                     0, 1, 4, 5, 14, 15, 24, 25, 28, 29}; // Row 9.2
    int bld_small_y_unity[bld_small_len] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // Row 1.1
                     1, 1, 1, 1, 1, 1, 1, 1, 1, 1, // Row 1.2
                     4, 4, 4, 4, 4, 4, 4, 4, 4, 4, // Row 2.1
                     5, 5, 5, 5, 5, 5, 5, 5, 5, 5, // Row 2.2
                     14, 14, 14, 14, 14, 14, 14, 14, 14, 14, // Row 5.1
                     15, 15, 15, 15, 15, 15, 15, 15, 15, 15, // Row 5.1
                     24, 24, 24, 24, 24, 24, 24, 24, 24, 24, // Row 8.1
                     25, 25, 25, 25, 25, 25, 25, 25, 25, 25, // Row 8.2
                     28, 28, 28, 28, 28, 28, 28, 28, 28, 28, // Row 9.1
                     29, 29, 29, 29, 29, 29, 29, 29, 29, 29 }; // Row 9.2
    // Add horizontal buildings
    int bld_2horiz_len = 80;
    int bld_2horiz_x_unity[bld_2horiz_len] = {8, 9, 10, 11, 18, 19, 20, 21, // Row 1.1
                      8, 9, 10, 11, 18, 19, 20, 21, // Row 1.2
                      8, 9, 10, 11, 18, 19, 20, 21, // Row 2.1
                      8, 9, 10, 11, 18, 19, 20, 21, // Row 2.2
                      8, 9, 10, 11, 18, 19, 20, 21, // Row 5.1
                      8, 9, 10, 11, 18, 19, 20, 21, // Row 5.2
                      8, 9, 10, 11, 18, 19, 20, 21, // Row 8.1
                      8, 9, 10, 11, 18, 19, 20, 21, // Row 8.2
                      8, 9, 10, 11, 18, 19, 20, 21, // Row 9.1
                      8, 9, 10, 11, 18, 19, 20, 21 }; // Row 9.2
    int bld_2horiz_y_unity[bld_2horiz_len] = {0, 0, 0, 0, 0, 0, 0, 0, // Row 1.1
                      1, 1, 1, 1, 1, 1, 1, 1, // Row 1.2
                      4, 4, 4, 4, 4, 4, 4, 4, // Row 2.1
                      5, 5, 5, 5, 5, 5, 5, 5, // Row 2.2
                     14, 14, 14, 14, 14, 14, 14, 14, // Row 5.1
                     15, 15, 15, 15, 15, 15, 15, 15, // Row 5.2
                     24, 24, 24, 24, 24, 24, 24, 24, // Row 8.1
                     25, 25, 25, 25, 25, 25, 25, 25, // Row 8.2
                     28, 28, 28, 28, 28, 28, 28, 28,  // Row 9.1
                     29, 29, 29, 29, 29, 29, 29, 29 }; // Row 9.2
    // Add vertical buildings
    int bld_2vert_len = 80;
    int bld_2vert_x_unity[bld_2vert_len] = {0, 1, 4, 5, 14, 15, 24, 25, 28, 29, // Row 3/4.1
                   0, 1, 4, 5, 14, 15, 24, 25, 28, 29, // Row 3/4.2
                   0, 1, 4, 5, 14, 15, 24, 25, 28, 29, // Row 3/4.3
                   0, 1, 4, 5, 14, 15, 24, 25, 28, 29, // Row 3/4.4
                   0, 1, 4, 5, 14, 15, 24, 25, 28, 29, // Row 6/7.1
                   0, 1, 4, 5, 14, 15, 24, 25, 28, 29, // Row 6/7.2
                   0, 1, 4, 5, 14, 15, 24, 25, 28, 29, // Row 6/7.3
                   0, 1, 4, 5, 14, 15, 24, 25, 28, 29 }; // Row 6/7.4
    int bld_2vert_y_unity[bld_2vert_len] = {8, 8, 8, 8, 8, 8, 8, 8, 8, 8, // Row 3.1
                   9, 9, 9, 9, 9, 9, 9, 9, 9, 9, // Row 3.2
                   10, 10, 10, 10, 10, 10, 10, 10, 10, 10, // Row 4.1
                   11, 11, 11, 11, 11, 11, 11, 11, 11, 11, // Row 4.2
                   18, 18, 18, 18, 18, 18, 18, 18, 18, 18, // Row 6.1
                   19, 19, 19, 19, 19, 19, 19, 19, 19, 19, // Row 6.2
                   20, 20, 20, 20, 20, 20, 20, 20, 20, 20, // Row 7.1
                   21, 21, 21, 21, 21, 21, 21, 21, 21, 21 // Row 7.2
                     };
    // Add large buildings
    int bld_large_len = 64;
    int bld_large_x_unity[bld_large_len] = {8, 9, 10, 11, 18, 19, 20, 21,  // Row 3/4.1
                    8, 9, 10, 11, 18, 19, 20, 21,  // Row 3/4.2
                    8, 9, 10, 11, 18, 19, 20, 21,  // Row 3/4.3
                    8, 9, 10, 11, 18, 19, 20, 21,  // Row 3/4.4
                    8, 9, 10, 11, 18, 19, 20, 21,  // Row 6/7.1
                    8, 9, 10, 11, 18, 19, 20, 21,  // Row 6/7.2
                    8, 9, 10, 11, 18, 19, 20, 21,  // Row 6/7.3
                    8, 9, 10, 11, 18, 19, 20, 21 };  // Row 6/7.4
    int bld_large_y_unity[bld_large_len] = {8, 8, 8, 8, 8, 8, 8, 8,  // Row 3/4.1
                   9, 9, 9, 9, 9, 9, 9, 9, // Row 3/4.2
                   10, 10, 10, 10, 10, 10, 10, 10, // Row 3/4.3
                   11, 11, 11, 11, 11, 11, 11, 11, // Row 3/4.4
                   18, 18, 18, 18, 18, 18, 18, 18, // Row 6/7.1
                   19, 19, 19, 19, 19, 19, 19, 19, // Row 6/7.2
                   20, 20, 20, 20, 20, 20, 20, 20, // Row 6/7.3
                   21, 21, 21, 21, 21, 21, 21, 21 }; // Row 6/7.4
    for (int i = 0; i < bld_small_len; i++)
    {
        // printf("Update small building location for %d, %d \n", bld_small_x_unity[i], bld_small_y_unity[i]);
        building_array[bld_small_x_unity[i]][bld_small_y_unity[i]] = 1;
        // printf("Update small building location with %d \n", building_array[bld_small_x_unity[i]][bld_small_y_unity[i]]);
    }
    for (int i = 0; i < bld_2horiz_len; i++)
    {
        building_array[bld_2horiz_x_unity[i]][bld_2horiz_y_unity[i]] = 1;
    }
    for (int i = 0; i < bld_2vert_len; i++)
    {
        building_array[bld_2vert_x_unity[i]][bld_2vert_y_unity[i]] = 1;
    }
    for (int i = 0; i < bld_large_len; i++)
    {
        building_array[bld_large_x_unity[i]][bld_large_y_unity[i]] = 1;
        // printf("Update large building location for i %d \n", i);
    }

    if (complexity.compare("low") == 0){
        // ROS_INFO_STREAM("Here!");
        int bush_len = 6*8+4*4+8*2;
        int bush_list_x[bush_len] = {4, 5, 18, 19, 20, 21,  // Row 2.1
                          4, 5, 18, 19, 20, 21,  // Row 2.2
                          8, 9, 10, 11, 24, 25, // Row 3.1
                          8, 9, 10, 11, 24, 25, // Row 3.2
                          8, 9, 10, 11, 18, 19, 20, 21, // Row 4.1
                          8, 9, 10, 11, 18, 19, 20, 21, // Row 4.2
                          4, 5, 20, 21, // Row 5.1
                          4, 5, 20, 21, // Row 5.2
                          8, 9, 10, 11, 24, 25, // Row 6.1
                          8, 9, 10, 11, 24, 25, // Row 6.2
                          8, 9, 10, 11, 18, 19, // Row 7.1
                          8, 9, 10, 11, 18, 19, // Row 7.2
                          4, 5, 24, 25, // Row 8.1
                          4, 5, 24, 25}; // Row 8.2
        int bush_list_y[bush_len] = {4, 4, 4, 4, 4, 4, // Row 2.1
                        5, 5, 5, 5, 5, 5, // Row 2.2
                        8, 8, 8, 8, 8, 8, // Row 3.1
                        9, 9, 9, 9, 9, 9, // Row 3.2
                        10, 10, 10, 10, 10, 10, 10, 10, // Row 4.1
                        11, 11, 11, 11, 11, 11, 11, 11, // Row 4.2
                        14, 14, 14, 14, // Ros 5.1
                        15, 15, 15, 15, // Row 5.2
                        18, 18, 18, 18, 18, 18, // Row 6.1
                        19, 19, 19, 19, 19, 19, // Row 6.2
                        20, 20, 20, 20, 20, 20, // Row 7.1
                        21, 21, 21, 21, 21, 21, // Row 7.2
                        24, 24, 24, 24, // Row 8.1
                        25, 25, 25, 25}; // Row 8.2
        for (int i = 0; i < bush_len; i++)
        {
            building_array[bush_list_x[i]][bush_list_y[i]] = 0;
        }
    }
}
