#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/Quaternion.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include <cmath>
#include <tf/transform_datatypes.h>
#include "std_msgs/Bool.h"


enum EState { waiting_for_a_person, 
             observing_the_person, 
             rotating_to_the_person, 
             moving_to_the_person,
             interacting_with_the_person, 
             rotating_to_the_base, 
             moving_to_the_base, 
             resetting_orientation, 
             undefined,
        };

#define frequency_expected 25
#define max_base_distance 6.0
#define detection_threshold 0.5 //threshold for motion detection
#define rotation_treshold M_PI/18


class decision_node
{
private:

    ros::NodeHandle n;

    // communication with datmo_node
    ros::Subscriber sub_person_position;
    bool new_person_position, person_tracked, person_lost;
    geometry_msgs::Point person_position, previous_person_position;

    // communication with robot_moving_node
    ros::Subscriber sub_robot_moving;
    bool robot_moving;

    // communication with rotation_action
    ros::Publisher pub_rotation_to_do;
    float rotation_to_person;

    // communication with action_node
    ros::Publisher pub_goal_to_reach;
    float translation_to_person;

    // communication with localization
    ros::Subscriber sub_localization;

    // QUESTION!
    bool new_localization = false;
    bool init_localization = false;

    bool rotated_to_the_base = false;
    bool moved_to_the_base = false;
    geometry_msgs::Point current_position;
    float current_orientation;
    float translation_to_base;
    float rotation_to_base;
    geometry_msgs::Point local_base_position;

    EState current_state, previous_state;
    int frequency = 0;
    geometry_msgs::Point base_position;
    float base_orientation;
    geometry_msgs::Point origin_position;
    bool state_has_changed;

public:

decision_node()
{

    // communication with datmo_node
    sub_person_position = n.subscribe("person_position", 1, &decision_node::person_positionCallback, this);

    // communication with rotation_node
    pub_rotation_to_do = n.advertise<geometry_msgs::Point>("rotation_to_do", 0);        // Preparing a topic to publish a rotation to perform

    // communication with action_node
    pub_goal_to_reach = n.advertise<geometry_msgs::Point>("goal_to_reach", 1);     // Preparing a topic to publish the goal to reach

    // communication with robot_moving_node
    sub_robot_moving = n.subscribe("robot_moving", 1, &decision_node::robot_movingCallback, this);

    // communication with localization node
    sub_localization = n.subscribe("localization", 1, &decision_node::localizationCallback, this);

    current_state = EState::waiting_for_a_person;
    previous_state = EState::undefined;

    new_person_position = false;
    state_has_changed = false;

    // Define base_position coordinates according to the chosen base / initial position in the map frame.
    base_position.x = 0;
    base_position.y = 0;
    base_orientation = 0;

    origin_position.x = 0;
    origin_position.y = 0;

    person_tracked = false;

    //INFINITE LOOP TO COLLECT LASER DATA AND PROCESS THEM
    ros::Rate r(10);// this node will work at 10hz
    while (ros::ok())
    {
        ros::spinOnce();// For each subscribed topic on which a message has been received, the corresponding callback function is called.
        update();
        r.sleep();//we wait if the processing (ie, callback+update) has taken less than 0.1s (ie, 10 hz)
    }

}

void update()
{
    // init_localization = true;
    if ( init_localization )
    {

        update_variables();

        // TO DO:
        // TO COMPLETE:
        // DO NOT FORGET that if robair is too far from its base (ie, its distance to the base is higher than max_base_distance),
        // then robair should stop to interact with the moving person and go back to its base. Where should we implement this?


        // TO DO:  OK translation_to_base ?????????
        if ( translation_to_base > max_base_distance ){
            ROS_INFO("Robair is too far from its base. Going back to the base.");
            current_state = EState::rotating_to_the_base;
        } else{
            switch ( current_state )
            {
                case EState::waiting_for_a_person:
                    process_waiting_for_a_person();
                    break;

                case EState::observing_the_person:
                    process_observing_the_person();
                    break;

                case EState::rotating_to_the_person:
                    process_rotating_to_the_person();
                    break;

                case EState::moving_to_the_person:
                    process_moving_to_the_person();
                    break;

                case EState::interacting_with_the_person:
                    process_interacting_with_the_person();
                    break;

                case EState::rotating_to_the_base:
                    process_rotating_to_the_base();
                    break;

                case EState::moving_to_the_base:
                    process_moving_to_the_base();
                    break;

                case EState::resetting_orientation:
                    process_resetting_orientation();
                    break;
            }
        }

    new_localization = false;
    new_person_position = false;

    state_has_changed = current_state != previous_state;
    previous_state = current_state;

    }
    else { 
        ROS_WARN("Initialize localization");
    }

}// end update

void update_variables()
{
    if ( new_person_position )
    {

        ROS_INFO("new_person_position = (%f, %f)",  person_position.x, person_position.y);

        translation_to_person = sqrt( ( person_position.x * person_position.x ) + ( person_position.y * person_position.y ) );

        if ( translation_to_person > 0 )
        {
        //we compute the /rotation_to_do
        rotation_to_person = acos( person_position.x / translation_to_person );
            if ( person_position.y < 0 ) {
                rotation_to_person *=-1;
            }
        }
        else {
            rotation_to_person = 0;
        }

        person_tracked = person_position.x != 0 || person_position.y != 0;
    }        

    if ( new_localization )
    {   

        ROS_INFO("new localization = %d",  new_localization);
        ROS_INFO("position of robair in the map: (%f, %f, %f)", current_position.x, current_position.y, current_orientation*180/M_PI);

        translation_to_base = distancePoints(base_position, current_position);

        int x_translated = base_position.x - current_position.x;
        int y_translated = base_position.y - current_position.y;

        rotation_to_base = atan2( y_translated, x_translated );
 
        local_base_position.x = x_translated;
        local_base_position.y = y_translated; 

        // when we receive a new position(x, y, o) of robair in the map, we update:
        // translation_to_base: the translation that robair has to do to reach its base
        // rotation_to_base: the rotation that robair has to do to reach its base
        // local_base_position: the position of the base in the cartesian local frame of robot
    }

}

void process_waiting_for_a_person()
{
    makeLog();

    // good will me pardonner pour ca
    rotated_to_the_base = false;
    moved_to_the_base = false;

    // As soon as we detect a moving person, we switch to the state "observing_the_person"
    if ( new_person_position ) {
        current_state = EState::observing_the_person;
    }

}

void process_observing_the_person()
{
    makeLog();

    if ( state_has_changed )
    {
        previous_person_position = person_position;
        frequency = 0;
    }

    // Processing of the state
    // Robair only observes and tracks the moving person
    if ( new_person_position )
    {
        frequency += 1;

    
        bool person_moved = distancePoints(person_position, previous_person_position) > detection_threshold;
        if (person_moved) {
            // update last reference to the new person position
            previous_person_position.x = person_position.x;
            previous_person_position.y = person_position.y;
            frequency = 0;
            ROS_INFO("Person Moved");
            return;
        } else if (frequency >= frequency_expected) {
            // not moving anymore
            ROS_INFO("Person Stopped");
            current_state = EState::rotating_to_the_person;
        }
    }

    
    // TO DO:
    // QUESTION! TO COMPLETE:
    // What should robair do if it loses the moving person ? 
    // (prior to this, you should think about what happens when datmo_node loses the person,
    //  in order to determine how this node can understand that the person has been lost. 
    //  Does it keep publishing? Does it publish a special value? Does it stop publishing?)

    if (person_lost) {
        ROS_INFO("Person lost");
        ROS_INFO("Changing state to waiting_for_a_person");
        current_state = EState::waiting_for_a_person;
    }

}

void process_rotating_to_the_person()
{
    makeLog();

    if ( state_has_changed )
    {
        frequency = 0;
        previous_person_position = person_position;
    }

    // Processing of the state
    // Robair rotates to be facing towards the moving person
    double distance = distancePoints(person_position, previous_person_position);
    bool person_moved = distance > detection_threshold;
    pub_rotation_to_do.publish(person_position);
    
    ROS_INFO("Rotation to do : %f", rotation_to_person);

    bool rotation_done = rotation_to_person < rotation_treshold;
    
    if ( new_person_position )
    {
        if (person_moved) {
            ROS_INFO("Person has moved : (%f, %f); distance = %f", person_position.x, person_position.y, distance);

            frequency = 0;
            
            ROS_INFO("Frequency back to 0");

            pub_rotation_to_do.publish(person_position);

            previous_person_position = person_position;

            return;
        } else if (rotation_done) {
            frequency += 1;
        }

        if (frequency >= frequency_expected) {
            ROS_INFO("Changing state to moving_to_the_person()");
            current_state = EState::moving_to_the_person;
        }

    // TO COMPLETE:
    // what should robair do if it loses the moving person ? 
    } 
    else if (person_lost) {
        ROS_INFO("Person lost");
        ROS_INFO("pub_goal_to_reach to go back to the local_base_position and changing state to Waiting_for_a_person");
        pub_goal_to_reach.publish(local_base_position);
        //TO DO: change state to resetting_orientation
        current_state = EState::waiting_for_a_person;
    }
    ROS_INFO("frequency : %d", frequency);

}

void process_moving_to_the_person()
{
    makeLog();
    
    if ( state_has_changed )
    {
        // getchar();

        frequency = 0;
        previous_person_position=person_position;
    }

    // Processing of the state
    // Robair moves to be close to the moving person
    if ( new_person_position )
    {
        ROS_INFO("person_position: (%f, %f)", person_position.x, person_position.y);

        bool person_moved = distancePoints(person_position, previous_person_position) > detection_threshold;

        //TO DO : if person move stop process_moving_to_the_person and return to the base ????????
        //if(person_moved) {
            //ROS_INFO("Person moved");
            //frequency = 0;
            //current_state = EState::rotating_to_the_base;
            //return;
        //}

        //increment frequency because the person is not moving
        frequency += 1;
        frequency = std::min(frequency, frequency_expected);
        
        //move robot in the front of the person
        pub_goal_to_reach.publish(person_position);
        ROS_INFO("Pub_goal_to_reach on person_position");


        // if robair is close to the moving person and the moving person does not move during a while (use frequency), we switch to the state "interacting_with_the_person"
        if (distancePoints(person_position, current_position) < detection_threshold && frequency >= frequency_expected) {
            ROS_INFO("Person close to the robot");
            ROS_INFO("Changing state to interacting_with_the_person");

        
        
        // publier un point à zero pour forcer l'arret
            geometry_msgs::Point zero_point;
            zero_point.x = 0;
            zero_point.y = 0;

            pub_goal_to_reach.publish(zero_point);
            current_state = EState::interacting_with_the_person;

        // already close to the person 
        } else if (frequency >= frequency_expected) {
            current_state = EState::interacting_with_the_person;
        }

        // rotating to the base before to go back to the base
    } else if (person_lost){
        ROS_INFO("Person lost");
        ROS_INFO("Changing state to rotating_to_the_base");
        current_state = EState::rotating_to_the_base;
    }

}

void process_interacting_with_the_person()
{
    makeLog();

    if ( state_has_changed )
    {
        // getchar();
        frequency = 0;
        previous_person_position = person_position;

    }

    double distance = distancePoints(person_position, previous_person_position);

    // Processing of the state
    // Robair does not move and interacts with the moving person until the moving person goes away from robair
    if ( new_person_position )
    {


        bool person_moved = distance > detection_threshold;
        bool moved_away = (sqrt(pow(person_position.x,2)+pow(person_position.y,2))-sqrt(pow(previous_person_position.x,2)+pow(previous_person_position.y,2))) > detection_threshold;
        

        ROS_INFO("distance = %f, person_moved = %d, moved_away = %d", distance, person_moved, moved_away);

        // TO COMPLETE:
        // if the person goes away from robair, after a while (use frequency), we switch to the state "rotating_to_the_base"

        if (person_lost) {
            person_moved=true; 
            moved_away=true;
        }
        
        if (person_moved && moved_away && ++frequency >= frequency_expected){
            current_state = EState::rotating_to_the_base;
        }
        else {
            frequency = 0;
        }
    } else if (!new_person_position || distance < 1e-5) {
        if (++frequency >= frequency_expected){
            current_state = EState::rotating_to_the_base;
        }
    }
}

void makeLog() {
     switch ( current_state )
            {
                case EState::waiting_for_a_person:
                    ROS_INFO("current state = waiting for a person\n");
                    break;

                case EState::observing_the_person:
                    ROS_INFO("current state = observing the person\n");
                    break;

                case EState::rotating_to_the_person:
                    ROS_INFO("current state = rotating to the person \n");
                    break;

                case EState::moving_to_the_person:
                    ROS_INFO("current state = moving to the person \n");
                    break;

                case EState::interacting_with_the_person:
                    ROS_INFO("current state = interacting with the person \n");
                    break;

                case EState::rotating_to_the_base:
                    ROS_INFO("current state = rotating to the base \n");
                    break;

                case EState::moving_to_the_base:
                    ROS_INFO("current state = moving to the base \n");
                    break;

                case EState::resetting_orientation:
                    ROS_INFO("current state = resetting orientation \n");
                    break;
            }
    ROS_INFO("person_position: (%f, %f) \n", person_position.x, person_position.y);
    ROS_INFO("frequency: %d \n", frequency);

    ROS_INFO("new localization = %d, localization (%f, %f, %f) \n", new_localization, current_position.x, current_position.y, current_position.z);
    ROS_INFO("Local base position = (%f, %f)", local_base_position.x, local_base_position.y);

}

void process_rotating_to_the_base()
{

    makeLog();

    if ( state_has_changed )
    {
        //getchar();
        frequency = 0;
    }

    // Processing of the state
    // Robair rotates to be face to its base
   
    ROS_INFO("position of robair in the map: (%f, %f, %f)", current_position.x, current_position.y, current_orientation*180/M_PI);

        // robair should rotate to align with the base position (requires expressing base position in the robot / laser frame)

    if (!rotated_to_the_base) {
        pub_rotation_to_do.publish(local_base_position);
        rotated_to_the_base = true;
    }
        
    if(++frequency >= 25) {
        current_state = EState::moving_to_the_base;
    }
}

void process_moving_to_the_base()
{
    ROS_INFO("\n\tEnter process_moving_to_the_base()");

    makeLog();


    if ( state_has_changed )
    {
        ROS_INFO("current_state: moving_to_the_base");
        ROS_INFO("position of robair in the map: (%f, %f, %f)", current_position.x, current_position.y, current_orientation*180/M_PI);
        ROS_INFO("press enter to continue");
        //getchar();
        frequency = 0;
    }

    frequency++;

    // Processing of the state
    // Robair moves to its base
    if ( new_localization )
    {
        ROS_INFO("position of robair in the map: (%f, %f, %f)", current_position.x, current_position.y, current_orientation*180/M_PI);

         double distance = distancePoints(current_position, local_base_position);

        // already very close to the base
        if (distance < 0.2) {
            current_state = EState::resetting_orientation;
        }

        if (!moved_to_the_base) {
            pub_goal_to_reach.publish(local_base_position);
            frequency = 0;
            moved_to_the_base = true;
        } 

        if (frequency >= frequency_expected) {
            current_state = EState::resetting_orientation;
        }

        //TO COMPLETE:
        // robair should move towards the base point (requires expressing base position in robair frame)

        // TO COMPLETE:
        // if robair is close to its base and does not move, after a while (use frequency), we switch to the state "resetting_orientation"
    }

}

void process_resetting_orientation()
{
ROS_INFO("\n\tEnter process_resetting_orientation()");
    if ( state_has_changed )
    {
        ROS_INFO("current_state: initializing_rotation");
        ROS_INFO("position of robair in the map: (%f, %f, %f)", current_position.x, current_position.y, current_orientation*180/M_PI);
        ROS_INFO("press enter to continue");
        //getchar();
        frequency = 0;
    }

    // Processing of the state
    // Robair rotates to its initial orientation
    if ( new_localization )
    {
        ROS_INFO("position of robair in the map: (%f, %f, %f)", current_position.x, current_position.y, current_orientation*180/M_PI);
        //TO COMPLETE:
        // robair should rotate to face the initial orientation

        //TO COMPLETE
        // if robair is close to its initial orientation and does not move, after a while (use frequency), we switch to the state "waiting_for_a_person"
    }

}

//CALLBACKS
/*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/
void person_positionCallback(const geometry_msgs::Point::ConstPtr& g)
{
    // process the goal received from moving_persons detector

    // person lost
    if (g->x == 0 && g->y == 0) {
        person_lost = true;
        return;
    }

    person_lost = false;

    new_person_position = true;
    person_position.x = g->x;
    person_position.y = g->y;

}

void robot_movingCallback(const std_msgs::Bool::ConstPtr& state)
{

    robot_moving = state->data;

}//robot_movingCallback

void localizationCallback(const geometry_msgs::Point::ConstPtr& l)
{
// process the localization received from my localization

    ROS_INFO("LOCALIZATION CALLBACK %f %f %f", l->x, l->y, l->z);

    new_localization = true;
    init_localization = true;
    current_position = *l;
    current_orientation = l->z;

    // QUESTION!
    if (!init_localization) {
        base_position = *l;
        base_orientation = l->z;
    }

}

// Distance between two points
float distancePoints(geometry_msgs::Point pa, geometry_msgs::Point pb) {

    return sqrt(pow((pa.x-pb.x),2.0) + pow((pa.y-pb.y),2.0));

}

};

int main(int argc, char **argv)
{

    ROS_INFO("(decision_node) waiting for a /person_position");
    ros::init(argc, argv, "decision_node");

    decision_node bsObject;

    ros::spin();

    return 0;

}