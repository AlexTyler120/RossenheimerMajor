#ifndef NAVIGATOR_H_
#define NAVIGATOR_H_

// include all necessary msg types
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseGoal.h>
#include <actionlib_msgs/GoalStatus.h>

// include relevant .h files
#include "Sensors/Sensor.h"
#include "Goal/Goal.h"
#include "Goal/GoalBased.h"
#include "Goal/GoalFire.h"
#include "Goal/GoalFlood.h"

// include standard libraries
#include <vector>
#include <array>
#include <iostream>
#include <cmath>

// enum for priority status of each incident
enum
{
    PRIORITY0 = 0,
    PRIORITY1,
    PRIORITY2,
    BASE
};

//---Navigator Implementation---------------------------------------
// The Navigator class is inherited by Controller, which utilises,
// the member functions in Navigator to achieve different objectives,
// including move to base and incidents. This class utilises functions
// to store and sort all 'Goals' found in the environment, as per their
// priority and location.

class Navigator {
    public:
        //--Constructor--
        Navigator(ros::NodeHandle& nh_);

        //--Destructor--
        ~Navigator();

        //--Functionality--

        // MoveToGoal utilises geometry_msgs to construct a target position 
        // in the map, and uses move_base, incorporating global and local 
        // planner to navigate to the target position. It takes an input of
        // a pointer to a Goal.
        void MoveToGoal(Goal* mvPoint);

        // SetGoal takes in the april id, position and orientation of TB3
        // when it recognises a tag, and dynamically stores them in private
        // vectors.
        void SetGoal(int aprilId, double x, double y, double z, double ox, double oy, double oz, double ow);
        
        // SortGoals(), utilises the vectors (_ids, _position, & _orientation), 
        // populated by SetGoal. This function sorts these vectors into the
        // _priorityBook. 
        void SortGoals();
        
        // SetBase() takes in desired inputs for the location of the Base,
        // and constructs a Based object for the location.
        void SetBase();

        // CheckPriorityBook() determines whether there are any remaining incidents
        bool CheckPriorityBook();
        
        // GetBase() returns a pointer to the Base.
        Goal* GetBase();

        // PrintBook prints the size of each priority vector from _priorityBook
        void PrintBook();

        // GetAddress() returns a vector of pointers to Goals, which are form
        // the route of the TB3 after its resupply at the Base
        std::vector<Goal*> GetAddress();

        // Algorithm, determines from the incidents remaining in _priorityBook
        // a route for the TB3 and the supplies it needs. I.e. how many fire
        // supressants and how many sandbags. It then populates the _addresses
        // with the Goals for the route, and removes them from the _priorityBook
        void Algorithm();

        // FindTag checks if a tag with a desired id already exists in _ids
        // it helps Controller prevent any duplicates being made of the 
        // Goals - from the detection of april tags. 
        bool FindTag(int tag);

    private:
        
        //--Private Members--

        ros::Publisher goal_pub;    // publisher for move_base

        // create an object of type SimpleActionClient for 
        // move_base to interact with
        actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac_; 
        
        //--Goal Information--
        
        // Vector of ids from april tags
        std::vector< int > _ids;

        // Vector of array of x, y, z position, for TB3 when each Goal is detected
        std::vector< std::array< double, 3 >> _position;

        // Vector of array of x, y, z, w orientation, for TB3 when each Goal is detected
        std::vector < std::array< double, 4 >> _orientation;
        
        // Array consisting of 4 vectors. The index of each vector represents
        // the priority of the incidents in it, with 0 being the most urgent,
        // 3 being the least urgent, and 4 being the base Goal.
        // This array represents a database in which all the fires and floods are recorded.
        // April Tag to _priorityBook guide below:

        // 0 - 50: Priority 0 Fire          - Require 3 fire extinguishers
        // 51 - 99: Priority 0 Flood        - Require 3 sandbags
        // 100 - 150: Priority 1 Fire       - Requires 2 fire extinguishers
        // 151 - 199: Priority 1 Flood      - Requires 2 sandbags
        // 200 - 250: Priority 2 Fire       - Requires 1 fire extinguisher
        // 251 - 299: Priority 2 Flood      - Requires 1 sandbag
        
        std::array< std::vector <Goal *>, (BASE + 1)> _priorityBook;

        // Vector of pointers to Goals (incidents). Populated in Algorithm(),
        // indicates the order and the route for TB3 to put out fires and block floods.
        // Pointers to goals get removed from _priorityBook & placed here, when the TB3
        // is handling the incident. Once TB3 has completed its route, this vector is 
        // cleared - for the next set of incidents.
        std::vector < Goal * > _addresses;


        //--Base Parameters

        // Position (xyz)
        double BASE_PX = -1.89;
        double BASE_PY = 0.1077;
        double BASE_PZ = 0.0001;

        // Orientation (xyzw)
        double BASE_OX = 0.0;
        double BASE_OY = 0.0;
        double BASE_OZ = 0.0;
        double BASE_OW = 1.0;

        int BASE_ARPIL = -1;
        
        int PRIORITY_INDEX_SIZE = 100;  // capacity of each priority
        int PRIORITY_DIVIDER = 50;      // halfway (between fire and flood)
};

#endif