#include "ros/ros.h"
#include <std_msgs/String.h>
#include <string>
#include<unistd.h>

class DropCargo
{
      private:
            ros::NodeHandle n_;
            ros::Publisher drop_cargo_publisher_;

            int index_{0};
            int sendtimes_{5};

            bool iscomplete_{true};

        public:
            DropCargo(ros::NodeHandle *n);
            ~DropCargo();
            void Drop(std::string cargo_name_data);
            bool GetStatus();
};
