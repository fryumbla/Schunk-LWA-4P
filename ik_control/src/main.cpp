extern "C" {
    #include "/home/francisco/schunk_ws/src/ik_control/remoteApi/extApi.h"
}

#include <iostream>
#include <string>


#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/JointState.h>

using namespace std;

sensor_msgs::JointState joint_state;
vector<string> measure;


ros::Publisher pub_joint_commands;
sensor_msgs::JointState JointState;

void SetJointStates(const sensor_msgs::JointState::ConstPtr &_js)
{
  static ros::Time startTime = ros::Time::now();
  {
    // for testing round trip time
    JointState.header.stamp = _js->header.stamp;

    // assign sinusoidal joint angle targets
    for (unsigned int i = 0; i < JointState.name.size(); i++)
      JointState.position[i] = 3.2* sin((ros::Time::now() - startTime).toSec());
    pub_joint_commands.publish(JointState);
  }
}

int main(int argc, char **argv) 
{
  string serverIP = "127.0.0.1";
  int serverPort = 19999;

  int clientID=simxStart((simxChar*)serverIP.c_str(),serverPort,true,true,2000,5);
  
  if (clientID==-1)
  {
      
    cout << "Servidor conectado!" << std::endl;
    
    //simxGetJointPosition(clientID,1,(simxFloat *) &vLeft, (simxInt) simx_opmode_oneshot_wait);

    //Initialize and start the node
  
    ros::init(argc, argv, "fran");
    ros::NodeHandle* nh = new ros::NodeHandle();



    joint_state.name.push_back("arm_1_joint");
    joint_state.name.push_back("arm_2_joint");
    joint_state.name.push_back("arm_3_joint");
    joint_state.name.push_back("arm_4_joint");
    joint_state.name.push_back("arm_5_joint");
    joint_state.name.push_back("arm_6_joint");

    unsigned int n = joint_state.name.size();
    cout << "Number of joints:" + n  << std::endl;

    joint_state.position.resize(n);
    std::vector<double> position_goal;
    position_goal.push_back(0.1);
    position_goal.push_back(0.1);
    position_goal.push_back(0.1);
    position_goal.push_back(0.1);
    position_goal.push_back(0.1);
    position_goal.push_back(0.1);

    //joint_state.position.push_back(position_goal);





    ros::SubscribeOptions jointStatesSo = ros::SubscribeOptions::create<sensor_msgs::JointState>("/joint_states", 1, SetJointStates,ros::VoidPtr(), nh->getCallbackQueue());
    jointStatesSo.transport_hints = ros::TransportHints().unreliable();
    ros::Subscriber subJointStates = nh->subscribe(jointStatesSo);

    cout << subJointStates.getNumPublishers() << std::endl;

    pub_joint_commands = nh->advertise<sensor_msgs::JointState>("/joint_state_publisher", 1, true);

    ros::spin();
    simxFinish(clientID); // fechando conexao com o servidor
    cout << "Conexao fechada!" << std::endl;
  }
  else
    cout << "Problemas para conectar con servidor!" << std::endl;
  return 0;

  
}
