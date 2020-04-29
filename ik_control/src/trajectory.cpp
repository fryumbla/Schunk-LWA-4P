//descargado de github

#include <ros/ros.h>

#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>

// global list to hold the setpoints. Not very thread safe but it's fine.
std::list< geometry_msgs::Point > pointlist;

// This callback is triggered each time that a set point is published.
// The only thing it does is to copy the received 3D point to the list
// of points
// Input: a new set point (3D position)
void callback( const geometry_msgs::Point& newpoint ){
  std::cout << "New point\n" << newpoint << std::endl;
  pointlist.push_back( newpoint ); 
}

// Compute and return the Jacobian of the robot given the current joint 
// positions
// input: the input joint state
// output: the 3x3 Jacobian (position only)
void Jacobian( const sensor_msgs::JointState& jointstate, double J[3][3] ){

  for( int r=0; r<3; r++ )
    for( int c=0; c<3; c++ )
      J[r][c] = 0.0;

  double q1 = jointstate.position[0];
  double q2 = jointstate.position[1];
  double q3 = jointstate.position[2];

  
  // Fill the values of the Jacobian matrix J
  J[0][0] = 0.4869*sin( q1 )*sin( q2 )*sin( q3 ) - 0.425*cos( q2 )*sin( q1 ) - 0.1914*cos( q1 ) - 0.4869*cos( q2 )*cos( q3 )*sin( q1 );
  J[0][1] = - 0.425*cos( q1 )*sin( q2 ) - 0.4869*cos( q1 )*cos( q2 )*sin( q3 ) - 0.4869*cos( q1 )*cos( q3 )*sin( q2 );
  J[0][2] = - 0.4869*cos( q1 )*cos( q2 )*sin( q3 ) - 0.4869*cos( q1 )*cos( q3 )*sin( q2 );

  J[1][0] = 0.425*cos( q1 )*cos( q2 ) - 0.1914*sin( q1 ) + 0.4869*cos( q1 )*cos( q2 )*cos( q3 ) - 0.4869*cos( q1 )*sin( q2 )*sin( q3 );
  J[1][1] = - 0.425*sin( q1 )*sin( q2 ) - 0.4869*cos( q2 )*sin( q1 )*sin( q3 ) - 0.4869*cos( q3 )*sin( q1 )*sin( q2 );
  J[1][2] = - 0.4869*cos( q2 )*sin( q1 )*sin( q3 ) - 0.4869*cos( q3 )*sin( q1 )*sin( q2 );

  J[2][0] = 0;
  J[2][1] = - 0.4869*cos( q2 + q3 ) - 0.425*cos( q2 );
  J[2][2] = - 0.4869*cos( q2 + q3 );

}

// Inverse a 3x3 matrix
// input: A 3x3 matrix
// output: A 3x3 matrix inverse
// return the determinant inverse
double Inverse( double A[3][3], double Ainverse[3][3] ){

  double determinant = (  A[0][0]*( A[1][1]*A[2][2]-A[2][1]*A[1][2] ) -
			  A[0][1]*( A[1][0]*A[2][2]-A[1][2]*A[2][0] ) +
			  A[0][2]*( A[1][0]*A[2][1]-A[1][1]*A[2][0] ) );

  double invdet = 1.0/determinant;

  Ainverse[0][0] =  ( A[1][1]*A[2][2] - A[2][1]*A[1][2] )*invdet;
  Ainverse[0][1] = -( A[0][1]*A[2][2] - A[0][2]*A[2][1] )*invdet;
  Ainverse[0][2] =  ( A[0][1]*A[1][2] - A[0][2]*A[1][1] )*invdet;

  Ainverse[1][0] = -( A[1][0]*A[2][2] - A[1][2]*A[2][0] )*invdet;
  Ainverse[1][1] =  ( A[0][0]*A[2][2] - A[0][2]*A[2][0] )*invdet;
  Ainverse[1][2] = -( A[0][0]*A[1][2] - A[1][0]*A[0][2] )*invdet;

  Ainverse[2][0] =  ( A[1][0]*A[2][1] - A[2][0]*A[1][1] )*invdet;
  Ainverse[2][1] = -( A[0][0]*A[2][1] - A[2][0]*A[0][1] )*invdet;
  Ainverse[2][2] =  ( A[0][0]*A[1][1] - A[1][0]*A[0][1] )*invdet;

  return determinant;

}



int main( int argc, char** argv ){

  // This must be called for every node
  ros::init( argc, argv, "trajectory" );

  // Create a node handle
  ros::NodeHandle nh;


  // changed the name of the topic to the correct topic name
  std::string published_topic_name( "/joint_states" );

  // Create a publisher that will publish joint states on the topic
  // /joint_states
  ros::Publisher pub_jointstate;
  pub_jointstate = nh.advertise<sensor_msgs::JointState>( published_topic_name, 1 );

  // This is the joint state message that will be published
  // http://wiki.ros.org/sensor_msgs
  sensor_msgs::JointState jointstate;
  jointstate.name.push_back( "shoulder_pan_joint" );      // name of first joint
  jointstate.name.push_back( "shoulder_lift_joint" );  // name of second joint
  jointstate.name.push_back( "elbow_joint" );     // name of third joint

  jointstate.position.push_back( 0.0 );     // initial position of first joint
  jointstate.position.push_back( -0.7 );     // initial position of second joint
  jointstate.position.push_back( -0.7 );     // initial position of third joint

  // Create a subscriber that will receive 3D setpoints
  ros::Subscriber sub_setpoint;
  sub_setpoint = nh.subscribe( "setpoint", 1, callback );

  // To listen to the current position and orientation of the robot
  // wiki.ros.org/tf/Tutorials/Writing%20a%20tf%20listener%20%28C%2B%2B%29
  tf::TransformListener listener;
  
  // Rate (Hz) of the trajectory
  // http://wiki.ros.org/roscpp/Overview/Time
  ros::Rate rate( 100 );            // the trajectory rate
  double period = 1.0/100.0;        // the period
  double positionincrement = 0.001; // how much we move 

  bool readinitpose = true;                       // used to initialize setpoint
  tf::Point setpoint;                             // the destination setpoint

  // This is the main trajectory loop
  // At each iteration it computes and publishes a new joint positions that
  // animate the motion of the robot
  // The loop exits when CTRL-C is pressed.
  while( nh.ok() ){

    // Read the current forward kinematics of the robot
    // wiki.ros.org/tf/Tutorials/Writing%20a%20tf%20listener%20%28C%2B%2B%29
    tf::Point current;
    try{

      
      // Change the name of the reference frame and target frame.
      std::string ref_frame( "base_link" );
      std::string tgt_frame( "ee_link" );

      tf::StampedTransform transform;
      listener.lookupTransform( ref_frame, tgt_frame, ros::Time(0), transform );
      current = transform.getOrigin();

      /* This if is used to initialize the position (a small hack)*/
      if( readinitpose ){
	setpoint = current;
	readinitpose = false;
      }

    }
    catch(tf::TransformException ex)
      { std::cout << ex.what() << std::endl; }

    // If the list is not empty, 
    // Read a setpoint from the list and keep the translation
    if( !pointlist.empty() ){
      setpoint.setX( pointlist.front().x );
      setpoint.setY( pointlist.front().y );
      setpoint.setZ( pointlist.front().z );
      pointlist.pop_front();
    }
    
    // This is the translation left for the trajectory
    tf::Point error = setpoint - current;

    // If the error is small enough go to the destination
    if( positionincrement < error.length() ){

      // Determine a desired cartesian linear velocity. 
      // We will command the robot to move with this velocity
      tf::Point v = ( error / error.length() ) * positionincrement;

      double J[3][3], Ji[3][3];

      // Compute the Jacobian
      Jacobian( jointstate, J );

      // Compute the inverse Jacobian. The inverse return the 
      // value of the determinant.
      if( fabs(Inverse( J, Ji )) < 1e-09 )
	{ std::cout << "Jacobian is near singular." << std::endl; }

      // Compute the joint velocity by multiplying the (Ji v)
      double qd[3];
      qd[0] = Ji[0][0]*v[0] + Ji[0][1]*v[1] + Ji[0][2]*v[2];
      qd[1] = Ji[1][0]*v[0] + Ji[1][1]*v[1] + Ji[1][2]*v[2];
      qd[2] = Ji[2][0]*v[0] + Ji[2][1]*v[1] + Ji[2][2]*v[2];

      // increment the joint positions
      jointstate.position[0] += qd[0];
      jointstate.position[1] += qd[1];
      jointstate.position[2] += qd[2];

    }

    // publish the joint states
    jointstate.header.stamp = ros::Time::now();
    pub_jointstate.publish( jointstate );

    // 
    ros::spinOnce();

    // sleep
    rate.sleep();

  }

  return 0;

}

