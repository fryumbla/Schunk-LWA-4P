#include "ros/ros.h"
#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit/planning_scene_interface/planning_scene_interface.h"
#include "geometric_shapes/shape_operations.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "building_workspace");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();
 

    const std::string PLANNING_GROUP = "both_arms";
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    move_group.setPlanningTime(45.0);

    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.resize(3);

    // Table
    collision_objects[0].id = "obj_table";
    collision_objects[0].header.frame_id = "base_link";
    /* Define the primitive and its dimensions. */
    collision_objects[0].primitives.resize(1);
    collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX;
    collision_objects[0].primitives[0].dimensions.resize(3);
    collision_objects[0].primitives[0].dimensions[0] = 0.730000257492;
    collision_objects[0].primitives[0].dimensions[1] = 0.750000059605;
    collision_objects[0].primitives[0].dimensions[2] = 1.20000016689;
    /* Define the pose of the table. */
    collision_objects[0].primitive_poses.resize(1);
    collision_objects[0].primitive_poses[0].position.x = 0.57500565052;
    collision_objects[0].primitive_poses[0].position.y = 0.0250088647008;
    collision_objects[0].primitive_poses[0].position.z = 0.36501121521;
    collision_objects[0].primitive_poses[0].orientation.x = -0.499993562698;
    collision_objects[0].primitive_poses[0].orientation.y = 0.500006496906;
    collision_objects[0].primitive_poses[0].orientation.z = -0.500006496906;
    collision_objects[0].primitive_poses[0].orientation.w = -0.499993562698;

    collision_objects[0].operation = collision_objects[0].ADD;

    // Juice
    collision_objects[1].id = "obj_juice";
    collision_objects[1].header.frame_id = "base_link";
    /* Define the primitive and its dimensions. */
    collision_objects[1].primitives.resize(1);
    collision_objects[1].primitives[0].type = collision_objects[0].primitives[0].CYLINDER;
    collision_objects[1].primitives[0].dimensions.resize(3);
    collision_objects[1].primitives[0].dimensions[0] = 0.143729999661;
    collision_objects[1].primitives[0].dimensions[1] = 0.0250000055134;
    // collision_objects[1].primitives[0].dimensions[2] = 0.143729999661;
    /* Define the pose of the table. */
    collision_objects[1].primitive_poses.resize(1);
    collision_objects[1].primitive_poses[0].position.x = 0.290748232603;
    collision_objects[1].primitive_poses[0].position.y = 0.0;
    collision_objects[1].primitive_poses[0].position.z = 0.801787078381;

    collision_objects[1].operation = collision_objects[1].ADD;

    // Milk
    collision_objects[2].id = "obj_milk";
    collision_objects[2].header.frame_id = "base_link";
    /* Define the primitive and its dimensions. */
    collision_objects[2].primitives.resize(1);
    collision_objects[2].primitives[0].type = collision_objects[0].primitives[0].BOX;
    collision_objects[2].primitives[0].dimensions.resize(3);
    collision_objects[2].primitives[0].dimensions[0] = 0.0450000055134;
    collision_objects[2].primitives[0].dimensions[1] = 0.0450000055134;
    collision_objects[2].primitives[0].dimensions[2] = 0.156530082226;
    /* Define the pose of the table. */
    collision_objects[2].primitive_poses.resize(1);
    collision_objects[2].primitive_poses[0].position.x = 0.438998311758;
    collision_objects[2].primitive_poses[0].position.y = 0.29629996419;
    collision_objects[2].primitive_poses[0].position.z = 0.808184683323;

    collision_objects[2].operation = collision_objects[2].ADD;

    ROS_INFO("anadiendo");
    // for(int i=0;i<=3;i++)
    // {
    //     move_group.attachObject(collision_objects[i].id);
    // }

    planning_scene_interface.applyCollisionObjects(collision_objects);

    ros::waitForShutdown();
    return 0;
}