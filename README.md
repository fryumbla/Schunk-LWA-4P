# Schunk-LWA-4P




roslaunch schunk_lwa4p test_description.launch   simula en rviz con control de joints  mediante aplicacion publica /joint_states


pack ik_control

los dos phyton son de codigo de una pagina 

rosrun ik_control comunication IK
eigen.cpp    se resuelve IK y se publica joints_states
eigen1.cpp   se resuelve IK es solo programa sin publicar nada


rosrun ik_control comunication        the acction is cpp Vrep_control.cpp   activa la comunicacion  verp con joint states

rosrun ik_control example                        publica joint_states con un juego de joints    trabaja con move_example.cpp

