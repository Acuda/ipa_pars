# DELETE TEST OBSTACLES TEST CASE 1
echo TEST boxes 1 , 2 erasing ...
rosservice call /gazebo/delete_model '{model_name: box1}'
rosservice call /gazebo/delete_model '{model_name: box2}'
rosservice call /gazebo/delete_model '{model_name: box3}'
rosservice call /gazebo/delete_model '{model_name: box4}'
rosservice call /gazebo/delete_model '{model_name: box5}'
rosservice call /gazebo/delete_model '{model_name: box6}'
rosservice call /gazebo/delete_model '{model_name: box7}'
rosservice call /gazebo/delete_model '{model_name: box8}'
rosservice call /gazebo/delete_model '{model_name: box9}'
rosservice call /gazebo/delete_model '{model_name: box10}'
rosservice call /gazebo/delete_model '{model_name: ragdoll}'