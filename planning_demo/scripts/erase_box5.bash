# DELETE TEST OBSTACLES TEST CASE 1
echo TEST boxe5 erasing ...
rosservice call /gazebo/delete_model '{model_name: box5}'
sleep 2
rosservice call /move_base/clear_costmaps
