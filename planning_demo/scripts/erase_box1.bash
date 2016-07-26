# DELETE TEST OBSTACLES TEST CASE 1
echo TEST box1 erasing ...
rosservice call /gazebo/delete_model '{model_name: box1}'
sleep 2
rosservice call /move_base/clear_costmaps
