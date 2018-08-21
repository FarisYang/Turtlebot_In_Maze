import os
#os.system('gnome-terminal -e "nav"')
os.system('gnome-terminal -e "rostopic pub /move_base_simple/goal geometry_msgs/PoseStamped \'{header: {stamp: now, frame_id: \"map\"}, pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.1, w: 0.1}}}\'"')
           