#!/bin/bash

source /environment.sh

# initialize launch file
dt-launchfile-init

# YOUR CODE BELOW THIS LINE
# ----------------------------------------------------------------------------


# NOTE: Use the variable DT_REPO_PATH to know the absolute path to your code
# NOTE: Use `dt-exec COMMAND` to run the main process (blocking process)

# launching app
# dt-exec rosrun wheels_package wheels_node.py # wheels_node.py
# dt-exec rosrun wheels_package wheels_sub_node.py # wheels_node.py
# dt-exec rosrun camera_package filtering_node.py
# dt-exec rosrun obstacle_detection_package obstacle_detection_node.py
# dt-exec rosrun obstacle_detection_package obstacle_avoider_node.py
# dt-exec rosrun wheels_package wheels_sub_node.py
dt-exec rosrun obstacle_detection_package obstacle_detection_node.py
dt-exec rosrun wheels_package wheels_node.py


# ----------------------------------------------------------------------------
# YOUR CODE ABOVE THIS LINE

# wait for app to end
dt-launchfile-join
