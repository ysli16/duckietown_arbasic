## Usage

### Build docker image

Run command

`dts devel build -H <duckiebot_name>.local -f`

Replace `<duckiebot_name>` with the name of your duckiebot.

### Run the container

Run command

`dts devel run -H <duckiebot_name>.local`

Replace `<duckiebot_name>` with the name of your duckiebot.

### Set desired map file

The default map file of the package is *hud.yaml*. You can change the following line in the *default.sh* file in *launchers* folder.

`dt-exec roslaunch augmented_reality_basics augmented_reality_basics_node.launch map_file:=<map_name> veh:="$VEHICLE_NAME"`

replace `<map_name>` with hud, calibration_pattern or lane. 

### Check the result

Run command 

`dts start_gui_tools <duckiebot_name>`

Replace `<duckiebot_name>` with the name of your duckiebot.

Then run command

`rqt_image_view`

and select topic */<robot_name>/<node_name>/<map_name>/image/compressed*. The *<robot_name>*,*<node_name>*, and *<map_name>* are the actual name of your duckiebot, ros node and map file.
