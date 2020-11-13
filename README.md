## Usage
To build image, run command

`dts devel build -H <duckiebot_name>.local -f`

replace `<duckiebot_name>` with the name of your duckiebot.

To run the container, run command

`dts devel run -H <duckiebot_name>.local`

replace `<duckiebot_name>` with the name of your duckiebot.

The default map file of the package is *hud.yaml*. You can change the following line in the *default.sh* file in *launchers* folder.

`dt-exec roslaunch augmented_reality_basics augmented_reality_basics_node.launch map_file:=<map_name> veh:="$VEHICLE_NAME"`

replace `<map_name>` with hud, calibration_pattern or lane. 
