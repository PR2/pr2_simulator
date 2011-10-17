#temporary script for setting up gazebo / ogre resourcesa
export GAZEBO_RESOURCE_PATH=`rospack find gazebo`/gazebo/share/gazebo-1.0.0:`rospack find gazebo`/gazebo/share/gazebo-1.0.0/Media/models
export GAZEBO_PLUGIN_PATH=`rospack find gazebo`/gazebo/share/gazebo-1.0.0/plugins:`rospack find gazebo_plugins`/lib:`rospack find pr2_gazebo_plugins`/lib
export OGRE_RESOURCE_PATH=`rospack find ogre`/ogre/lib/OGRE
