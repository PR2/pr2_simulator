XACRO = `rospack find xacro`/xacro.py
URDF2GAZEBO = `rospack find gazebo_tools`/bin/urdf2file
MODEL_FILE = `rospack find pr2_examples_gazebo`

ROBOT = table

$(ROBOT).model: $(ROBOT).xml
	$(XACRO) $^ > $^.expanded
	$(URDF2GAZEBO) $^.expanded $(MODEL_FILE)/$@

clean:
	$(RM) $(ROBOT).xml.expanded $(MODEL_FILE)/$(ROBOT).model
	$(RM) core.* Ogre.log

