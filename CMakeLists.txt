cmake_minimum_required(VERSION 3.0.2)
project(sound_mouth_motik_emotions)

find_package(catkin REQUIRED)
catkin_package()

catkin_install_python(PROGRAMS
  scripts/robot_mouth_talk_node.py
  scripts/robot_mouth_talk.py
  scripts/sound_and_mouth_talk.py
  scripts/emotions_display_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
)

install(DIRECTORY voice/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/voice
)
