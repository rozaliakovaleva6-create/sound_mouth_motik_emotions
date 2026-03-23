cmake_minimum_required(VERSION 3.0.2)
project(sound_mouth_motik_emotions)

find_package(catkin REQUIRED)
catkin_package()

catkin_install_python(PROGRAMS
  scripts/robot_mouth_talk_node.py
  scripts/robot_mouth_talk.py
  scripts/sound_and_mouth_talk.py
  scripts/emotions_display_node.py
  scripts/play_audio_wait.py
  scripts/emotion_cycle_wait.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS scripts/oled_play_block.bash
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
)

install(DIRECTORY voice/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/voice
)
