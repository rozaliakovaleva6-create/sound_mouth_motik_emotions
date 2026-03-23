#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Клиент воспроизведения: вызывает /oled_mouth/play_audio с суффиксом ::block и ждёт конца трека
в том же процессе. Ctrl+C → публикация /oled_mouth/stop_audio (стоп как у сервиса stop).

Пример:
  rosrun sound_mouth_motik_emotions play_audio_wait.py Korol_i_SHut_-_Lesnik_62571704.mp3

Требуется: roscore и запущенная robot_mouth_talk_node.
"""
from __future__ import annotations

import signal
import sys

import rospy
from std_msgs.msg import Empty
from ainex_interfaces.srv import SetString


def main():
    argv = [a for a in sys.argv[1:] if not a.startswith("__")]
    if not argv:
        print(
            "Использование: rosrun sound_mouth_motik_emotions play_audio_wait.py <файл из voice/ или путь>",
            file=sys.stderr,
        )
        sys.exit(2)

    path = argv[0].strip()
    # Не вешаем внутренние обработчики rospy на SIGINT — наш обработчик шлёт stop и разблокирует ::block
    rospy.init_node("play_audio_wait_cli", anonymous=True, disable_signals=True)

    pub_stop = rospy.Publisher("/oled_mouth/stop_audio", Empty, queue_size=2, latch=False)
    rospy.sleep(0.25)

    def on_sigint(_signum, _frame):
        try:
            pub_stop.publish(Empty())
        except Exception:
            pass
        print("\nplay_audio_wait: Ctrl+C — стоп звука (/oled_mouth/stop_audio)", file=sys.stderr)

    signal.signal(signal.SIGINT, on_sigint)

    rospy.wait_for_service("/oled_mouth/play_audio", timeout=60.0)
    play = rospy.ServiceProxy("/oled_mouth/play_audio", SetString)
    low = path.lower()
    block_path = path if low.endswith("::block") else (path + "::block")

    try:
        resp = play(block_path)
        print("play_audio_wait: success=%s message=%s" % (resp.success, resp.message))
    except KeyboardInterrupt:
        try:
            pub_stop.publish(Empty())
        except Exception:
            pass
        print("play_audio_wait: прервано", file=sys.stderr)
        sys.exit(130)
    except rospy.ServiceException as e:
        rospy.logerr("play_audio_wait: %s", e)
        sys.exit(1)


if __name__ == "__main__":
    main()
