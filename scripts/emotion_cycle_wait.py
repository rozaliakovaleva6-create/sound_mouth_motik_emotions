#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Клиент карусели эмоций: включает /emotions_display/set_cycle_demo_sec и держит процесс активным,
пока не нажмёте Ctrl+C — тогда публикуется 0.0 (карусель выключается, на OLED neutral).

Так же удобно, как play_audio_wait.py для звука: один терминал = старт круга, Ctrl+C в нём = стоп.

Условия (как у ноды emotions_display_node):
  • Должна быть запущена emotions_display_node.
  • Карусель на дисплее только при active_driver=motik (опция --motik переключает сама).
  • Анимация «сигареты» на OLED — в emotions_display_node: сигарета неподвижна; два шлейфа дыма
    ползут змейкой вдоль волнистых путей и поочерёдно обновляются (smoke в карусели, /emotions,
    бездействие ~60 с); ~smoke_anim_enabled, ~smoke_anim_frame_sec.

Примеры:
  rosrun sound_mouth_motik_emotions emotion_cycle_wait.py 2.5
  rosrun sound_mouth_motik_emotions emotion_cycle_wait.py --motik 2.5
"""
from __future__ import annotations

import signal
import sys

import rospy
from std_msgs.msg import Float32, String


def main():
    raw = [a for a in sys.argv[1:] if not a.startswith("__")]
    switch_motik = False
    if raw and raw[0] == "--motik":
        switch_motik = True
        raw = raw[1:]
    if len(raw) != 1:
        print(
            "Использование: rosrun sound_mouth_motik_emotions emotion_cycle_wait.py [--motik] <секунды>",
            file=sys.stderr,
        )
        print(
            "  <секунды> > 0 — интервал смены эмоций; процесс не завершается, пока не Ctrl+C (стоп карусели).",
            file=sys.stderr,
        )
        sys.exit(2)
    try:
        sec = float(raw[0])
    except ValueError:
        print("emotion_cycle_wait: ожидается число секунд", file=sys.stderr)
        sys.exit(2)
    if sec <= 0.0:
        print("emotion_cycle_wait: интервал должен быть > 0", file=sys.stderr)
        sys.exit(2)

    rospy.init_node("emotion_cycle_wait_cli", anonymous=True, disable_signals=True)

    pub_cycle = rospy.Publisher(
        "/emotions_display/set_cycle_demo_sec", Float32, queue_size=2, latch=False
    )
    pub_driver = rospy.Publisher(
        "/oled_3d/active__driver", String, queue_size=1, latch=False
    )
    rospy.sleep(0.3)

    if switch_motik:
        try:
            pub_driver.publish(String(data="motik"))
        except Exception:
            pass
        rospy.sleep(0.05)

    def publish_off():
        try:
            pub_cycle.publish(Float32(data=0.0))
        except Exception:
            pass

    def on_sigint(_signum, _frame):
        publish_off()
        print("\nemotion_cycle_wait: Ctrl+C — карусель выключена (set_cycle_demo_sec=0)", file=sys.stderr)
        rospy.signal_shutdown("sigint")

    signal.signal(signal.SIGINT, on_sigint)

    try:
        pub_cycle.publish(Float32(data=sec))
    except Exception as e:
        rospy.logerr("emotion_cycle_wait: не удалось включить карусель: %s", e)
        sys.exit(1)

    print(
        "Карусель включена (интервал %.2f с). Ctrl+C в этом терминале — выключить."
        % sec,
        flush=True,
    )
    rospy.loginfo(
        "emotion_cycle_wait: карусель %.2f с; Ctrl+C → set_cycle_demo_sec=0", sec
    )

    try:
        rospy.spin()
    except KeyboardInterrupt:
        publish_off()
        print("emotion_cycle_wait: прервано", file=sys.stderr)
        sys.exit(130)


if __name__ == "__main__":
    main()
