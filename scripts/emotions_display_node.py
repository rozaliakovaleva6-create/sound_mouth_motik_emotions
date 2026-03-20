#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ROS-нода: эмоции робота (motik) на OLED 0x3D.

Топики:
  /emotions (std_msgs/String) — эмоция: "happy", "sad", "neutral"
  /oled_3d/active__driver (std_msgs/String) — переключатель: "motik" или "mouth"

Рисует только когда active_driver == "motik".
При переключении на "mouth" — очищает дисплей (отдаёт его robot_mouth_talk_node).
"""
from __future__ import annotations

import os
import sys
import threading
import time

import rospy
from std_msgs.msg import String

W, H = 128, 64

try:
    from luma.core.interface.serial import i2c
    from luma.oled.device import ssd1306
    from PIL import Image, ImageDraw
    LUMA_AVAILABLE = True
except ImportError:
    LUMA_AVAILABLE = False


def draw_neutral():
    """
    Нейтральная эмоция: без глаз; одна горизонтальная полоса по вертикальному центру экрана,
    толще прежней линии рта (раньше была линия width=2 у нижнего края).
    """
    img = Image.new("1", (W, H), 0)
    draw = ImageDraw.Draw(img)
    cy = H // 2
    x0, x1 = 24, 104
    thick = 5
    half = thick // 2
    y0 = cy - half
    y1 = y0 + thick - 1
    draw.rectangle((x0, y0, x1, y1), fill=255)
    return img


def draw_happy():
    """
    Радость: без глаз; одна дуга-улыбка по вертикальному центру экрана, толще прежней (width=2).
    Нижняя полуэллипса 0°…180° — классическая «улыбка» внутри ограничивающего прямоугольника.
    """
    img = Image.new("1", (W, H), 0)
    draw = ImageDraw.Draw(img)
    cy = H // 2
    # Было: arc (24,38)-(104,62), центр ~50 по Y. Центрируем по cy≈32, сохраняем ширину ~80, высоту ~24.
    half_w, half_h = 40, 12
    x0, y0 = W // 2 - half_w, cy - half_h
    x1, y1 = W // 2 + half_w, cy + half_h
    draw.arc((x0, y0, x1, y1), 0, 180, fill=255, width=5)
    return img


def draw_sad():
    img = Image.new("1", (W, H), 0)
    draw = ImageDraw.Draw(img)
    draw.arc((16, 14, 52, 50), 0, 360, fill=255, width=2)
    draw.pieslice((24, 28, 44, 44), 200, 340, fill=255)
    draw.arc((76, 14, 112, 50), 0, 360, fill=255, width=2)
    draw.pieslice((84, 28, 104, 44), 200, 340, fill=255)
    draw.arc((34, 48, 94, 68), 180, 360, fill=255, width=2)
    return img


EMOTION_DRAWERS = {
    "neutral": draw_neutral,
    "happy": draw_happy,
    "sad": draw_sad,
}


def _wait_for_robot_standup(timeout=30):
    """Ждём пока ainex_controller поставит робота (init_pose/init_finish=True)."""
    start = time.time()
    while not rospy.is_shutdown():
        if rospy.get_param('init_pose/init_finish', False):
            rospy.loginfo("emotions_display_node: робот встал, запускаем дисплей 3D")
            return True
        if time.time() - start > timeout:
            rospy.logwarn("emotions_display_node: таймаут ожидания подъёма робота (%ds), запускаемся без подтверждения", timeout)
            return False
        time.sleep(0.5)
    return False


class EmotionsDisplayNode:
    def __init__(self):
        rospy.init_node("emotions_display_node", anonymous=False)
        _wait_for_robot_standup()
        self.device = None
        self._active_driver = str(rospy.get_param("/oled_3d/active_driver_default", "mouth")).strip().lower()
        self._emotion = rospy.get_param("~default_emotion", "neutral")
        self._lock = threading.Lock()
        self._dirty = True

        self._init_display()

        rospy.Subscriber("/emotions", String, self._cb_emotion, queue_size=1)
        rospy.Subscriber("/oled_3d/active__driver", String, self._cb_active_driver, queue_size=1)

        rospy.on_shutdown(self._on_shutdown)
        rospy.loginfo("emotions_display_node: запущен, эмоция=%s, active_driver=%s", self._emotion, self._active_driver)

    def _init_display(self):
        if not LUMA_AVAILABLE:
            rospy.logerr("emotions_display_node: pip3 install luma.oled pillow")
            return
        try:
            serial = i2c(port=1, address=0x3D)
            self.device = ssd1306(serial, width=W, height=H)
        except Exception as e:
            rospy.logerr("emotions_display_node: дисплей 0x3D недоступен: %s", e)
            self.device = None

    def _cb_emotion(self, msg):
        val = (msg.data or "").strip().lower()
        if val in EMOTION_DRAWERS:
            with self._lock:
                if self._emotion != val:
                    self._emotion = val
                    self._dirty = True

    def _cb_active_driver(self, msg):
        val = (msg.data or "").strip().lower()
        if val:
            with self._lock:
                if self._active_driver != val:
                    self._active_driver = val
                    self._dirty = True

    def _display(self, img):
        if self.device is None:
            return
        try:
            self.device.display(img)
        except Exception as e:
            rospy.logdebug("emotions_display: %s", e)

    def _on_shutdown(self):
        pass

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            with self._lock:
                driver = self._active_driver
                emotion = self._emotion
                dirty = self._dirty
                self._dirty = False

            if driver == "motik" and dirty:
                drawer = EMOTION_DRAWERS.get(emotion, draw_neutral)
                self._display(drawer())

            rate.sleep()


def main():
    try:
        node = EmotionsDisplayNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr("emotions_display_node: %s", e)
        sys.exit(1)


if __name__ == "__main__":
    main()
