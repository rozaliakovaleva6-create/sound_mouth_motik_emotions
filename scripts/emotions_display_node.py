#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ROS-нода: эмоции робота (motik) на OLED 0x3D.

Топики:
  /emotions (std_msgs/String) — эмоция: "happy", "sad", "neutral", "cute", "smoke"
  /oled_3d/active__driver (std_msgs/String) — переключатель: "motik" или "mouth"

Рисует только когда active_driver == "motik".
При переключении на "mouth" — очищает дисплей (отдаёт его robot_mouth_talk_node).

Параметры:
  ~cycle_emotions_demo_sec (float, по умолчанию 0) — стартовый интервал карусели (сек); 0 = выкл.

Топик (без перезапуска ноды):
  /emotions_display/set_cycle_demo_sec (std_msgs/Float32) — задать интервал карусели в рантайме.
    data <= 0 — выключить карусель, на OLED сразу neutral, дальше только /emotions и переключение mouth/motik.
    data > 0 — включить карусель с этим интервалом (с начала списка с neutral).

Бездействие (~idle_timeout_sec, по умолчанию 60 с):
  Пока нет «действия» и нет воспроизведения звука (см. /oled_3d/mouth_playback_active), на OLED
  принудительно motik + эмоция smoke. Таймер сбрасывается от активности.
  Выход из smoke: явная смена /oled_3d/active__driver — стартовый экран выбранного драйвера
  (mouth → рот/осциллограмма, motik → neutral). Пинг от рта без смены драйвера — возврат как
  до простоя. Команда /emotions или карусель во время smoke — сразу выбранная эмоция/карусель,
  без принудительного neutral.

~idle_boot_grace_sec (по умолчанию 60 с): первые N секунд после завершения инициализации ноды
  принудительный smoke от бездействия не включается (осциллограмма не перехватывается). Дальше
  действует только idle_timeout_sec. Задать 0 — без задержки (как при отсутствии этого параметра).

Важно: переключение mouth <-> motik на дисплее работает, только если запущены ОБЕ ноды
(robot_mouth_talk_node и emotions_display_node). При launch_mouth:=false рот не рисует — экран
может оставаться пустым после переключения на mouth.
"""
from __future__ import annotations

import atexit
import os
import sys
import threading
import time

import rospy
from std_msgs.msg import String, Float32, Empty

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
    """
    Грусть: без глаз; одна дуга «хмурость» по вертикальному центру, толще прежней (width=2).
    Верхняя полуэллипса 180°…360° — дуга вниз (противоположность улыбке).
    Тот же bbox, что у draw_happy(), для единого масштаба эмоций.
    """
    img = Image.new("1", (W, H), 0)
    draw = ImageDraw.Draw(img)
    cy = H // 2
    half_w, half_h = 40, 12
    x0, y0 = W // 2 - half_w, cy - half_h
    x1, y1 = W // 2 + half_w, cy + half_h
    draw.arc((x0, y0, x1, y1), 180, 360, fill=255, width=5)
    return img


def draw_cute():
    """
    «Милый» рот в стиле ~uwu: форма буквы W из двух вогнутых вниз участков с общим центральным пиком.
    По центру экрана, заметная толщина линии для OLED.
    """
    img = Image.new("1", (W, H), 0)
    draw = ImageDraw.Draw(img)
    cy = H // 2
    # Полилиния с плавными изгибами (левый конец → левый минимум → к центру → пик → … → правый конец)
    pts = [
        (30, cy + 2),
        (38, cy + 6),
        (46, cy + 4),
        (54, cy - 2),
        (64, cy - 7),
        (74, cy - 2),
        (82, cy + 4),
        (90, cy + 6),
        (98, cy + 2),
    ]
    kw = {"fill": 255, "width": 4}
    try:
        draw.line(pts, joint="curve", **kw)
    except (TypeError, ValueError):
        draw.line(pts, **kw)
    return img


def draw_smoke():
    """
    Стилизованная сигарета с дымом: овал на «типе», две параллельные линии корпуса, волнистый дым вверх.
    """
    img = Image.new("1", (W, H), 0)
    draw = ImageDraw.Draw(img)
    lw = 2
    # Кончик / тлеющий край (слева внизу)
    draw.ellipse((16, 46, 28, 56), outline=255, width=lw)
    # Корпус — две параллельные линии под углом вправо-вверх
    draw.line((26, 50, 92, 42), fill=255, width=lw)
    draw.line((26, 54, 94, 46), fill=255, width=lw)
    # Короткая «окантовка» у фильтра
    draw.line((88, 40, 88, 48), fill=255, width=lw)
    # Дым — две волнистые линии от кончика
    smoke1 = [(24, 46), (18, 38), (22, 30), (16, 22), (20, 14), (14, 8), (18, 4)]
    smoke2 = [(28, 44), (32, 34), (26, 26), (30, 18), (24, 10), (28, 5)]
    draw.line(smoke1, fill=255, width=lw)
    draw.line(smoke2, fill=255, width=lw)
    return img


EMOTION_DRAWERS = {
    "neutral": draw_neutral,
    "happy": draw_happy,
    "sad": draw_sad,
    "cute": draw_cute,
    "smoke": draw_smoke,
}

# Порядок показа при демо-карусели и для вывода списка в лог
EMOTION_NAMES_ORDER = tuple(
    k for k in ("neutral", "happy", "sad", "cute", "smoke") if k in EMOTION_DRAWERS
)
if not EMOTION_NAMES_ORDER:
    EMOTION_NAMES_ORDER = tuple(sorted(EMOTION_DRAWERS.keys()))


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

        # Демо: все эмоции по очереди на OLED (секунды на кадр); 0 = выключено
        self._cycle_demo_sec = float(rospy.get_param("~cycle_emotions_demo_sec", 0.0))
        self._emotion_cycle = list(EMOTION_NAMES_ORDER)
        self._cycle_index = 0
        self._last_cycle_time = time.time()

        # Таймер бездействия → smoke; восстановление mouth / motik+neutral
        self._idle_timeout_sec = float(rospy.get_param("~idle_timeout_sec", 60.0))
        self._idle_boot_grace_sec = float(rospy.get_param("~idle_boot_grace_sec", 60.0))
        self._idle_smoke_enabled = bool(rospy.get_param("~idle_smoke_enabled", True))
        self._in_idle_smoke = False
        self._restore_driver_after_idle = "motik"
        self._driver_change_internal = False
        self._pub_user_activity = rospy.Publisher("/oled_3d/user_activity", Empty, queue_size=2, latch=False)
        self._pub_active_driver = rospy.Publisher("/oled_3d/active__driver", String, queue_size=1, latch=False)

        self._init_display()

        rospy.Subscriber("/oled_3d/user_activity", Empty, self._cb_user_activity, queue_size=10)
        rospy.Subscriber("/emotions", String, self._cb_emotion, queue_size=1)
        rospy.Subscriber("/oled_3d/active__driver", String, self._cb_active_driver, queue_size=1)
        rospy.Subscriber(
            "/emotions_display/set_cycle_demo_sec",
            Float32,
            self._cb_set_cycle_demo_sec,
            queue_size=1,
        )

        # Отсчёт «после включения»: от конца init (после ожидания standup и подписок)
        t0 = time.time()
        self._boot_mono = time.monotonic()
        self._last_activity = t0

        rospy.on_shutdown(self._on_shutdown)
        atexit.register(self._shutdown_display_neutral)

        # Вывод всех эмоций по очереди в лог (нумерованный список)
        rospy.loginfo("emotions_display_node: зарегистрированные эмоции (порядок карусели):")
        for i, name in enumerate(self._emotion_cycle, start=1):
            rospy.loginfo("  %d. %s", i, name)

        if self._cycle_demo_sec > 0.0 and self._emotion_cycle:
            self._emotion = self._emotion_cycle[0]
            self._cycle_index = 0
            self._dirty = True
            rospy.logwarn(
                "emotions_display_node: карусель ВКЛ — смена каждые %.1f с: %s",
                self._cycle_demo_sec,
                " → ".join(self._emotion_cycle),
            )
        rospy.loginfo("emotions_display_node: запущен, эмоция=%s, active_driver=%s", self._emotion, self._active_driver)
        if self._idle_smoke_enabled and self._idle_timeout_sec > 0:
            rospy.loginfo(
                "emotions_display_node: бездействие %.0f с → smoke (не раньше %.0f с после старта ноды); активность → восстановление",
                self._idle_timeout_sec,
                self._idle_boot_grace_sec,
            )

    def _emit_user_activity(self):
        """Сообщить всем (и mouth), что было действие пользователя."""
        try:
            self._pub_user_activity.publish(Empty())
        except Exception:
            pass

    def _register_user_activity(self, restore_motik_neutral=True, driver_override=None):
        """
        Обновить таймер; если был smoke от бездействия — выйти из него.
        restore_motik_neutral=False — не затирать эмоцию (уже выставлены /emotions или карусель).
        driver_override — 'mouth'|'motik': при выходе из smoke принудительно этот драйвер
        (явная команда /oled_3d/active__driver или запрос воспроизведения от рта).
        """
        if not self._idle_smoke_enabled or self._idle_timeout_sec <= 0.0:
            return
        self._last_activity = time.time()
        if self._in_idle_smoke:
            self._exit_idle_smoke(
                restore_motik_neutral=restore_motik_neutral,
                driver_override=driver_override,
            )

    def _cb_user_activity(self, _msg):
        """События от robot_mouth_talk_node (звук, очередь, режим)."""
        self._register_user_activity()

    def _exit_idle_smoke(self, restore_motik_neutral=True, driver_override=None):
        """Выйти из idle-smoke: драйвер из driver_override, иначе как до простоя; опционально neutral на motik."""
        rd = (driver_override or self._restore_driver_after_idle or "motik").strip().lower()
        if rd not in ("mouth", "motik"):
            rd = "motik"
        self._driver_change_internal = True
        try:
            self._pub_active_driver.publish(String(data=rd))
        except Exception:
            pass
        with self._lock:
            self._in_idle_smoke = False
            self._active_driver = rd
            if restore_motik_neutral and rd == "motik":
                self._emotion = "neutral"
            self._dirty = True
        rospy.loginfo("emotions_display_node: конец бездействия → '%s'", rd)

    def _enter_idle_smoke(self):
        """Принудительно motik + smoke; запомнить текущий active_driver."""
        with self._lock:
            if self._in_idle_smoke:
                return
            self._restore_driver_after_idle = self._active_driver
            self._in_idle_smoke = True
            self._active_driver = "motik"
            self._emotion = "smoke"
            self._dirty = True
        self._driver_change_internal = True
        try:
            self._pub_active_driver.publish(String(data="motik"))
        except Exception:
            pass
        rospy.loginfo(
            "emotions_display_node: бездействие %.0f с → smoke (было: %s)",
            self._idle_timeout_sec,
            self._restore_driver_after_idle,
        )

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
        if val not in EMOTION_DRAWERS:
            return
        with self._lock:
            self._emotion = val
            self._dirty = True
        self._register_user_activity(restore_motik_neutral=False)
        self._emit_user_activity()

    def _cb_active_driver(self, msg):
        val = (msg.data or "").strip().lower()
        if val not in ("mouth", "motik"):
            return
        internal = self._driver_change_internal
        self._driver_change_internal = False
        with self._lock:
            in_smoke = self._in_idle_smoke
            if self._active_driver != val:
                self._active_driver = val
                self._dirty = True
                # После mouth на OLED в памяти ноды мог остаться smoke (idle или /emotions) —
                # при явном переходе на motik показываем стартовый neutral, не сигарету.
                if val == "motik" and not internal:
                    self._emotion = "neutral"
                    self._last_cycle_time = time.time()
        if not internal:
            # Явное переключение во время smoke — драйвер из команды + стартовый экран (neutral / рот)
            self._register_user_activity(
                restore_motik_neutral=True,
                driver_override=(val if in_smoke else None),
            )

    def _cb_set_cycle_demo_sec(self, msg):
        """Выключение карусели (data<=0) → сразу neutral на дисплее; data>0 — снова карусель."""
        sec = float(msg.data)
        with self._lock:
            if sec <= 0.0:
                self._cycle_demo_sec = 0.0
                self._emotion = "neutral"
                self._dirty = True
                rospy.loginfo("emotions_display_node: карусель выключена → neutral на OLED")
            else:
                self._cycle_demo_sec = sec
                self._last_cycle_time = time.time()
                self._cycle_index = 0
                if self._emotion_cycle:
                    self._emotion = self._emotion_cycle[0]
                self._dirty = True
                rospy.loginfo("emotions_display_node: карусель %.1f с (с %s)", sec, self._emotion)
        self._register_user_activity(restore_motik_neutral=False)
        self._emit_user_activity()

    def _display(self, img):
        if self.device is None:
            return
        try:
            self.device.display(img)
        except Exception as e:
            rospy.logdebug("emotions_display: %s", e)

    def _on_shutdown(self):
        self._shutdown_display_neutral()

    def _shutdown_display_neutral(self):
        """
        При выходе (Ctrl+C / kill) не оставлять чёрный экран: если motik владеет OLED — рисуем neutral.
        Дублируем кадр + пауза для завершения I2C (как у robot_mouth_talk_node).
        """
        if self.device is None:
            return
        try:
            # Без lock: избегаем взаимной блокировки с run() при Ctrl+C
            if getattr(self, "_active_driver", "") != "motik":
                return
            img = draw_neutral()
            self._display(img)
            time.sleep(0.15)
            self._display(img)
        except Exception as e:
            rospy.logdebug("shutdown_display_neutral: %s", e)

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            now = time.time()
            mono = time.monotonic()
            boot_ok = self._idle_boot_grace_sec <= 0.0 or (
                (mono - self._boot_mono) >= self._idle_boot_grace_sec
            )
            mouth_busy = bool(rospy.get_param("/oled_3d/mouth_playback_active", False))
            if (
                self._idle_smoke_enabled
                and self._idle_timeout_sec > 0.0
                and not mouth_busy
                and boot_ok
            ):
                with self._lock:
                    should_smoke = (
                        not self._in_idle_smoke
                        and (now - self._last_activity) >= self._idle_timeout_sec
                    )
                if should_smoke:
                    self._enter_idle_smoke()

            with self._lock:
                if (
                    not self._in_idle_smoke
                    and self._cycle_demo_sec > 0.0
                    and self._emotion_cycle
                    and self._active_driver == "motik"
                    and (now - self._last_cycle_time) >= self._cycle_demo_sec
                ):
                    self._last_cycle_time = now
                    self._cycle_index = (self._cycle_index + 1) % len(self._emotion_cycle)
                    self._emotion = self._emotion_cycle[self._cycle_index]
                    self._dirty = True
                    rospy.loginfo("emotions_display_node: карусель → %s", self._emotion)

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
