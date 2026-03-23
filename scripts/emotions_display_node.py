#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ROS-нода: эмоции робота (motik) на OLED 0x3D.

Топики:
  /emotions (std_msgs/String) — эмоция: "happy", "sad", "neutral", "cute", "cat", "smoke"
  /oled_3d/active__driver (std_msgs/String) — переключатель: "motik" или "mouth"

Рисует только когда active_driver == "motik".
При переключении на "mouth" — очищает дисплей (отдаёт его robot_mouth_talk_node).

Параметры:
  ~cycle_emotions_demo_sec (float, по умолчанию 0) — стартовый интервал карусели (сек); 0 = выкл.
  ~smoke_anim_enabled (bool, по умолчанию true) — анимация дыма у эмоции smoke (карусель, /emotions, бездействие).
  ~smoke_anim_frame_sec (float, по умолчанию 0.18) — шаг анимации дыма (сек): две «змейки» ползут по путям.
  ~cat_anim_enabled (bool, по умолчанию true) — чередование кадров «cat» (усы чуть вниз).
  ~cat_anim_frame_sec (float, по умолчанию 0.45) — длительность одного кадра cat (сек); цикл 2 кадра.

Топик (без перезапуска ноды):
  /emotions_display/set_cycle_demo_sec (std_msgs/Float32) — задать интервал карусели в рантайме.
    data <= 0 — выключить карусель, на OLED сразу neutral, дальше только /emotions и переключение mouth/motik.
    data > 0 — включить карусель с этим интервалом (с начала списка с neutral).

  Удобный клиент с Ctrl+C в том же терминале (процесс «держит» карусель, пока не остановите):
    rosrun sound_mouth_motik_emotions emotion_cycle_wait.py [--motik] <секунды>

  В интерактивном терминале (прямой rosrun этой ноды): первый Ctrl+C — выключить карусель (как data<=0),
  если она была включена; второй Ctrl+C подряд (~2 с) — выход из ноды. При roslaunch без TTY поведение не меняется.

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
import signal
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


def draw_cat_animated(mono: float, frame_sec: float):
    """
    Кошачья мордочка: нос (перевёрнутый треугольник) + 6 усов.
    Кадр 0 — старт (усы «нейтральнее»); кадр 1 — конец: кончики усов ниже (иллюзия шевеления).
    Один нос (треугольник сверху), без отдельного «рта» под ним. Цикл 0↔1.
    """
    img = Image.new("1", (W, H), 0)
    draw = ImageDraw.Draw(img)
    lw = 2
    fs = max(frame_sec, 0.05)
    phase = int(mono / fs) % 2

    # Нос: скруглённый перевёрнутый треугольник (широкий верх, острие вниз)
    draw.polygon([(59, 29), (69, 29), (64, 37)], fill=255, outline=255)

    # Базы усов у щёк (фиксированы)
    left_base = [(51, 31), (50, 34), (51, 37)]
    right_base = [(77, 31), (78, 34), (77, 37)]

    if phase == 0:
        # Стартовый кадр: усы чуть «вверх / в сторону»
        left_tip = [(22, 25), (18, 34), (22, 41)]
        right_tip = [(106, 25), (110, 34), (106, 41)]
    else:
        # Конечный кадр: только усы опущены вниз (без второго треугольника/рта под носом)
        left_tip = [(24, 29), (19, 40), (24, 47)]
        right_tip = [(104, 29), (109, 40), (104, 47)]

    for b, t in zip(left_base, left_tip):
        draw.line([b, t], fill=255, width=lw)
    for b, t in zip(right_base, right_tip):
        draw.line([b, t], fill=255, width=lw)

    return img


def draw_cat():
    """Первый кадр эмоции cat (без тика времени)."""
    return draw_cat_animated(0.0, 0.45)


# --- Дым: две волнистые «змейки» (длинная + короткая), ползут вдоль пути и по очереди обновляются.
# Сигарета рисуется ОДИН раз в фиксированных координатах (капсула с тлеющим концом).
SMOKE_LONG_PATH = [
    (27, 48),
    (24, 44),
    (26, 40),
    (23, 36),
    (26, 32),
    (22, 28),
    (25, 24),
    (21, 20),
    (24, 16),
    (20, 12),
    (23, 8),
    (19, 5),
    (22, 2),
]
SMOKE_SHORT_PATH = [
    (31, 47),
    (33, 42),
    (30, 38),
    (32, 33),
    (29, 29),
    (31, 25),
    (28, 21),
    (30, 17),
]
SMOKE_LONG_VISIBLE = 7
SMOKE_SHORT_VISIBLE = 5
SMOKE_SNAKE_PAUSE_STEPS = 3
# Сдвиг фазы второго шлейфа (тиков): пока одна змейка «уходит», другая как бы подхватывает
SMOKE_SHORT_TICK_STAGGER = 7


def _draw_cigarette_static(draw, lw=2):
    """
    Неподвижная сигарета: капсула (толстый штрих с круглыми концами в Pillow) + тлеющий край + черта у границы.
    Координаты зафиксированы — не зависят от кадра дыма.
    """
    # Тело — наклон ~18° вверх вправо (левый конец ниже = тлеющий)
    draw.line([(28, 52), (90, 41)], fill=255, width=5)
    draw.ellipse((23, 48, 33, 55), outline=255, width=lw)
    draw.line([(31, 49), (31, 53)], fill=255, width=1)


def _smoke_snake_segment(path, tick, L_visible, pause_steps):
    """
    Один шаг анимации для одной полилинии: нарастание от кончика → скольжение окна L_visible вверх → пауза.
    Возвращает список точек для draw.line или None в фазе паузы.
    """
    n = len(path)
    if n < 2:
        return None
    L = min(L_visible, n)
    grow_steps = max(0, L - 2) + 1 if L >= 2 else 1
    slide_max = max(1, n - L + 1)
    cycle = grow_steps + slide_max + pause_steps
    step = tick % cycle
    if step < grow_steps:
        k = min(L, 2 + step)
        if k < 2:
            return None
        return path[0:k]
    if step < grow_steps + slide_max:
        s0 = step - grow_steps
        return path[s0 : s0 + L]
    return None


def draw_smoke_animated(mono: float, frame_sec: float):
    """
    Сигарета неподвижна. Два шлейфа дыма (длинный S-образный и короткий) — каждый ползёт «змейкой»
    вдоль своего пути (скользящее окно вершин), затем пауза и снова нарастание с кончика.
    Второй шлейф со сдвигом фазы, чтобы линии визуально поочерёдно сменяли друг друга.
    Эмоция smoke и бездействие (idle → smoke) используют одну и ту же отрисовку.
    """
    img = Image.new("1", (W, H), 0)
    draw = ImageDraw.Draw(img)
    lw = 2
    fs = max(frame_sec, 0.05)
    tick = int(mono / fs)

    _draw_cigarette_static(draw, lw=lw)
    if tick % 3 != 1:
        draw.point((25, 51), fill=255)
        draw.point((27, 52), fill=255)

    seg_long = _smoke_snake_segment(
        SMOKE_LONG_PATH, tick, SMOKE_LONG_VISIBLE, SMOKE_SNAKE_PAUSE_STEPS
    )
    if seg_long is not None and len(seg_long) >= 2:
        draw.line(seg_long, fill=255, width=lw)

    tick_short = tick + SMOKE_SHORT_TICK_STAGGER
    seg_short = _smoke_snake_segment(
        SMOKE_SHORT_PATH, tick_short, SMOKE_SHORT_VISIBLE, SMOKE_SNAKE_PAUSE_STEPS
    )
    if seg_short is not None and len(seg_short) >= 2:
        draw.line(seg_short, fill=255, width=lw)

    return img


def draw_smoke():
    """Статичный момент цикла (начало змеек у кончика) для совместимости."""
    return draw_smoke_animated(0.0, 0.18)


EMOTION_DRAWERS = {
    "neutral": draw_neutral,
    "happy": draw_happy,
    "sad": draw_sad,
    "cute": draw_cute,
    "cat": draw_cat,
    "smoke": draw_smoke,
}

# Порядок показа при демо-карусели и для вывода списка в лог
EMOTION_NAMES_ORDER = tuple(
    k for k in ("neutral", "happy", "sad", "cute", "cat", "smoke") if k in EMOTION_DRAWERS
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


def _oled_3d_i2c_address():
    """Параметр /oled_3d/i2c_address — 7-битный адрес OLED «3D» (по умолчанию 0x3D=61)."""
    v = rospy.get_param("/oled_3d/i2c_address", 0x3D)
    if isinstance(v, str):
        return int(v.strip(), 0)
    return int(v)


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
        self._smoke_anim_enabled = bool(rospy.get_param("~smoke_anim_enabled", True))
        self._smoke_frame_sec = float(rospy.get_param("~smoke_anim_frame_sec", 0.18))
        if self._smoke_frame_sec < 0.05:
            self._smoke_frame_sec = 0.05
        self._cat_anim_enabled = bool(rospy.get_param("~cat_anim_enabled", True))
        self._cat_frame_sec = float(rospy.get_param("~cat_anim_frame_sec", 0.45))
        if self._cat_frame_sec < 0.08:
            self._cat_frame_sec = 0.08
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
        self._install_tty_sigint_disable_carousel()

    def _disable_carousel_ctrl_c(self):
        """Выключить карусель как при set_cycle_demo_sec <= 0 (только если была включена)."""
        with self._lock:
            if self._cycle_demo_sec <= 0.0:
                return False
            self._cycle_demo_sec = 0.0
            self._emotion = "neutral"
            self._dirty = True
        rospy.loginfo("emotions_display_node: карусель выключена (Ctrl+C) → neutral на OLED")
        self._register_user_activity(restore_motik_neutral=False)
        self._emit_user_activity()
        return True

    def _install_tty_sigint_disable_carousel(self):
        """В TTY: 1-й Ctrl+C — стоп карусели; 2-й — выход (без TTY не вмешиваемся в SIGINT)."""
        if not sys.stdin.isatty():
            return
        self._sigint_count = 0
        self._sigint_timer = None
        self._sigint_lock = threading.Lock()
        reset_delay = 2.0

        def reset_count():
            with self._sigint_lock:
                self._sigint_count = 0

        def schedule_reset():
            t = getattr(self, "_sigint_timer", None)
            if t is not None:
                try:
                    t.cancel()
                except Exception:
                    pass
            self._sigint_timer = threading.Timer(reset_delay, reset_count)
            self._sigint_timer.daemon = True
            self._sigint_timer.start()

        def handler(signum, frame):
            with self._sigint_lock:
                self._sigint_count += 1
                n = self._sigint_count
            if n == 1:
                try:
                    self._disable_carousel_ctrl_c()
                except Exception as ex:
                    rospy.logwarn("emotions_display_node: Ctrl+C: %s", ex)
                rospy.loginfo(
                    "emotions_display_node: Ctrl+C — карусель сброшена (если была включена). "
                    "Повторите Ctrl+C для выхода из ноды."
                )
                schedule_reset()
            else:
                rospy.signal_shutdown("sigint")

        signal.signal(signal.SIGINT, handler)

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
        addr = _oled_3d_i2c_address()
        try:
            serial = i2c(port=1, address=addr)
            self.device = ssd1306(serial, width=W, height=H)
            rospy.loginfo("emotions_display_node: OLED 3D I2C адрес %#x", addr)
        except Exception as e:
            rospy.logerr("emotions_display_node: дисплей %#x недоступен: %s", addr, e)
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

            # Дым: непрерывная анимация (эмоция smoke, в т.ч. карусель и idle-smoke)
            smoke_anim = (
                driver == "motik"
                and emotion == "smoke"
                and self._smoke_anim_enabled
            )
            cat_anim = (
                driver == "motik"
                and emotion == "cat"
                and self._cat_anim_enabled
            )
            if smoke_anim:
                self._display(draw_smoke_animated(mono, self._smoke_frame_sec))
            elif cat_anim:
                self._display(draw_cat_animated(mono, self._cat_frame_sec))
            elif driver == "motik" and dirty:
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
