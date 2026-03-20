#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Имитация речи на OLED: рот анимируется без воспроизведения звука.

Звук не воспроизводится (нет ALSA/pygame). Рот открывается/закрывается
по синусоиде, создавая эффект говорения заданное время.

Запуск:
  python3 sound_and_mouth_talk.py

Время имитации задаётся в блоке «Настройки» ниже (IMITATE_SPEECH_DURATION_SEC)
или переменной окружения OLED_TALK_DURATION (секунды).
"""
from __future__ import annotations

import math
import os
import sys
import time

# -----------------------------------------------------------------------------
# Настройки (здесь можно выставить время имитации речи)
# -----------------------------------------------------------------------------
# Длительность имитации речи в секундах (по умолчанию 10)
# Можно переопределить: OLED_TALK_DURATION=15 python3 sound_and_mouth_talk.py
IMITATE_SPEECH_DURATION_SEC = 12.0

I2C_BUS = 1
OLED_ADDRESS = 0x3D
OLED_WIDTH, OLED_HEIGHT = 128, 64
FPS = 50
MIN_OPENNESS = 0.02
FALLBACK_OPENNESS = 0.4
# Скорость «говорения»: чем больше — тем быстрее открывается/закрывается рот
SPEECH_ANIMATION_SPEED = 10.0
# Обычный рот без действий после имитации (небольшое значение = закрытый/спокойный рот)
NORMAL_MOUTH_OPENNESS = 0.08


def get_duration_sec() -> float:
    """Длительность в секундах: OLED_TALK_DURATION или IMITATE_SPEECH_DURATION_SEC."""
    env = os.environ.get("OLED_TALK_DURATION", "").strip()
    if env:
        try:
            return max(0.1, float(env))
        except ValueError:
            pass
    return IMITATE_SPEECH_DURATION_SEC


def draw_mouth_frame(openness: float, width: int = OLED_WIDTH, height: int = OLED_HEIGHT):
    """
    Рисует один кадр рта: три вертикальных полоски, высота от openness (0..1).
    Возвращает PIL Image "1" для SSD1306.
    """
    try:
        from PIL import Image, ImageDraw
        import numpy as np
    except ImportError:
        raise ImportError("Установите Pillow и numpy: pip install Pillow numpy")
    img = Image.new("1", (width, height), 0)
    draw = ImageDraw.Draw(img)
    cx, cy = width // 2, 42
    # Рот увеличен в 3 раза
    h = max(6, int(42 * max(0, min(1, openness)))) if openness > MIN_OPENNESS else 0
    for i in range(3):
        left = cx - 45 + i * 36
        top = cy - h // 2
        draw.rectangle((left, top, left + 18, top + h), fill=255)
    return Image.fromarray((np.array(img, dtype=np.uint8)) * 255).convert("1")


def init_oled():
    """Инициализация OLED (Adafruit SSD1306)."""
    try:
        import Adafruit_SSD1306
        import numpy as np
        from PIL import Image
    except ImportError as e:
        print("Установите: pip install Adafruit-SSD1306 Pillow numpy.", e, file=sys.stderr)
        return None
    try:
        screen = Adafruit_SSD1306.SSD1306_128_64(
            rst=None, i2c_bus=I2C_BUS, gpio=1, i2c_address=OLED_ADDRESS
        )
        screen.begin()
        screen.clear()
        screen.display()
        return screen
    except Exception as e:
        print("OLED не найден (I2C 0x{:02X}):".format(OLED_ADDRESS), e, file=sys.stderr)
        return None


def main() -> None:
    duration_sec = get_duration_sec()
    print("Имитация речи {:.1f} с (время задаётся: IMITATE_SPEECH_DURATION_SEC или OLED_TALK_DURATION)".format(duration_sec))

    screen = init_oled()
    if screen is None:
        sys.exit(1)

    start_time = time.time()
    frame_interval = 1.0 / FPS

    try:
        while True:
            elapsed = time.time() - start_time
            if elapsed >= duration_sec:
                break
            # Рот то открывается, то закрывается — имитация речи
            smoothed = FALLBACK_OPENNESS + FALLBACK_OPENNESS * math.sin(elapsed * SPEECH_ANIMATION_SPEED)
            if smoothed < MIN_OPENNESS:
                smoothed = MIN_OPENNESS
            openness = min(1.0, smoothed)
            try:
                img = draw_mouth_frame(openness)
                screen.image(img)
                screen.display()
            except Exception as e:
                print("Отрисовка:", e, file=sys.stderr)
            time.sleep(frame_interval)
        # Сразу по окончании имитации — обычный рот без действий (экран не тухнет)
        if screen:
            try:
                img = draw_mouth_frame(NORMAL_MOUTH_OPENNESS)
                screen.image(img)
                screen.display()
            except Exception as e:
                print("Отрисовка рта:", e, file=sys.stderr)
    except KeyboardInterrupt:
        pass
    finally:
        # Не очищаем экран — остаётся нарисованный обычный рот
        pass
    print("Готово.")


if __name__ == "__main__":
    main()
