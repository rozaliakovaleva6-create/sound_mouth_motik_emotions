#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ROS-нода: рот с осциллограммой. Аудио приходит из топика или сервиса.

Топики:
  /oled_mouth/audio_path (std_msgs/String) — путь к файлу или имя — воспроизвести и показать осцилограмму
  /oled_mouth/stop_audio (std_msgs/Empty) — стоп воспроизведения и очереди (как play_audio stop); из любого терминала,
    в отличие от Ctrl+C (см. ниже).
  /oled_mouth/mode      (std_msgs/String) — режим: "idle" (статичный рот), "oscillogram" (по умолчанию при воспроизведении)
  /audio/mouth_open_level (std_msgs/Float32) — уровень открытия рта 0..1 от третьей ноды.
  /audio/playback_level (std_msgs/Float32) — публикуется нодой рта во время воспроизведения: уровень 0..1 из воспроизводимого файла (сигнал на карту); третья нода использует его вместо микрофона.

Сервисы:
  /oled_mouth/play_audio (ainex_interfaces/SetString) — data = путь или имя файла — воспроизвести
    data = stop|halt|q|quit — остановить очередь и текущее воспроизведение
    суффикс ::block у имени файла — ответ сервиса только после окончания этого трека (или стопа), см. play_audio_wait.py

Ctrl+C (SIGINT) в процесс этой ноды: 1-й раз — стоп звука и очереди (как play_audio stop);
  2-й раз в течение ~2 с — выход ноды. Работает и под roslaunch (без отдельного rosrun).
  Важно: Ctrl+C в терминале, где только rosservice call, процессу ноды не доставляется (SIGINT идёт в foreground
  shell). Стоп оттуда: rosservice … stop, или одна публикация: rostopic pub -1 /oled_mouth/stop_audio std_msgs/Empty

Вывод пикселей на дисплей 0x3D отключён в комментариях — дисплей использует motik (топик emotions).

Параметры:
  ~output_device (str) — ALSA-устройство для воспроизведения (Pygame + aplay). По умолчанию "default".
    Как узнать: aplay -l — список устройств воспроизведения; для вывода указывать, например, plughw:CARD,0.
    Ввод (микрофон): arecord -l; в цепочке нод test используется plughw:2,0 для захвата.

Осциллограмма на 0x3D во время воспроизведения: уровень из **того же файла** (мгновенная амплитуда + RMS), синхронизация по **длине файла** (не только pygame.get_busy). Три столбца **одной высоты** в кадре, модуляция синусом по времени (без разных фаз по колонкам — иначе «рот» визуально смещается). /audio/playback_level — огибающая 0..1. Параметры: ~osc_sin_freq_hz, ~mouth_smooth_playback.
"""
from __future__ import annotations

import atexit
import math
import os
import signal
import subprocess
import sys
import threading
import time

os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
os.environ.setdefault("SDL_AUDIODRIVER", "alsa")
try:
    import pygame
    pygame.init()
except Exception as e:
    print("Ошибка pygame:", e, file=sys.stderr)

import rospy
from std_msgs.msg import String, Float32, Empty
from ainex_interfaces.srv import SetString, SetStringResponse

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
VOICE_DIR = os.path.normpath(os.path.join(SCRIPT_DIR, "..", "voice"))
MUSIC_DIR = "/home/ubuntu/Music"

NORMAL_MOUTH_OPEN = 0.08
IMITATE_ONLY_DURATION_SEC = 10.0
W, H = 128, 64
FPS = 60
# Общая геометрия рта (idle / осциллограф / mode7) — один центр и размеры, без скачка при старте звука.
MOUTH_CX = 64
MOUTH_CY = 42
MOUTH_BAR_W = 18
MOUTH_BAR_H_IDLE = 6
MOUTH_BAR_LEFT0 = MOUTH_CX - 45
MOUTH_BAR_DX = 36
# Макс. высота полоски с центром в MOUTH_CY, полностью в кадре 0..H-1 (иначе низ обрезается — «рот вниз и меньше»).
MOUTH_BAR_H_MAX = 2 * min(MOUTH_CY, H - MOUTH_CY)
MOUTH_SMOOTH = 0.76
# Во время воспроизведения — быстрее реагировать на огибающую файла (меньше = резче)
MOUTH_SMOOTH_PLAYBACK = 0.52
# Синусоидальная модуляция высот столбцов (Гц)
OSC_SIN_FREQ_HZ = 3.5
# Таймаут «свежести» уровня из /audio/mouth_open_level (сек); при превышении рисуем статичный рот
MOUTH_SYNC_TIMEOUT_SEC = 0.5
# Частота обновления рта (Гц) — должна совпадать с частотой публикации третьей ноды (/mouth_sync_hz или ~rate)
DEFAULT_MOUTH_UPDATE_HZ = 30.0


def _oled_3d_i2c_address():
    """Глобальный параметр /oled_3d/i2c_address — 7-битный адрес SSD1306 (по умолчанию 0x3D=61)."""
    v = rospy.get_param("/oled_3d/i2c_address", 0x3D)
    if isinstance(v, str):
        return int(v.strip(), 0)
    return int(v)


try:
    from luma.core.interface.serial import i2c
    from luma.oled.device import ssd1306
    from PIL import Image, ImageDraw
    LUMA_AVAILABLE = True
except ImportError:
    LUMA_AVAILABLE = False

try:
    import numpy as np
    HAS_NUMPY = True
except ImportError:
    np = None  # type: ignore
    HAS_NUMPY = False

try:
    import soundfile as sf  # type: ignore

    HAS_SOUNDFILE = True
except ImportError:
    HAS_SOUNDFILE = False

# RMS по файлу не требует scipy (раньше scipy тянули в один try → без scipy весь RMS отключался).
HAS_RMS = HAS_NUMPY


def _get_sdk_audio_dir():
    try:
        from ainex_sdk import voice_play
        return voice_play.get_audio_dir()
    except Exception:
        return None


def resolve_audio_path(path_or_name):
    s = (path_or_name or "").strip()
    if not s:
        return None
    p = os.path.expanduser(s)
    if os.path.isfile(p):
        return p
    if not os.path.isabs(p) and "/" not in s and "\\" not in s:
        # Основной сценарий: вводим ТОЛЬКО имя файла из ainex_bringup/voice.
        # Расширение можно не указывать.
        voice_dir = VOICE_DIR
        full = os.path.join(os.path.expanduser(voice_dir), s)
        if os.path.isfile(full):
            return full
        if not (s.lower().endswith(".mp3") or s.lower().endswith(".wav")):
            for ext in (".mp3", ".wav"):
                f2 = full + ext
                if os.path.isfile(f2):
                    return f2
        # Фолбэк: если приложение продолжает вызывать системные ключи из SDK.
        sdk_dir = _get_sdk_audio_dir() or ""
        if sdk_dir:
            full2 = os.path.join(os.path.expanduser(sdk_dir), s)
            if os.path.isfile(full2):
                return full2
            if not (s.lower().endswith(".mp3") or s.lower().endswith(".wav")):
                for ext in (".mp3", ".wav"):
                    f2 = full2 + ext
                    if os.path.isfile(f2):
                        return f2
        try:
            from ainex_sdk import voice_play
            for lang in ("English", "Chinese"):
                vp_path = voice_play.get_path(s, lang)
                if os.path.isfile(vp_path):
                    return vp_path
        except Exception:
            pass
    return None


def _init_mixer(output_device=None):
    """Инициализация Pygame mixer. output_device задаётся через AUDIODEV до первого вызова."""
    if pygame.mixer.get_init():
        return True
    if output_device and output_device != "default":
        os.environ["AUDIODEV"] = output_device
    for channels, buf in [(2, 1024), (1, 512)]:
        try:
            pygame.mixer.init(frequency=44100, size=-16, channels=channels, buffer=buf)
            return True
        except pygame.error:
            pass
        time.sleep(0.3)
    try:
        pygame.mixer.init()
        return True
    except pygame.error:
        return False


def _play_via_alsa(path, output_device="default"):
    """Воспроизведение через aplay/mpv/ffplay. output_device — ALSA-устройство (например plughw:2,0)."""
    path = os.path.abspath(path)
    if not os.path.isfile(path):
        return None
    env = os.environ.copy()
    if output_device and output_device != "default":
        env["AUDIODEV"] = output_device
    try:
        if path.lower().endswith(".wav"):
            cmd = ["aplay", "-q"]
            if output_device and output_device != "default":
                cmd.extend(["-D", output_device])
            cmd.append(path)
            return subprocess.Popen(
                cmd,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                env=env,
            )
        for cmd in (
            ["mpv", "--no-video", "--really-quiet", path],
            ["ffplay", "-nodisp", "-autoexit", "-loglevel", "quiet", path],
        ):
            try:
                return subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, env=env)
            except FileNotFoundError:
                continue
    except Exception:
        pass
    return None


def _load_wav_stdlib(path):
    """Моно float32 [-1,1] из .wav без soundfile (только PCM 8/16/32 bit)."""
    import struct
    import wave

    with wave.open(path, "rb") as w:
        nch = w.getnchannels()
        sw = w.getsampwidth()
        sr = w.getframerate()
        nframes = w.getnframes()
        raw = w.readframes(nframes)
    if not raw or sr <= 0:
        return None, 0, 0.0
    if sw == 1:
        arr_u8 = np.frombuffer(raw, dtype=np.uint8)
        samples = (arr_u8.astype(np.float32) - 128.0) / 128.0
    elif sw == 2:
        n = len(raw) // 2
        samples = np.array(struct.unpack("<" + "h" * n, raw), dtype=np.float32) / 32768.0
    elif sw == 4:
        n = len(raw) // 4
        samples = np.array(struct.unpack("<" + "i" * n, raw), dtype=np.float32) / 2147483648.0
    else:
        return None, 0, 0.0
    if nch > 1:
        samples = samples.reshape(-1, nch).mean(axis=1)
    return samples.astype(np.float32), sr, len(samples) / float(sr)


def _load_audio_pydub(path):
    """Любой формат, который понимает pydub (часто mp3 через ffmpeg)."""
    from pydub import AudioSegment

    ext = os.path.splitext(path)[1].lower()
    if ext == ".mp3":
        audio = AudioSegment.from_mp3(path)
    elif ext == ".wav":
        audio = AudioSegment.from_wav(path)
    else:
        audio = AudioSegment.from_file(path)
    audio = audio.set_channels(1)
    sr = int(audio.frame_rate)
    # Нормализация амплитуды по разрядности сэмпла
    sw = audio.sample_width
    max_val = float(2 ** (8 * sw - 1))
    samples = np.array(audio.get_array_of_samples(), dtype=np.float32) / max_val
    return samples, sr, len(samples) / float(sr)


def _load_audio_ffmpeg(path, target_sr=44100):
    """
    Декодирование через ffmpeg в моно float32 (как у pygame mixer по умолчанию).
    Работает для mp3/wav/ogg и т.д., если установлен ffmpeg.
    """
    if not HAS_NUMPY:
        return None, 0, 0.0
    try:
        sz = os.path.getsize(path)
        timeout = min(600, max(60, sz // 20000 + 30))
        cmd = [
            "ffmpeg",
            "-nostdin",
            "-hide_banner",
            "-loglevel",
            "error",
            "-i",
            path,
            "-f",
            "f32le",
            "-ac",
            "1",
            "-ar",
            str(target_sr),
            "pipe:1",
        ]
        p = subprocess.run(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            timeout=timeout,
        )
        if p.returncode != 0 or not p.stdout:
            return None, 0, 0.0
        samples = np.frombuffer(p.stdout, dtype=np.float32).copy()
        if samples.size == 0:
            return None, 0, 0.0
        dur = samples.size / float(target_sr)
        return samples, target_sr, dur
    except (FileNotFoundError, subprocess.TimeoutExpired, Exception):
        return None, 0, 0.0


def load_wav_for_rms(path):
    """
    Загрузка сэмплов для осциллограммы по **содержимому файла** (не захват с микрофона).
    Цепочка: soundfile → stdlib wave → pydub → ffmpeg.
    """
    if not HAS_NUMPY or not path or not os.path.isfile(path):
        return None, 0, 0.0

    # 1) soundfile (wav/flac/ogg и т.д.)
    if HAS_SOUNDFILE:
        try:
            data, sr = sf.read(path)
            if data.ndim > 1:
                data = data.mean(axis=1)
            data = data.astype(np.float32)
            return data, int(sr), len(data) / float(sr)
        except Exception:
            pass

    # 2) WAV через stdlib (без soundfile)
    if path.lower().endswith(".wav"):
        try:
            return _load_wav_stdlib(path)
        except Exception:
            pass

    # 3) pydub (mp3 и др., нужен ffmpeg для mp3)
    try:
        return _load_audio_pydub(path)
    except Exception:
        pass

    # 4) ffmpeg напрямую
    return _load_audio_ffmpeg(path, target_sr=44100)


def _resample_linear(samples, sr_in, sr_out):
    """Линейная пересборка под частоту pygame mixer (по умолчанию 44100)."""
    if sr_in == sr_out or samples is None or len(samples) < 2:
        return samples, sr_in
    dur = len(samples) / float(sr_in)
    t_out_n = max(2, int(dur * sr_out))
    t_in = np.linspace(0.0, dur, num=len(samples), endpoint=False)
    t_out = np.linspace(0.0, dur, num=t_out_n, endpoint=False)
    out = np.interp(t_out, t_in, samples).astype(np.float32)
    return out, sr_out


def get_rms_from_samples(wav_samples, wav_sr, pos_ms, window_ms=40):
    """RMS громкости в окне для анимации рта. Используется сырой RMS, чтобы рот реагировал на любой звук (речь, музыка)."""
    if wav_samples is None or wav_sr <= 0 or not HAS_NUMPY:
        return 0.0
    center = int(pos_ms / 1000.0 * wav_sr)
    half = int(window_ms / 2000.0 * wav_sr)
    start = max(0, center - half)
    end = min(len(wav_samples), center + half)
    if end <= start:
        return 0.0
    chunk = wav_samples[start:end]
    rms = float(np.sqrt(np.mean(chunk ** 2)))
    return rms


def _playback_level_from_samples(wav_samples, wav_sr, pos_ms, peak_abs, rms_ref, window_ms=48):
    """
    Уровень 0..1 для осциллограммы: мгновенная амплитуда + RMS в окне,
    нормированные на пик и средний RMS по всему файлу.
    """
    if wav_samples is None or len(wav_samples) == 0 or wav_sr <= 0 or not HAS_NUMPY:
        return 0.0
    pos_ms = max(0.0, float(pos_ms))
    center = int(pos_ms / 1000.0 * wav_sr)
    center = max(0, min(len(wav_samples) - 1, center))
    instant = abs(float(wav_samples[center])) / peak_abs
    rms = get_rms_from_samples(wav_samples, wav_sr, pos_ms, window_ms=window_ms)
    slow = min(1.0, (rms / rms_ref) * 1.8)
    level = 0.5 * instant + 0.5 * slow
    return max(0.0, min(1.0, level))


def draw_oscilloscope_frame(level, t_sec, sin_freq_hz=None):
    """
    Три столбца: огибающая из файла, синусоидальная модуляция по времени.
    Высота столбцов в одном кадре **общая** (одна фаза синуса на кадр): иначе у колонок разные
    top = cy - h//2 и рот «разъезжается» по вертикали — на OLED кажется, что положение и размер
    прыгнули относительно спокойного трёхполосного рта.
    Полоски симметрично расширяются от MOUTH_CY; высота ≤ MOUTH_BAR_H_MAX (без обрезки снизу).
    """
    if sin_freq_hz is None:
        sin_freq_hz = OSC_SIN_FREQ_HZ
    img = Image.new("1", (W, H), 0)
    draw = ImageDraw.Draw(img)
    cy = MOUTH_CY
    max_h = MOUTH_BAR_H_MAX
    base = max(0.0, min(1.0, level))
    idle_bar_h = MOUTH_BAR_H_IDLE
    # Одна модуляция на весь «рот» — как в покое, три одинаковых полоски в один ряд.
    phase = 2.0 * math.pi * sin_freq_hz * t_sec
    wobble = 0.32 + 0.68 * (0.5 + 0.5 * math.sin(phase))
    h = max(idle_bar_h, int(max_h * base * wobble))
    top = cy - h // 2
    for i in range(3):
        left = MOUTH_BAR_LEFT0 + i * MOUTH_BAR_DX
        draw.rectangle((left, top, left + MOUTH_BAR_W, top + h), fill=255)
    return img


def draw_mouth_mode7(mouth_open):
    import random
    img = Image.new("1", (W, H), 0)
    draw = ImageDraw.Draw(img)
    cy = MOUTH_CY
    cap = min(42, MOUTH_BAR_H_MAX)
    h = max(MOUTH_BAR_H_IDLE, int(cap * mouth_open) if mouth_open > 0.02 else 0)
    for i in range(3):
        th = max(MOUTH_BAR_H_IDLE, h + (random.randint(-9, 9) if mouth_open > 0.1 else 0))
        th = min(th, MOUTH_BAR_H_MAX)
        left = MOUTH_BAR_LEFT0 + i * MOUTH_BAR_DX
        top = cy - th // 2
        draw.rectangle((left, top, left + MOUTH_BAR_W, top + th), fill=255)
    return img


def draw_idle_mouth():
    img = Image.new("1", (W, H), 0)
    draw = ImageDraw.Draw(img)
    cy = MOUTH_CY
    bar_height, bar_width = MOUTH_BAR_H_IDLE, MOUTH_BAR_W
    for i in range(3):
        left = MOUTH_BAR_LEFT0 + i * MOUTH_BAR_DX
        top = cy - bar_height // 2
        draw.rectangle((left, top, left + bar_width, top + bar_height), fill=255)
    return img


def _wait_for_robot_standup(timeout=30):
    """Ждём пока ainex_controller поставит робота (init_pose/init_finish=True)."""
    start = time.time()
    while not rospy.is_shutdown():
        if rospy.get_param('init_pose/init_finish', False):
            rospy.loginfo("robot_mouth_talk_node: робот встал, запускаем дисплей 3D")
            return True
        if time.time() - start > timeout:
            rospy.logwarn("robot_mouth_talk_node: таймаут ожидания подъёма робота (%ds), запускаемся без подтверждения", timeout)
            return False
        time.sleep(0.5)
    return False


class RobotMouthTalkNode:
    def __init__(self):
        rospy.init_node("robot_mouth_talk_node", anonymous=False)
        _wait_for_robot_standup()
        self.device = None
        self.play_queue = []  # элементы: (path: str, service_blocks: bool)
        self.lock = threading.Lock()
        self._srv_block_done = threading.Event()
        self._srv_block_waiting = False
        self._srv_block_interrupted = False
        self.mode = rospy.get_param("~mode", "oscillogram")
        # Если true — рисуем анимацию/рот на OLED I2C 0x3D.
        # Важно: дисплеем может владеть только один процесс одновременно.
        self._use_oled = bool(rospy.get_param("~use_oled", True))
        self._active_driver = str(rospy.get_param("/oled_3d/active_driver_default", "mouth")).strip().lower()
        self._stop_requested = threading.Event()
        self._current_play_proc = None
        self._current_use_subprocess = False
        # Устройство вывода звука (ALSA). Как узнать: aplay -l — воспроизведение, arecord -l — ввод
        self._output_device = (rospy.get_param("~output_device", "default") or "default").strip()
        if self._output_device != "default":
            os.environ["AUDIODEV"] = self._output_device
        # Уровень от третьей ноды (/audio/mouth_open_level) — осциллограмма или микрофон
        self._mouth_sync_level = 0.0
        self._mouth_sync_time = 0.0
        self._mouth_sync_lock = threading.Lock()
        self._sync_timeout = float(rospy.get_param("~mouth_sync_timeout_sec", MOUTH_SYNC_TIMEOUT_SEC))
        # Частота обновления дисплея (Гц): ~rate или общий /mouth_sync_hz — должна совпадать с третьей нодой
        self._mouth_update_hz = float(rospy.get_param("~rate", rospy.get_param("/mouth_sync_hz", DEFAULT_MOUTH_UPDATE_HZ)))
        self._osc_sin_freq = float(rospy.get_param("~osc_sin_freq_hz", OSC_SIN_FREQ_HZ))
        self._mouth_smooth_playback = float(
            rospy.get_param("~mouth_smooth_playback", MOUTH_SMOOTH_PLAYBACK)
        )
        rospy.Subscriber("/oled_3d/active__driver", String, self._cb_active_driver, queue_size=1)
        self._init_display()

        rospy.Subscriber("/oled_mouth/audio_path", String, self._cb_audio_path, queue_size=1)
        rospy.Subscriber("/oled_mouth/stop_audio", Empty, self._cb_stop_audio, queue_size=3)
        rospy.Subscriber("/oled_mouth/mode", String, self._cb_mode, queue_size=1)
        rospy.Subscriber("/audio/mouth_open_level", Float32, self._cb_mouth_open_level, queue_size=5)
        self._pub_playback_level = rospy.Publisher("/audio/playback_level", Float32, queue_size=5)
        self._pub_user_activity = rospy.Publisher("/oled_3d/user_activity", Empty, queue_size=10, latch=False)
        self._pub_active_driver = rospy.Publisher("/oled_3d/active__driver", String, queue_size=1, latch=False)
        self.srv = rospy.Service("/oled_mouth/play_audio", SetString, self._srv_play_audio)

        rospy.on_shutdown(self._shutdown_display)  # при остановке ноды — финальный кадр, экран не гаснет
        atexit.register(self._shutdown_display)   # дублируем на случай выхода без rospy (kill, исключение)
        rospy.loginfo(
            "robot_mouth_talk_node: /oled_mouth/play_audio, /oled_mouth/stop_audio, звук: %s, обновление: %.1f Гц",
            self._output_device,
            self._mouth_update_hz,
        )
        self._install_sigint_stop_audio()

    def _install_sigint_stop_audio(self):
        """1-й SIGINT — stop_audio_playback(); 2-й за ~2 с — rospy.signal_shutdown. Всегда (в т.ч. roslaunch)."""
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
                    self.stop_audio_playback()
                except Exception as ex:
                    rospy.logwarn("robot_mouth_talk_node: Ctrl+C stop: %s", ex)
                rospy.loginfo(
                    "robot_mouth_talk_node: Ctrl+C (SIGINT) — воспроизведение остановлено "
                    "(как play_audio stop). Повторите Ctrl+C в течение ~2 с для выхода из ноды."
                )
                schedule_reset()
            else:
                rospy.signal_shutdown("sigint")

        signal.signal(signal.SIGINT, handler)
        if sys.stdin.isatty():
            rospy.loginfo("robot_mouth_talk_node: SIGINT — стоп звука / повторный — выход (интерактивный терминал).")
        else:
            rospy.loginfo(
                "robot_mouth_talk_node: SIGINT — стоп звука / повторный — выход (в т.ч. roslaunch: Ctrl+C в окне launch)."
            )

    def stop_audio_playback(self):
        """Очистить очередь и остановить текущий трек (аналог сервиса data='stop')."""
        with self.lock:
            self.play_queue[:] = []
        if getattr(self, "_srv_block_waiting", False):
            self._srv_block_interrupted = True
        try:
            self._srv_block_done.set()
        except Exception:
            pass
        self._stop_requested.set()
        try:
            if pygame.mixer.get_init():
                pygame.mixer.music.stop()
        except Exception:
            pass
        if self._current_use_subprocess and self._current_play_proc is not None:
            try:
                if self._current_play_proc.poll() is None:
                    self._current_play_proc.terminate()
            except Exception:
                pass
        self._ping_user_activity()

    def _ping_user_activity(self):
        """Сброс таймера бездействия (эмоции → smoke) для ноды emotions_display_node."""
        try:
            self._pub_user_activity.publish(Empty())
        except Exception:
            pass

    def _request_mouth_driver_for_playback(self):
        """
        После idle-smoke OLED у motik: переключаем на mouth до пинга активности, чтобы одной
        командой play выйти из smoke и сразу отдать дисплей рту (осциллограмма + звук).
        """
        try:
            self._pub_active_driver.publish(String(data="mouth"))
        except Exception:
            pass

    def _cb_active_driver(self, msg):
        val = (msg.data or "").strip().lower()
        if val:
            self._active_driver = val

    def _init_display(self):
        self.device = None
        if not self._use_oled:
            return
        if not LUMA_AVAILABLE:
            rospy.logerr("robot_mouth_talk_node: для вывода на OLED установите: pip3 install luma.oled pillow")
            return
        addr = _oled_3d_i2c_address()
        try:
            serial = i2c(port=1, address=addr)
            self.device = ssd1306(serial, width=W, height=H)
            try:
                if self._active_driver == "mouth":
                    self.device.display(draw_idle_mouth())
            except Exception:
                pass
            rospy.loginfo("robot_mouth_talk_node: OLED 3D I2C адрес %#x", addr)
        except Exception as e:
            rospy.logerr("robot_mouth_talk_node: дисплей I2C %#x недоступен: %s", addr, e)
            self.device = None

    def _cb_audio_path(self, msg):
        path = (msg.data or "").strip()
        if path:
            self._request_mouth_driver_for_playback()
            with self.lock:
                self.play_queue.append((path, False))
            self._ping_user_activity()

    def _cb_stop_audio(self, _msg):
        """Топик — стоп из любого терминала (аналог сервиса play_audio stop)."""
        self.stop_audio_playback()
        rospy.loginfo("robot_mouth_talk_node: /oled_mouth/stop_audio — воспроизведение остановлено")

    def _cb_mode(self, msg):
        m = (msg.data or "").strip().lower()
        if m in ("idle", "oscillogram"):
            self.mode = m
            self._ping_user_activity()

    def _cb_mouth_open_level(self, msg):
        """Уровень открытия рта от цепочки нод (третья нода): осциллограмма с динамика/микрофона."""
        val = max(0.0, min(1.0, float(msg.data)))
        with self._mouth_sync_lock:
            self._mouth_sync_level = val
            self._mouth_sync_time = time.time()
        rospy.loginfo_throttle(5.0, "robot_mouth_talk_node: получаем /audio/mouth_open_level (осциллограмма)")

    def _get_sync_mouth_level(self):
        """Возвращает (level, valid). valid=False если топик давно не обновлялся."""
        with self._mouth_sync_lock:
            level = self._mouth_sync_level
            t = self._mouth_sync_time
        return level, (time.time() - t) <= self._sync_timeout

    def _srv_play_audio(self, req):
        data = (req.data or "").strip()
        if data.lower() in ("stop", "halt", "q", "quit"):
            self.stop_audio_playback()
            return SetStringResponse(success=True, message="stopped")

        if not data:
            self._ping_user_activity()
            return SetStringResponse(success=True, message="empty")

        block = False
        path = data
        low = data.lower()
        if low.endswith("::block"):
            block = True
            path = data[: -len("::block")].rstrip()
            if not path:
                return SetStringResponse(success=False, message="empty_path_with_block")

        self._request_mouth_driver_for_playback()
        if block:
            self._srv_block_interrupted = False
            self._srv_block_done.clear()
            self._srv_block_waiting = True
        with self.lock:
            self.play_queue.append((path, block))
        self._ping_user_activity()

        if block:
            try:
                while not rospy.is_shutdown():
                    if self._srv_block_done.wait(0.1):
                        break
            finally:
                self._srv_block_waiting = False
                self._srv_block_done.clear()
            msg = "interrupted" if self._srv_block_interrupted else "finished"
            return SetStringResponse(success=True, message=msg)

        return SetStringResponse(success=True, message="queued")

    def _display(self, img):
        if self.device is None:
            return
        if not self._use_oled or self._active_driver != "mouth":
            return
        try:
            self.device.display(img)
        except Exception as e:
            rospy.logdebug("display: %s", e)

    def _shutdown_display(self):
        """При остановке ноды рисуем финальный кадр — экран не гаснет и не уходит в случайное состояние."""
        if self.device is not None:
            try:
                self._display(draw_idle_mouth())
                time.sleep(0.15)  # даём I2C передаче завершиться до выхода процесса
                self._display(draw_idle_mouth())
            except Exception as e:
                rospy.logdebug("shutdown_display: %s", e)

    def _play_and_animate(self, audio_path):
        # Сбрасываем сигнал stop перед новым треком
        self._stop_requested.clear()

        resolved = resolve_audio_path(audio_path)
        if not resolved:
            rospy.logwarn("Аудио не найдено: %s", audio_path)
            return
        try:
            rospy.set_param("/oled_3d/mouth_playback_active", True)
            wav_samples, wav_sr, duration_sec = load_wav_for_rms(resolved)
            # Pygame mixer инициализируется на 44100 — выравниваем огибающую по времени воспроизведения
            mixer_sr = 44100
            peak_abs, rms_ref = 1.0, 1.0
            if wav_samples is not None and wav_sr > 0:
                wav_samples, wav_sr = _resample_linear(wav_samples, wav_sr, mixer_sr)
                duration_sec = len(wav_samples) / float(wav_sr)
                peak_abs = float(np.max(np.abs(wav_samples))) + 1e-9
                rms_ref = float(np.sqrt(np.mean(np.asarray(wav_samples, dtype=np.float64) ** 2))) + 1e-9
            elif not HAS_NUMPY:
                rospy.logwarn(
                    "Осциллограмма по файлу недоступна: установите numpy (pip3 install numpy)"
                )
            else:
                rospy.logwarn(
                    "Не удалось загрузить сэмплы для RMS (%s). Осциллограмма будет глухой. "
                    "Установите: pip3 install soundfile pydub && sudo apt install ffmpeg",
                    os.path.basename(resolved),
                )

            play_proc = None
            audio_ok = False
            use_subprocess = False
            self._current_play_proc = None
            self._current_use_subprocess = False

            if _init_mixer(self._output_device):
                try:
                    pygame.mixer.music.load(resolved)
                    pygame.mixer.music.play()
                    audio_ok = True
                except pygame.error as e:
                    rospy.logwarn("pygame: %s", e)

            if not audio_ok:
                play_proc = _play_via_alsa(resolved, self._output_device)
                if play_proc and play_proc.poll() is None:
                    use_subprocess, audio_ok = True, True
                    self._current_play_proc = play_proc
                    self._current_use_subprocess = use_subprocess

            if not audio_ok:
                rospy.logwarn("Звук не запущен. Имитация %s с.", IMITATE_ONLY_DURATION_SEC)

            # Старт с «спокойного рта» — совпадает с draw_idle_mouth / NORMAL_MOUTH_OPEN, без скачка первого кадра
            mouth_open = float(NORMAL_MOUTH_OPEN)
            start_time = time.time()
            duration_ms = int(duration_sec * 1000) if duration_sec > 0 else 0
            min_anim_sec = 0.3
            has_waveform = (
                wav_samples is not None
                and wav_sr > 0
                and duration_sec > 0
                and len(wav_samples) > 0
            )
            # Хвост после длительности файла (мс → сек): дорисовываем кадр, пока буфер ALSA догрызает
            timeline_tail_sec = 0.4

            while rospy.is_shutdown() is False:
                if self._stop_requested.is_set():
                    break
                elapsed = time.time() - start_time
                elapsed_ms = int(elapsed * 1000)

                if use_subprocess and play_proc:
                    is_playing_audio = play_proc.poll() is None and (
                        duration_ms == 0 or elapsed_ms < duration_ms
                    )
                else:
                    is_playing_audio = (
                        pygame.mixer.music.get_busy() if pygame.mixer.get_init() else False
                    )

                # Критично: на части железа get_busy() почти сразу False, хотя звук идёт —
                # тогда огибающая не обновлялась. Если есть декодированный файл — крутим осциллограмму
                # по таймкоду файла, а не только по get_busy().
                if has_waveform:
                    in_timeline = elapsed < (duration_sec + timeline_tail_sec)
                    show_scope = in_timeline
                else:
                    show_scope = is_playing_audio

                target = 0.0
                if show_scope and has_waveform:
                    pos_ms = min(duration_sec * 1000.0, max(0.0, elapsed * 1000.0))
                    raw_level = _playback_level_from_samples(
                        wav_samples, wav_sr, pos_ms, peak_abs, rms_ref
                    )
                    self._pub_playback_level.publish(Float32(data=raw_level))
                    target = raw_level
                elif show_scope:
                    target = NORMAL_MOUTH_OPEN
                    self._pub_playback_level.publish(Float32(data=target))

                smooth = (
                    self._mouth_smooth_playback
                    if (show_scope and has_waveform)
                    else MOUTH_SMOOTH
                )
                mouth_open += (target - mouth_open) * (1.0 - smooth)
                mouth_open = max(0.0, min(1.0, mouth_open))

                if show_scope and has_waveform:
                    frame = draw_oscilloscope_frame(
                        mouth_open, elapsed, sin_freq_hz=self._osc_sin_freq
                    )
                elif show_scope:
                    frame = draw_mouth_mode7(mouth_open)
                else:
                    frame = draw_idle_mouth()

                self._display(frame)
                time.sleep(1.0 / FPS)

                # Выход: при наличии волны — строго по длине файла (+хвост)
                if has_waveform and elapsed >= duration_sec + timeline_tail_sec:
                    break
                if not has_waveform:
                    if use_subprocess and play_proc:
                        if elapsed >= min_anim_sec and (
                            play_proc.poll() is not None
                            or (duration_ms > 0 and elapsed_ms >= duration_ms)
                        ):
                            break
                    elif not is_playing_audio and audio_ok and elapsed > max(1.0, min_anim_sec):
                        break
                    elif not audio_ok and elapsed >= IMITATE_ONLY_DURATION_SEC:
                        break

            if use_subprocess and play_proc and play_proc.poll() is None:
                play_proc.terminate()
            if pygame.mixer.get_init() and audio_ok and not use_subprocess:
                try:
                    pygame.mixer.music.stop()
                except Exception:
                    pass

            self._current_play_proc = None
            self._current_use_subprocess = False
        finally:
            try:
                rospy.set_param("/oled_3d/mouth_playback_active", False)
            except Exception:
                pass

        # После остановки/окончания — возврат к статичному рту обработает run()

    def run(self):
        self._display(draw_idle_mouth())
        # Первый кадр на OLED — сброс таймера бездействия в emotions_display_node
        self._ping_user_activity()
        # В rospy нет spin_once(); колбэки обрабатываем в отдельном потоке
        spin_thread = threading.Thread(target=rospy.spin, daemon=True)
        spin_thread.start()
        rate = rospy.Rate(self._mouth_update_hz)
        while not rospy.is_shutdown():
            item = None
            with self.lock:
                if self.play_queue:
                    item = self.play_queue.pop(0)
            if item:
                if isinstance(item, tuple):
                    path, block_svc = item[0], item[1]
                else:
                    path, block_svc = item, False
                try:
                    self._play_and_animate(path)
                finally:
                    if block_svc:
                        try:
                            self._srv_block_done.set()
                        except Exception:
                            pass
                self._display(draw_idle_mouth())
            else:
                if self.mode == "idle":
                    self._display(draw_idle_mouth())
                else:
                    # Режим oscillogram: рот по уровню от третьей ноды (осциллограмма с динамика/микрофона)
                    level, valid = self._get_sync_mouth_level()
                    if valid:
                        self._display(draw_mouth_mode7(level))
                    else:
                        rospy.loginfo_throttle(5.0, "robot_mouth_talk_node: нет данных с /audio/mouth_open_level — запустите сначала три ноды (roslaunch test audio_oscillogram_full.launch)")
                        # Тот же кадр, что после трека и при старте — без расхождения с осциллограммой в нуле
                        self._display(draw_idle_mouth())
            rate.sleep()


def main():
    try:
        node = RobotMouthTalkNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr("robot_mouth_talk_node: %s", e)
        sys.exit(1)


if __name__ == "__main__":
    main()
