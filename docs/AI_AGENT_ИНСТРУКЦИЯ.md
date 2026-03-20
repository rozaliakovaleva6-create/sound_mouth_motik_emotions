# Инструкция для AI ассистента / AI агента / агента-разработчика

## Контекст проекта

Это ROS-пакет `sound_mouth_motik_emotions` для управления 3D дисплеем (OLED 0x3D) робота AINEX. Пакет содержит две ноды:

1. **robot_mouth_talk_node** (mouth) — осциллограмма + воспроизведение звука
2. **emotions_display_node** (motik) — эмоции (happy/sad/neutral/cute/smoke) на дисплее

## Архитектура

### Файловая структура

```
/home/ubuntu/ros_ws/src/sound_mouth_motik_emotions/
├── CMakeLists.txt          # Сборка catkin
├── package.xml             # Зависимости: rospy, std_msgs, ainex_interfaces
├── launch/
│   ├── display_3d.launch   # ГЛАВНЫЙ launch — запускает обе ноды
│   ├── oled_mouth.launch   # Только mouth
│   └── oled_mouth02.launch # Обёртка
├── scripts/
│   ├── robot_mouth_talk_node.py   # ROS-нода mouth
│   ├── robot_mouth_talk.py        # Автономный скрипт
│   ├── sound_and_mouth_talk.py    # Имитация без звука
│   └── emotions_display_node.py   # ROS-нода motik
├── voice/                          # Аудио файлы (.mp3, .wav)
└── docs/                           # Документация
```

### ROS-интерфейс

**Топики:**
- `/oled_3d/active__driver` (String) — переключатель `"mouth"` / `"motik"`
- `/oled_mouth/audio_path` (String) — путь к файлу для воспроизведения
- `/oled_mouth/mode` (String) — `"idle"` / `"oscillogram"`
- `/audio/mouth_open_level` (Float32) — уровень открытия рта 0..1 (вход)
- `/audio/playback_level` (Float32) — уровень воспроизведения 0..1 (выход)
- `/emotions` (String) — `"happy"` / `"sad"` / `"neutral"` / `"cute"` / `"smoke"`

**Сервисы:**
- `/oled_mouth/play_audio` (ainex_interfaces/SetString) — воспроизвести файл или `"stop"`

### Связь с ainex_bringup

Пакет подключается из `ainex_bringup/launch/bringup.launch`:
```xml
<include file="$(find sound_mouth_motik_emotions)/launch/display_3d.launch"/>
```

Запускается ПОСЛЕ `base.launch` (который содержит `ainex_controller` — подъём робота).

## Правила для AI агента

### 1. НЕ трогать ainex_bringup (кроме launch-файлов)

Ноды подъёма робота (`ainex_controller`, `ainex_kinematics`) лежат в `ainex_driver`. Их НЕ модифицировать. Файл `bringup.launch` в `ainex_bringup` можно редактировать только для подключения/отключения пакетов.

### 2. Безопасность дисплея

- Дисплей 0x3D может использоваться ТОЛЬКО одним процессом
- Переключение через `/oled_3d/active__driver` — каждая нода проверяет флаг перед рисованием
- Если нода дисплея упала — робот продолжит работать
- Никогда не запускать два процесса рисования на 0x3D одновременно

### 3. Добавление новых эмоций

1. Создать функцию `draw_<имя>()` в `emotions_display_node.py`
2. Добавить в словарь `EMOTION_DRAWERS`
3. Вызывать: `rostopic pub -1 /emotions std_msgs/String "data: '<имя>'"`

### 4. Добавление аудио

1. Положить файл в `voice/`
2. Вызвать: `rosservice call /oled_mouth/play_audio "data: '<имя_файла>'"`

### 5. Сборка после изменений

```bash
cd ~/ros_ws
source /opt/ros/noetic/setup.zsh
sudo catkin build sound_mouth_motik_emotions
source devel/setup.zsh
```

### 6. Тестирование

```bash
# Проверить что ноды запущены
rosnode list | grep -E "mouth|emotions"

# Проверить сервис
rosservice list | grep oled_mouth

# Тест звука
rosservice call /oled_mouth/play_audio "data: 'adam-dragon-us_3475244.mp3'"

# Тест эмоций
rostopic pub -1 /oled_3d/active__driver std_msgs/String "data: 'motik'"
rostopic pub -1 /emotions std_msgs/String "data: 'happy'"

# Вернуть mouth
rostopic pub -1 /oled_3d/active__driver std_msgs/String "data: 'mouth'"
```

## Диагностика проблем

### Робот не встаёт

1. Проверить `bringup.launch` на merge-конфликты (`<<<<<<`, `=======`, `>>>>>>>`). Если есть — XML невалидный и roslaunch падает.
2. Проверить `systemctl status start_app_node.service`
3. Проверить `ainex_controller.py` — должен загружать `init_pose.yaml`

### Дисплей не работает

1. `sudo i2cdetect -y 1` — должен показать 0x3D
2. `rosnode list` — проверить ноды
3. `rostopic echo /oled_3d/active__driver` — проверить текущий режим
4. Убедиться что `oled_display.service` не конфликтует на 0x3D

### Нет звука

1. `aplay -l` — список устройств
2. `amixer sget Master` — громкость
3. `speaker-test -t wav -c 1` — тестовый звук
4. Проверить наличие файла в `voice/`

## Ключевые пути

| Компонент | Путь |
|-----------|------|
| Пакет дисплея | `/home/ubuntu/ros_ws/src/sound_mouth_motik_emotions/` |
| Аудио файлы | `sound_mouth_motik_emotions/voice/` |
| Бринг-ап | `/home/ubuntu/ros_ws/src/ainex_bringup/` |
| Кинематика | `/home/ubuntu/ros_ws/src/ainex_driver/ainex_kinematics/` |
| Контроллер | `ainex_driver/ainex_kinematics/scripts/ainex_controller.py` |
| Init поза | `ainex_driver/ainex_kinematics/config/init_pose.yaml` |
| Action-файлы | `/home/ubuntu/software/ainex_controller/ActionGroups/` |
| systemd сервисы | `/etc/systemd/system/start_app_node.service`, `oled_display.service` |

## Часто используемые команды

```bash
# Переключить на mouth
rostopic pub -1 /oled_3d/active__driver std_msgs/String "data: 'mouth'"

# Переключить на motik
rostopic pub -1 /oled_3d/active__driver std_msgs/String "data: 'motik'"

# Воспроизвести звук
rosservice call /oled_mouth/play_audio "data: 'adam-dragon-us_3475244.mp3'"

# Остановить звук
rosservice call /oled_mouth/play_audio "data: 'stop'"

# Эмоции
rostopic pub -1 /emotions std_msgs/String "data: 'happy'"
rostopic pub -1 /emotions std_msgs/String "data: 'sad'"
rostopic pub -1 /emotions std_msgs/String "data: 'neutral'"

# Перезапуск сервиса
sudo systemctl restart start_app_node.service

# Логи
journalctl -u start_app_node.service -f

# Сборка
cd /home/ubuntu/ros_ws && catkin_make && source devel/setup.bash
```
