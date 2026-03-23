#!/usr/bin/env bash
# Обёртка над rosservice call ...::block: по Ctrl+C публикуем /oled_mouth/stop_audio и рвём клиент.
# Сырой rosservice call часто не реагирует на Ctrl+C во время долгого ответа сервиса.
set +e
if [[ $# -lt 1 ]]; then
  echo "Использование: rosrun sound_mouth_motik_emotions oled_play_block.bash <файл.mp3>" >&2
  echo "  Лучше: rosrun sound_mouth_motik_emotions play_audio_wait.py <файл.mp3>" >&2
  exit 2
fi
FILE="$1"
ROSSVC_PID=""
cleanup() {
  rostopic pub -1 /oled_mouth/stop_audio std_msgs/Empty >/dev/null 2>&1 || true
  if [[ -n "$ROSSVC_PID" ]] && kill -0 "$ROSSVC_PID" 2>/dev/null; then
    kill -INT "$ROSSVC_PID" 2>/dev/null || true
    wait "$ROSSVC_PID" 2>/dev/null || true
  fi
  exit 130
}
trap cleanup INT
rosservice call /oled_mouth/play_audio "data: '${FILE}::block'" &
ROSSVC_PID=$!
wait "$ROSSVC_PID"
RC=$?
trap - INT
exit "$RC"
