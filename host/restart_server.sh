#!/usr/bin/env bash
# עוצר מופע רץ של control_server (אם קיים) ומפעיל אותו מחדש.
# שימוש:  ./host/restart_server.sh        (מריץ ברקע ומחזיר שליטה)
#         ./host/restart_server.sh -f     (מריץ בחזית, לוג בטרמינל)
set -u

PORT=8765
HOST_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# מציאת ה-PID שמאזין על הפורט (Windows netstat) ועצירתו
PID=$(netstat -ano 2>/dev/null | grep ":${PORT}" | grep -i LISTEN | awk '{print $NF}' | head -1)
if [[ -n "${PID:-}" ]]; then
  echo "עוצר שרת קיים (PID ${PID})..."
  taskkill //PID "${PID}" //F >/dev/null 2>&1 || true
  sleep 1
else
  echo "לא נמצא שרת רץ על פורט ${PORT}."
fi

cd "${HOST_DIR}" || exit 1

if [[ "${1:-}" == "-f" ]]; then
  echo "מפעיל שרת בחזית..."
  exec python control_server.py
else
  echo "מפעיל שרת ברקע (לוג: ${HOST_DIR}/server.log)..."
  nohup python control_server.py >"${HOST_DIR}/server.log" 2>&1 &
  sleep 1
  echo "השרת פועל על http://127.0.0.1:${PORT}  (PID $!)"
fi
