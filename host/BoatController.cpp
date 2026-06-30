#include "BoatController.h"

#include <algorithm>
#include <cstdio>
#include <cstring>

#if defined(_WIN32)
  #include <windows.h>
#else
  #include <fcntl.h>
  #include <termios.h>
  #include <unistd.h>
  #include <errno.h>
  #include <sys/select.h>
#endif

BoatController::BoatController() = default;

BoatController::~BoatController() {
    close();
}

int BoatController::clampInt(int value, int lo, int hi) {
    return std::max(lo, std::min(hi, value));
}

// =====================================================================
//                          ניהול חיבור
// =====================================================================

#if defined(_WIN32)

bool BoatController::open(const std::string& portName, unsigned int baudRate) {
    close();

    // שמות פורט מעל COM9 דורשים את התחילית "\\.\"
    std::string fullName = "\\\\.\\" + portName;

    HANDLE h = CreateFileA(fullName.c_str(),
                           GENERIC_READ | GENERIC_WRITE,
                           0, nullptr, OPEN_EXISTING, 0, nullptr);
    if (h == INVALID_HANDLE_VALUE) {
        lastError_ = "Failed to open port: " + portName;
        return false;
    }

    DCB dcb;
    std::memset(&dcb, 0, sizeof(dcb));
    dcb.DCBlength = sizeof(dcb);
    if (!GetCommState(h, &dcb)) {
        lastError_ = "GetCommState failed";
        CloseHandle(h);
        return false;
    }
    dcb.BaudRate = baudRate;
    dcb.ByteSize = 8;
    dcb.Parity   = NOPARITY;
    dcb.StopBits = ONESTOPBIT;
    dcb.fBinary  = TRUE;
    dcb.fDtrControl = DTR_CONTROL_ENABLE;
    dcb.fRtsControl = RTS_CONTROL_ENABLE;
    if (!SetCommState(h, &dcb)) {
        lastError_ = "SetCommState failed";
        CloseHandle(h);
        return false;
    }

    COMMTIMEOUTS to;
    std::memset(&to, 0, sizeof(to));
    // קריאה לא חוסמת ברמת ה-OS; הזמן מנוהל ב-readLine
    to.ReadIntervalTimeout         = MAXDWORD;
    to.ReadTotalTimeoutConstant    = 0;
    to.ReadTotalTimeoutMultiplier  = 0;
    to.WriteTotalTimeoutConstant   = 100;
    to.WriteTotalTimeoutMultiplier = 0;
    SetCommTimeouts(h, &to);

    PurgeComm(h, PURGE_RXCLEAR | PURGE_TXCLEAR);

    handle_ = h;
    lastError_.clear();
    return true;
}

void BoatController::close() {
    if (handle_) {
        CloseHandle(static_cast<HANDLE>(handle_));
        handle_ = nullptr;
    }
    rxBuffer_.clear();
}

bool BoatController::isOpen() const {
    return handle_ != nullptr;
}

bool BoatController::writeLine(const std::string& line) {
    if (!isOpen()) { lastError_ = "Port not open"; return false; }
    DWORD written = 0;
    if (!WriteFile(static_cast<HANDLE>(handle_), line.data(),
                   static_cast<DWORD>(line.size()), &written, nullptr)
        || written != line.size()) {
        lastError_ = "WriteFile failed";
        return false;
    }
    return true;
}

bool BoatController::readLine(std::string& out, int timeoutMs) {
    if (!isOpen()) { lastError_ = "Port not open"; return false; }

    ULONGLONG start = GetTickCount64();
    for (;;) {
        // האם כבר יש שורה שלמה בבאפר?
        size_t nl = rxBuffer_.find('\n');
        if (nl != std::string::npos) {
            out = rxBuffer_.substr(0, nl);
            rxBuffer_.erase(0, nl + 1);
            if (!out.empty() && out.back() == '\r') out.pop_back();
            return true;
        }

        char buf[256];
        DWORD got = 0;
        if (ReadFile(static_cast<HANDLE>(handle_), buf, sizeof(buf), &got, nullptr) && got > 0) {
            rxBuffer_.append(buf, got);
            continue;
        }

        if (static_cast<int>(GetTickCount64() - start) >= timeoutMs) {
            lastError_ = "Read timeout";
            return false;
        }
        Sleep(1);
    }
}

#else  // ---------------------- POSIX ----------------------

static speed_t toSpeedT(unsigned int baud) {
    switch (baud) {
        case 9600:   return B9600;
        case 19200:  return B19200;
        case 38400:  return B38400;
        case 57600:  return B57600;
        case 115200: return B115200;
        case 230400: return B230400;
        default:     return B115200;
    }
}

bool BoatController::open(const std::string& portName, unsigned int baudRate) {
    close();

    int fd = ::open(portName.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) {
        lastError_ = "Failed to open port: " + portName;
        return false;
    }

    termios tty;
    std::memset(&tty, 0, sizeof(tty));
    if (tcgetattr(fd, &tty) != 0) {
        lastError_ = "tcgetattr failed";
        ::close(fd);
        return false;
    }

    speed_t spd = toSpeedT(baudRate);
    cfsetispeed(&tty, spd);
    cfsetospeed(&tty, spd);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;   // 8 data bits
    tty.c_cflag |= (CLOCAL | CREAD);              // enable receiver
    tty.c_cflag &= ~PARENB;                       // no parity
    tty.c_cflag &= ~CSTOPB;                        // 1 stop bit
    tty.c_cflag &= ~CRTSCTS;                       // no HW flow control

    cfmakeraw(&tty);                               // raw mode (no processing)
    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 0;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        lastError_ = "tcsetattr failed";
        ::close(fd);
        return false;
    }

    tcflush(fd, TCIOFLUSH);

    fd_ = fd;
    lastError_.clear();
    return true;
}

void BoatController::close() {
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
    rxBuffer_.clear();
}

bool BoatController::isOpen() const {
    return fd_ >= 0;
}

bool BoatController::writeLine(const std::string& line) {
    if (!isOpen()) { lastError_ = "Port not open"; return false; }
    size_t total = 0;
    while (total < line.size()) {
        ssize_t n = ::write(fd_, line.data() + total, line.size() - total);
        if (n < 0) {
            if (errno == EAGAIN || errno == EINTR) continue;
            lastError_ = "write failed";
            return false;
        }
        total += static_cast<size_t>(n);
    }
    return true;
}

bool BoatController::readLine(std::string& out, int timeoutMs) {
    if (!isOpen()) { lastError_ = "Port not open"; return false; }

    for (;;) {
        size_t nl = rxBuffer_.find('\n');
        if (nl != std::string::npos) {
            out = rxBuffer_.substr(0, nl);
            rxBuffer_.erase(0, nl + 1);
            if (!out.empty() && out.back() == '\r') out.pop_back();
            return true;
        }

        fd_set rfds;
        FD_ZERO(&rfds);
        FD_SET(fd_, &rfds);
        timeval tv;
        tv.tv_sec  = timeoutMs / 1000;
        tv.tv_usec = (timeoutMs % 1000) * 1000;

        int r = select(fd_ + 1, &rfds, nullptr, nullptr, &tv);
        if (r < 0) {
            if (errno == EINTR) continue;
            lastError_ = "select failed";
            return false;
        }
        if (r == 0) {
            lastError_ = "Read timeout";
            return false;
        }

        char buf[256];
        ssize_t got = ::read(fd_, buf, sizeof(buf));
        if (got > 0) {
            rxBuffer_.append(buf, static_cast<size_t>(got));
        } else if (got < 0 && errno != EAGAIN && errno != EINTR) {
            lastError_ = "read failed";
            return false;
        }
    }
}

#endif // platform

// =====================================================================
//                     פקודות: מחשב → סירה
// =====================================================================

bool BoatController::sendCommand(int leftSpeed, int rightSpeed, int winchSpeed, int radarAngle) {
    leftSpeed_  = clampInt(leftSpeed,  MOTOR_MIN, MOTOR_MAX);
    rightSpeed_ = clampInt(rightSpeed, MOTOR_MIN, MOTOR_MAX);
    winchSpeed_ = clampInt(winchSpeed, MOTOR_MIN, MOTOR_MAX);
    radarAngle_ = clampInt(radarAngle, RADAR_ANGLE_MIN, RADAR_ANGLE_MAX);

    char line[64];
    std::snprintf(line, sizeof(line), "%d,%d,%d,%d\n",
                  leftSpeed_, rightSpeed_, winchSpeed_, radarAngle_);
    return writeLine(line);
}

bool BoatController::setMotors(int leftSpeed, int rightSpeed) {
    return sendCommand(leftSpeed, rightSpeed, winchSpeed_, radarAngle_);
}

bool BoatController::setLeftSpeed(int speed) {
    return sendCommand(speed, rightSpeed_, winchSpeed_, radarAngle_);
}

bool BoatController::setRightSpeed(int speed) {
    return sendCommand(leftSpeed_, speed, winchSpeed_, radarAngle_);
}

bool BoatController::setWinch(int speed) {
    return sendCommand(leftSpeed_, rightSpeed_, speed, radarAngle_);
}

bool BoatController::setRadarAngle(int angle) {
    return sendCommand(leftSpeed_, rightSpeed_, winchSpeed_, angle);
}

bool BoatController::forward(int speed) {
    return sendCommand(speed, speed, winchSpeed_, radarAngle_);
}

bool BoatController::backward(int speed) {
    return sendCommand(-speed, -speed, winchSpeed_, radarAngle_);
}

bool BoatController::turnInPlace(int speed) {
    // חיובי = ימינה (שמאל קדימה, ימין אחורה)
    return sendCommand(speed, -speed, winchSpeed_, radarAngle_);
}

bool BoatController::emergencyStop() {
    return sendCommand(0, 0, 0, RADAR_ANGLE_CENTER);
}

// =====================================================================
//                    טלמטריה: סירה → מחשב
// =====================================================================

bool BoatController::readTelemetry(Telemetry& out, int timeoutMs, std::string* errorLine) {
    std::string line;
    if (!readLine(line, timeoutMs)) {
        return false; // lastError_ כבר עודכן ב-readLine
    }

    Telemetry t;
    int parsed = std::sscanf(line.c_str(), "%d,%d,%d,%d,%d",
                             &t.usRadar, &t.usFront, &t.usLeft, &t.usRight, &t.radarAngle);
    if (parsed != 5) {
        // לא טלמטריה – ככל הנראה הודעת שגיאה מהממסר (למשל "ERROR: ...")
        if (errorLine) *errorLine = line;
        lastError_ = "Non-telemetry line: " + line;
        return false;
    }

    lastTelemetry_ = t;
    out = t;
    return true;
}
