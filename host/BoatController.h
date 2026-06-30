#ifndef BOAT_CONTROLLER_H
#define BOAT_CONTROLLER_H

#include <string>
#include <cstdint>

// ערכי טלמטריה שמתקבלים מהסירה (כל הערכים בס"מ).
// VALUE_OUT_OF_RANGE (999) מסמן "אין קליטה / רחוק מדי".
struct Telemetry {
    int usRadar = 0;  // חיישן מכ"ם מסתובב
    int usFront = 0;  // חיישן קדמי קבוע
    int usLeft  = 0;  // חיישן שמאלי קבוע
    int usRight = 0;  // חיישן ימני קבוע
};

// מחלקה שמנגישה את ה-API הטורי של ארדואינו החוף (ראו API.md).
// אחראית על פתיחת הפורט, שליחת פקודות וקריאת טלמטריה.
class BoatController {
public:
    // גבולות הפרוטוקול
    static constexpr int MOTOR_MIN          = -255;
    static constexpr int MOTOR_MAX          = 255;
    static constexpr int RADAR_ANGLE_MIN    = 0;
    static constexpr int RADAR_ANGLE_MAX    = 180;
    static constexpr int RADAR_ANGLE_CENTER = 90;
    static constexpr int VALUE_OUT_OF_RANGE = 999;

    BoatController();
    ~BoatController();

    // לא ניתן להעתיק (המחלקה מחזיקה handle של פורט)
    BoatController(const BoatController&) = delete;
    BoatController& operator=(const BoatController&) = delete;

    // --- ניהול חיבור ---

    // פתיחת הפורט הטורי. portName לדוגמה: "COM3" (Windows) או "/dev/ttyUSB0" (Linux).
    // מחזיר true בהצלחה.
    bool open(const std::string& portName, unsigned int baudRate = 115200);

    // סגירת הפורט.
    void close();

    // האם הפורט פתוח כרגע.
    bool isOpen() const;

    // --- פקודות: מחשב → סירה ---

    // שליחת פקודה מלאה (כל 4 השדות). הערכים יוגבלו (clamp) לטווח החוקי.
    // מחזיר true אם הפקודה נשלחה בהצלחה.
    bool sendCommand(int leftSpeed, int rightSpeed, int winchSpeed, int radarAngle);

    // קיצורים נוחים – שולחים פקודה מלאה תוך שמירת ערכי השדות האחרונים.
    bool setMotors(int leftSpeed, int rightSpeed);   // מהירויות שני המנועים
    bool setLeftSpeed(int speed);                    // מנוע שמאל בלבד
    bool setRightSpeed(int speed);                   // מנוע ימין בלבד
    bool setWinch(int speed);                         // כננת הרשת
    bool setRadarAngle(int angle);                    // זווית סרוו המכ"ם
    bool forward(int speed);                          // קדימה ישר
    bool backward(int speed);                         // אחורה ישר
    bool turnInPlace(int speed);                      // סיבוב במקום (חיובי=ימינה)

    // עצירת חירום: "0,0,0,90" – עוצר מנועים וכננת, מכ"ם למרכז.
    bool emergencyStop();

    // --- טלמטריה: סירה → מחשב ---

    // קריאת שורת טלמטריה אחת (חוסם עד timeoutMs אלפיות שנייה).
    // מחזיר true ומעדכן את out אם התקבלה שורה תקינה של 4 מספרים.
    // שורות שאינן 4 מספרים (למשל הודעות "ERROR:") יושמו ב-errorLine אם סופק.
    bool readTelemetry(Telemetry& out, int timeoutMs = 200, std::string* errorLine = nullptr);

    // הטלמטריה האחרונה שנקראה בהצלחה.
    const Telemetry& lastTelemetry() const { return lastTelemetry_; }

    // הודעת השגיאה האחרונה (פנימית של המחלקה).
    const std::string& lastError() const { return lastError_; }

private:
    bool writeLine(const std::string& line);
    bool readLine(std::string& out, int timeoutMs);
    static int clampInt(int value, int lo, int hi);

    // ערכי הפקודה האחרונים (לקיצורים שמשנים שדה בודד)
    int leftSpeed_  = 0;
    int rightSpeed_ = 0;
    int winchSpeed_ = 0;
    int radarAngle_ = RADAR_ANGLE_CENTER;

    Telemetry   lastTelemetry_;
    std::string lastError_;
    std::string rxBuffer_;   // באפר לקריאת שורות חלקיות

#if defined(_WIN32)
    void* handle_ = nullptr;            // HANDLE
#else
    int   fd_     = -1;                 // file descriptor
#endif
};

#endif // BOAT_CONTROLLER_H
