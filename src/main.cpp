#include <RH_ASK.h>
#include <Servo.h>
#include <Adafruit_NeoPixel.h>
#include "rf_link.h"  // פריימים דחוסים + אימות MAC (משותף עם shore_radio.cpp)

// רדיו: speed=2000bps, rxPin=7, txPin=4 (שני הקצות חייבים להתאים!)
// קצב נמוך => מקלט ASK רגיש יותר => טווח טוב יותר. מחיר: זמן-שידור
// לפאקטה מוכפל (~150ms) – לכן ה-heartbeat בשרת הוגדל ל-400ms כדי שה-round-trip ייכנס.
// (ה-TX עבר מ-D6 ל-D4 כדי לפנות את D6 ל-PWM של מנוע)
RH_ASK driver(2000, 7, 4);

// טבעת ניאו-פיקסל (8 לדים) על D11 לחיווי מצב + שליטה מרחוק בצבע.
// הועבר מ-D1 ל-D11: D1 הוא קו ה-TX של ה-USART ומוחזק HIGH במנוחה, ולכן אות ה-
// DATA של ה-NeoPixel (שדורש מנוחה LOW ותזמון מדויק) משתבש ואף לד לא נדלק.
// D11 הוא פין דיגיטלי פנוי ונקי בסירה.
const int PIXEL_PIN = 11;
const int PIXEL_COUNT = 8;
Adafruit_NeoPixel pixel(PIXEL_COUNT, PIXEL_PIN, NEO_GRB + NEO_KHZ800);

// סרוו המכם על D3 (מונע על-ידי Timer1 דרך ספריית Servo)
const int RADAR_SERVO_PIN = 3;
Servo radarServo;

// --- סריקת מכם מקומית ואוטונומית + החלקת תנועה ---
// הסרוו סורק בכוחות עצמו (0..90; 4 החיישנים ב-90° מכסים 360°) לפי טיימר מקומי,
// בלי תלות בפקודות מהחוף. כך אין תלות כפולה בקו התקשורת: הסריקה לא צריכה את
// קו הפקודות (חוף→סירה), והסירה משדרת זווית+מרחקים בכל צעד. התנועה מוחלקת
// בצעדים קטנים (slew) כדי שתהיה רציפה ולא קופצנית. הסרוו נשאר מחובר תמיד — לא
// מנתקים אותו (detach) כדי שלא "יפול" בכבידה ואז יקפוץ בחדות בהתחברות מחדש.
float radarServoPos = 90.0f;               // מיקום נוכחי מוחלק
int   radarServoTarget = 90;               // זווית יעד/סריקה נוכחית
const float RADAR_SLEW_STEP_DEG = 3.0f;    // גודל צעד החלקה בכל פעימה
const int   RADAR_SLEW_STEP_DELAY_MS = 6;  // השהיה בין צעדי החלקה
// פרמטרי הסריקה האוטונומית
const int RADAR_SWEEP_MIN = 0;             // גבול תחתון לזווית הסריקה
const int RADAR_SWEEP_MAX = 90;            // גבול עליון (מספיק ל-360° עם 4 חיישנים)
const int RADAR_SWEEP_STEP = 15;           // צעד סריקה בכל פעימה
const unsigned long RADAR_SWEEP_INTERVAL_MS = 90; // קצב פעימת סריקה
int radarSweepDir = 1;                      // כיוון הסריקה הנוכחי (+1/-1)
unsigned long lastRadarSweepMs = 0;         // חותם-זמן הפעימה האחרונה
unsigned long lastTelemetrySentMs = 0;      // חותם-זמן שידור טלמטריה אחרון

// סרוו הרשת על D10 (אותו Timer1 של Servo — בלי קונפליקט טיימר נוסף)
const int NET_SERVO_PIN = 10;
Servo netServo;
// מעריך את cmd.winchSpeed (טווח שרת: -255..255) לזווית סרוו 0..180;
// 0 = מרכז/ניטרלי (90°). מתאים לסרוו זוויתי וגם לסרוו רציף (90=עצירה).
const int NET_SERVO_NEUTRAL = 90;

// אולטרה-סוני: טריגר משותף ו-4 קווי Echo (המכם על A1 כדי לחסוך פין D)
// הטריגר עבר מ-D5 ל-D2 כדי לפנות את D5 ל-PWM של מנוע
//
// ⚠️ שינוי חיווט (2026-07): החיישן הקדמי המקורי (A1) התקלקל. במקומו חובר
// החיישן שהיה מאחור אל חזית הסירה — אך הוא נשאר על קו ה-Echo המקורי שלו (A5).
// לכן A5 מודד עכשיו את *החזית*, ו-A1 מת. כדי לתקן זאת בלי לגעת בשאר הצינור
// מחליפים כאן בין שני הפינים: US_ECHO_FRONT_PIN => A5 (החיישן העובד שפונה
// קדימה), US_ECHO_RADAR_PIN => A1 (החיישן המת, ממילא מדולג כ"אחורי שבור").
// כך telemetry.usFront נושא את הקריאה הקדמית האמיתית, ושאר הקוד (ניווט, שו"ב,
// תאום-מוק) עובד ללא שינוי. אין חיישן אחורי כלל — נותרו 3 עובדים: חזית/שמאל/ימין.
const int US_TRIG_PIN = 2;
const int US_ECHO_RADAR_PIN = A1;  // החיישן הקדמי המת (מדולג כ"אחורי שבור")
const int US_ECHO_FRONT_PIN = A5;  // החיישן העובד שפונה עכשיו לחרטום
const int US_ECHO_LEFT_PIN = A2;
const int US_ECHO_RIGHT_PIN = A4;
// חלון-האזנה ל-echo. 24ms מכסה ~400ס"מ (התקרה בקוד ממילא 400ס"מ; מעבר לכך → 999),
// וקוצר מ-38ms כדי להאיץ את הלולאה כשקרן רחוקה לא מחזירה החזר.
const unsigned long US_TIMEOUT_US = 24000UL;

// שני מנועים דרך H-Bridge
// מנוע שמאל: כיוון D8/D9, מהירות PWM D5 (Timer0)
const int MOTOR1_IN1 = 8;
const int MOTOR1_IN2 = 9;
const int MOTOR1_PWM = 5;
// מנוע ימין: כיוון D12/D13, מהירות PWM D6 (Timer0)
const int MOTOR2_IN1 = 12;
const int MOTOR2_IN2 = 13;
const int MOTOR2_PWM = 6;

struct CommandPacket {
  int leftSpeed;
  int rightSpeed;
  int winchSpeed;
  int radarAngle;
  int mode;           // 0 = ידני (ירוק), 1 = אוטומטי (כחול) — צבע הטבעת
};

struct TelemetryPacket {
  int usRadar;
  int usFront;
  int usLeft;
  int usRight;
  int radarAngle;
};

CommandPacket cmd = {0, 0, 0, 90, 0};
TelemetryPacket telemetry;

// מוני-מחזור (anti-replay) של פרוטוקול ה-RF:
// lastCmdSeq = ה-seq האחרון שהתקבל מהחוף; telSeq = ה-seq היוצא של הטלמטריה.
// haveCmdSeq=false => הפריים החתום הראשון אחרי אתחול מתקבל בלי בדיקת-רעננות
// כדי להסתנכרן למונה של החוף, ואז חוסמים replay של פריימים ישנים.
uint8_t lastCmdSeq = 0;
bool haveCmdSeq = false;
uint8_t telSeq = 0;
// מוני-מחזור לפריים קונפיג-הניווט (anti-replay נפרד מהפקודות).
uint8_t lastNavSeq = 0;
bool haveNavSeq = false;

// Failsafe: פרק הזמן שבו היעדר פקודה תקינה נחשב "אבד קשר". במצב ידני זה מפעיל
// עצירה והמתנה; במצב אוטומטי הניווט המובנה ממשיך (רק החיווי משתנה).
// 1500ms — "רוכב מעל" דעיכות RF קצרות (טווח) בלי להיחשב מיד כניתוק.
const unsigned long FAILSAFE_TIMEOUT_MS = 1500;
unsigned long lastCommandMs = 0;
// חיווי אובדן קשר על הטבעת (אדום=ידני-ממתין, סגול=אוטונומי-ממשיך). מונע ציור חוזר.
bool linkLost = false;
// המצב האחרון שהוצג על הטבעת (0=ידני,1=אוטומטי). -1 = טרם הוצג,
// מאלץ ציור ראשון. show() מתבצע רק כשהערך הזה משתנה — אירוע נדיר.
int lastMode = -1;

// --- חיווי כיוון על טבעת הלדים (כמו מצפן) ---
// הטבעת מצביעה לכיוון התנועה הנגזר מפקודות המנועים (כיוון הג'ויסטיק):
// קדימה → לד קדמי, פנייה ימינה/שמאלה → צד, אחורה → לד אחורי.
// כשהסירה עומדת (אין פקודה) — הטבעת כבויה.
// LED 0 = חזית הסירה. RING_OFFSET מסובב את המיפוי אם הטבעת מורכבת מסובבת
// פיזית (כל יחידה = 45°). RING_CW=+1 אם הלדים עולים עם כיוון השעון, -1 אם נגדו.
const int RING_OFFSET = 0;
const int RING_CW = 1;
// מתחת לסף התנועה הזה נחשבת עמידה — הטבעת כבויה.
const int MOTION_DEADZONE = 20;

// שידור-גיבוי עצמאי של טלמטריה כשאין פקודות נכנסות (uplink שותק). כשעברו יותר
// מ-UPLINK_SILENCE_MS בלי פקודה, הסירה משדרת בעצמה כל TELEMETRY_FALLBACK_MS
// כדי שהמכם ימשיך לשדר זווית+מרחק גם ללא תקשורת נכנסת.
// הועלה מ-300 ל-550ms: ה-heartbeat בשרת הוא כעת 400ms, כך שבמהלך קשר תקין
// הסף לא נחצה והשידור-גיבוי לא מתנגש עם הפקודה הבאה — הוא נכנס רק בניתוק אמיתי.
const unsigned long UPLINK_SILENCE_MS = 550;
// קריטי ב-2000bps: פריים טלמטריה = ~150ms אוויר. אם משדרים גיבוי כל 150ms
// הסירה משדרת ברציפות ומקלטה חירש (חצי-דופלקס) => לעולם לא תופסת פקודה נכנסת
// => דדלוק. הועלה ל-500ms: הסירה משדרת ~150ms ואז מאזינה ~350ms ותופסת את
// ה-heartbeat (400ms) של החוף, נכנסת ל-"פינג-פונג" והגיבוי מפסיק מעצמו.
const unsigned long TELEMETRY_FALLBACK_MS = 500;

// ===== ניווט אוטונומי מובנה (על-הסיפון) =====
// כל לוגיקת ההימנעות רצה כאן, על הארדואינו של הסירה, בקצב החיישנים המקומי המלא
// (מהיר בהרבה מקישור ה-RF האיטי/לא-יציב). קישור הרדיו משמש רק ל: תמונת שו"ב
// ב-UI, מעבר בין מצב אוטונומי לידני, ופקודות ידניות.
//   • הסירה לעולם אינה יוזמת מצב אוטומטי בעצמה — רק פקודת mode=1 מהחוף מפעילה אותו.
//   • אם התקשורת מתנתקת במצב אוטומטי — הסירה ממשיכה בניווט האוטונומי המובנה.
//   • אם התקשורת מתנתקת במצב ידני — הסירה עוצרת וממתינה עד שהקשר יתחדש.
// ⚠️ החיישן האחורי (usRadar) שבור => אין חישה לאחור => הניווט מעדיף סיבוב-במקום
//    על נסיעה לאחור. יוצא-דופן: כשההתנגשות החזיתית קרובה ומסוכנת (מתחת ל-
//    EMERGENCY) עדיף פרץ-נסיעה קצר לאחור — סיכון אחורי לא-ודאי (חיישן שבור) עדיף
//    על התנגשות חזיתית ודאית. מאחר שנמנעים מנסיעה אחורה ברוב הזמן, טווח ההימנעות
//    הקדמי הוגדל (BLOCK) כדי להגיב מוקדם יותר.

// זווית הסרוו שבה החיישן הקדמי מצביע אל חרטום הסירה (מכיול ~60°). דורש כיול ביבשה.
float NAV_BOW_OFFSET_DEG = 60.0f;
// כיוון הסריקה מול כיוון הסיבוב הפיזי (+1/-1). להפוך אם ימין/שמאל מתחלפים בכיול.
int   NAV_SWEEP_SIGN = 1;
// חצי-רוחב מגזר החרטום (מעלות): קריאה בטווח ±זה נחשבת "לפנים". צר בכוונה (±20)
// כדי שרק מכשול *ישר לפנים* בנתיב יחסום — קיר צד שנראה באלכסון (למשל 20ס"מ לצד
// בזמן שהחזית פתוחה) לא ייכנס לחרוט ולא יעצור את השיוט; קירות-צד מטופלים ע"י
// איזון-הצד. (קודם היה ±45 כפיצוי על קיזוז-חרטום שגוי; עכשיו הקיזוז מדויק אז
// אפשר לצמצם — מונע "החזית חסומה" שקרי ונדידה אלכסונית ליד קיר.)
const float NAV_FRONT_HALF_DEG = 20.0f;
// מרחק שמתחתיו החרטום חסום => עצור קדימה, פנה במקום. הוגדל (טווח הימנעות מוקדם
// יותר, כי נמנעים מנסיעה אחורה), אך נשאר מתחת לרצפת החזר-המים של usFront (~56ס"מ)
// כדי שמים פתוחים לא ייחשבו בטעות לחסומים.
int   NAV_FRONT_BLOCK_CM = 55;
// מרחק שמעליו החרטום נחשב פנוי שוב (היסטרזיס — מונע ריצוד בין נסיעה לסיבוב).
int   NAV_FRONT_CLEAR_CM = 90;
// "סכנת התנגשות חזיתית": קרוב מדי מכדי להסתפק בסיבוב-במקום => פרץ נסיעה לאחור.
int   NAV_FRONT_EMERGENCY_CM = 45;
// ניווט "עקוב-אחר-הפער" (follow-the-gap): במקום לנסוע ישר עד שנתקעים, הסירה
// סורקת קשת קדמית ומכוונת אל הכיוון ה*פתוח ביותר*, ונוסעת רק כשהחרטום מיושר איתו.
// כך היא פונה יזום אל מרחב פתוח (למשל ימינה בהתחלה) ולעולם לא נוסעת לתוך מכשול.
const float NAV_SCAN_HALF_DEG = 90.0f;    // סורקים ±90° מהחרטום לחיפוש הפער העמוק ביותר
const float NAV_ALIGN_DEG = 20.0f;        // חרטום בטווח ±זה מהפער => נוסעים; אחרת מסתובבים אליו
const float NAV_GAP_WINDOW_DEG = 22.0f;   // חצי-חלון החרוט סביב כל כיוון-מועמד (פער רחב, לא תא בודד)
// מרחק שבו כבר מתחילים להחליט לאן לפנות (מוקדם => הפנייה קטנה ובתוך הקשת הקדמית).
int   NAV_FRONT_DECISION_CM = 100;
// חצי-קשת ההחלטה הקדמית (=120° סה"כ): מחפשים בה את הפער ומכוונים אליו מוקדם.
float NAV_DECISION_HALF_DEG = 60.0f;
// משך פרץ הנסיעה-לאחור בחירום (מחויב — מונע ריצוד קדימה/אחורה על רעש חיישן).
const unsigned long NAV_REVERSE_BURST_MS = 600;
// --- שיוט מבוקר: איזון קירות-צד (מירכוז במעברים צרים) ---
// בשיוט קדימה, אם קיר צדדי קרוב מ-NAV_SIDE_TARGET_CM הסירה מטה בעדינות את החרטום
// הצידה (קשת קדימה, לא סיבוב-במקום) כדי להתרחק ולהתמרכז. כששני הצדדים קרובים —
// התיקון הנטו מטה אל הצד הפתוח יותר => מירכוז אוטומטי בערוץ (למשל מעברי 70ס"מ).
const int   NAV_SIDE_TARGET_CM = 40;   // מרחק שיוט אידיאלי מקיר צדדי
const float NAV_SIDE_HALF_DEG  = 45.0f;// חצי-רוחב חרוט הצד (סביב ±90°)
const float NAV_SIDE_GAIN      = 1.6f; // הגבר: כל ס"מ מתחת ליעד => כמה יחידות PWM מורידים מהמנוע הרחוק
// מירכוז חל רק בתוך "מסדרון" — כששני הצדדים קרובים מזה (יש קיר משני הצדדים). קיר
// יחיד עם מים פתוחים בצד השני => נוסעים ישר (לא נוטים לרוחב כל הבריכה).
const int   NAV_SIDE_CORRIDOR_CM = 70;
// מרחק-שמירה מקיר צד: אם צד קרוב מזה, מטים בעדינות *הרחק* ממנו (הורדת המנוע
// הרחוק פרופורציונלית) כדי לשמור מרווח מעבר גם ליד קיר יחיד (קצה באפל בפער).
int   NAV_SIDE_STANDOFF_CM = 40;
// --- דגימת חיישנים: כמה פינגים למדיאן בכל זווית-סרוו ---
const int NAV_PINGS_PER_SAMPLE = 3;    // מדיאן-של-3 (דוחה דרופאאוט/ספייק בלי לטשטש בין כיוונים)
const int NAV_PING_GAP_MS      = 8;    // מרווח בין פינגים (הפחתת ringing של החיישן)
// עוצמות המנוע לניווט האוטונומי נקבעות ע"י שרת החוף ומגיעות בשדות המהירות של
// הפקודה: כשהחוף מעביר למצב אוטומטי הוא שולח את עוצמות המנוע המכוילות (למשל
// שמאל 84 / ימין 88 — המנוע השמאלי חלש יותר). הניווט המובנה משתמש ב|ערך המוחלט|
// של cmd.leftSpeed/cmd.rightSpeed כעוצמה לכל צד ומחליט בעצמו רק על הכיוון
// (קדימה/סיבוב/אחורה). כך הכיול נשאר כולו בחוף — אין צורך לצרוב מחדש כדי לכוונן.
// תוקף קריאת-מגזר: קריאה ישנה מזה נחשבת "לא ידוע". 4000ms מגשר על סריקת-סרוו
// איטית (חרוט חזית צר מתרענן רק כשהסרוו עובר במרכז) כדי שהחזית לא תתיישן בין
// מעברי-סריקה ותגרום לעצירות-המתנה; הסירה איטית וקירות מאומתים בכל סריקה.
const unsigned long NAV_SECTOR_TTL_MS = 4000;

// מצב הניווט המובנה
bool navTurning = false;          // האם כרגע מבצעים סיבוב-הימנעות (עם היסטרזיס)
int  navTurnDir = 1;              // +1 = חרטום ימינה, -1 = חרטום שמאלה
unsigned long navReverseUntil = 0; // סוף פרץ-הנסיעה-לאחור בחירום (0 = לא פעיל)

// מפת-מרחק לפי כיוון (bearing) ביחס לחרטום — 24 תאים ברזולוציית 15° (תואם צעד
// הסריקה ומרווח 90° בין 4 החיישנים). כל תא שומר את המרחק האחרון שנמדד בכיוון
// ההוא + חותם-זמן. זהו המבנה הנכון (כמו liveScan ב-UI): קיר דק שנתפס בזווית-סרוו
// אחת נשאר בתא שלו לאורך ה-TTL, ואינו נמחק ע"י קריאת "פתוח" מזווית סמוכה — כך
// הסירה לא נוסעת דרך מכשול דק שהחיישן פספס ברוב זוויות הסריקה. מרחק המגזר =
// המינימום (הקרוב ביותר) על-פני התאים הטריים בחרוט (coneMin).
const int NAV_BINS = 24;               // 360°/15°
const int NAV_BIN_DEG = 15;
int  navBinDist[NAV_BINS];             // מרחק אחרון לכל תא (ס"מ; 999 = פתוח)
unsigned long navBinT[NAV_BINS];       // חותם-זמן אחרון לכל תא (0 = לא נמדד)

void controlMotor(int in1, int in2, int pwmPin, int speed);
void readAllUltrasonic(int results[4]);
void pingAllUltrasonicOnce(int results[4]);
void showPixel(uint8_t r, uint8_t g, uint8_t b);
void showModeColor(int mode);
void slewRadarServoTo(int target);
void sendTelemetry();
void applyDrive(int left, int right);
void navReset();
void navIngest(int idx, int dist, int servoAngle);
int  navConeMin(float centerDeg, float halfDeg, unsigned long now, bool *anyFresh);
void navDeepest(float loDeg, float hiDeg, unsigned long now, float *bestBearing, int *bestClear);
void navCruiseWithSideBalance(unsigned long now, int magL, int magR, int *leftOut, int *rightOut);
void computeAutonomousDrive(unsigned long now, int *leftOut, int *rightOut);
float navNormalizeDeg(float d);

void setup() {
  pinMode(MOTOR1_IN1, OUTPUT);
  pinMode(MOTOR1_IN2, OUTPUT);
  pinMode(MOTOR1_PWM, OUTPUT);
  pinMode(MOTOR2_IN1, OUTPUT);
  pinMode(MOTOR2_IN2, OUTPUT);
  pinMode(MOTOR2_PWM, OUTPUT);

  pinMode(US_TRIG_PIN, OUTPUT);
  digitalWrite(US_TRIG_PIN, LOW);
  pinMode(US_ECHO_RADAR_PIN, INPUT);
  pinMode(US_ECHO_FRONT_PIN, INPUT);
  pinMode(US_ECHO_LEFT_PIN, INPUT);
  pinMode(US_ECHO_RIGHT_PIN, INPUT);

  radarServo.attach(RADAR_SERVO_PIN);
  radarServo.write(cmd.radarAngle);
  radarServoPos = cmd.radarAngle;
  radarServoTarget = cmd.radarAngle;

  netServo.attach(NET_SERVO_PIN);
  netServo.write(NET_SERVO_NEUTRAL);

  pixel.begin();
  pixel.setBrightness(40);
  showPixel(0, 255, 0); // ירוק = בדיקת תקינות הניאו-פיקסל בהדלקה; נשאר עד תחילת פעילות

  if (!driver.init()) {
    showPixel(60, 0, 0); // אדום קבוע = כשל אתחול רדיו
    while (1);
  }
}

void loop() {
  unsigned long now = millis();

  // 1) קליטת פקודות (לא חוסמת): מעדכנת מצב (ידני/אוטומטי), מהירויות ידניות וכננת.
  //    מיד עם קבלת פקודה משדרים טלמטריה ("פינג-פונג") כדי שהיא תנחת אמין בזמן
  //    שהחוף מאזין. כל פריים חייב לעבור אימות MAC + בדיקת seq — פריים זר/ישן נדחה.
  //    הערה: המנועים אינם מופעלים כאן אלא בשלב 3 (שיפוט אחיד לפי מצב+קשר), כדי
  //    שבמצב אוטומטי הניווט המובנה ישלוט ולא ערוצי המהירות הידניים שבפקודה.
  uint8_t buf[RH_ASK_MAX_MESSAGE_LEN];
  uint8_t buflen = sizeof(buf);
  if (driver.recv(buf, &buflen)) {
    // קונפיג-ניווט מהחוף (כיול/כוונון מה-UI בלי צריבה) — מזוהה לפי אורך הפריים.
    if (buflen == sizeof(RfNavConfig)) {
      RfNavConfig nc;
      memcpy(&nc, buf, sizeof(RfNavConfig));
      if (rf_navcfg_verify(&nc) && (!haveNavSeq || rf_seq_fresh(nc.seq, lastNavSeq))) {
        haveNavSeq = true;
        lastNavSeq = nc.seq;
        NAV_FRONT_BLOCK_CM     = rf_dist_decode(nc.frontBlock);
        NAV_FRONT_CLEAR_CM     = rf_dist_decode(nc.frontClear);
        NAV_FRONT_EMERGENCY_CM = rf_dist_decode(nc.frontEmergency);
        NAV_FRONT_DECISION_CM  = rf_dist_decode(nc.decision);
        NAV_DECISION_HALF_DEG  = (float)nc.decisionHalf;
        NAV_SIDE_STANDOFF_CM   = rf_dist_decode(nc.sideStandoff);
        NAV_BOW_OFFSET_DEG     = (float)nc.bowOffset;
        NAV_SWEEP_SIGN         = (nc.flags & 0x01) ? 1 : -1;
      }
    } else if (buflen == sizeof(RfCommand)) {
    RfCommand in;
    memcpy(&in, buf, sizeof(RfCommand));
    if (rf_cmd_verify(&in) && (!haveCmdSeq || rf_seq_fresh(in.seq, lastCmdSeq))) {
      haveCmdSeq = true;
      lastCmdSeq = in.seq;
      cmd.leftSpeed  = rf_speed_decode(in.left);
      cmd.rightSpeed = rf_speed_decode(in.right);
      cmd.winchSpeed = rf_speed_decode(in.winch);
      int newMode    = (in.flags & 0x01) ? 1 : 0;
      // מעבר לאוטומטי => אתחל את מצב הניווט כדי שלא יסתמך על נעילת-סיבוב ישנה.
      if (newMode == 1 && cmd.mode != 1) navReset();
      cmd.mode = newMode;
      lastCommandMs = now;
      linkLost = false;

      sendTelemetry();
      lastTelemetrySentMs = now;

      // סרוו הרשת: ממפה מהירות -255..255 לזווית 0..180 (מופעל בכל מצב).
      int netAngle = map(constrain(cmd.winchSpeed, -255, 255), -255, 255, 0, 180);
      netServo.write(netAngle);

      // חיווי מצב על הטבעת רק כשהמצב משתנה בפועל — אירוע נדיר — ורק כאן, מיד
      // אחרי קליטה+שידור (החלון הבטוח: החוף מאזין, ואין קליטת RF שתישבש מ-show()).
      if (cmd.mode != lastMode) {
        lastMode = cmd.mode;
        showModeColor(cmd.mode);
      }
    }
    }
  }

  // 2) סריקת מכם אוטונומית מקומית — רצה תמיד לפי טיימר, גם ללא פקודות כלל.
  //    מקדמת את זווית הסריקה, מזיזה את הסרוו, דוגמת את החיישנים, שומרת ל-telemetry
  //    ובנוסף מזינה את תפיסת הניווט המובנה (navIngest) בכל קריאה תקינה.
  if (now - lastRadarSweepMs >= RADAR_SWEEP_INTERVAL_MS) {
    lastRadarSweepMs = now;

    int nextAngle = radarServoTarget + RADAR_SWEEP_STEP * radarSweepDir;
    if (nextAngle >= RADAR_SWEEP_MAX) {
      nextAngle = RADAR_SWEEP_MAX;
      radarSweepDir = -1;
    } else if (nextAngle <= RADAR_SWEEP_MIN) {
      nextAngle = RADAR_SWEEP_MIN;
      radarSweepDir = 1;
    }

    slewRadarServoTo(nextAngle);

    int usResults[4];
    readAllUltrasonic(usResults);
    telemetry.usRadar = usResults[0];
    telemetry.usFront = usResults[1];
    telemetry.usLeft  = usResults[2];
    telemetry.usRight = usResults[3];
    telemetry.radarAngle = nextAngle;

    // הזנת תפיסת הניווט המובנה: כל חיישן (למעט האחורי השבור, idx 0) ממופה
    // לזווית ביחס לחרטום לפי זווית הסרוו ומוכנס למגזר קדמי/ימני/שמאלי.
    for (int i = 0; i < 4; i++) navIngest(i, usResults[i], nextAngle);
  }

  // 3) שיפוט הנעה לפי מצב + מצב-קשר:
  //    linkActive = התקבלה פקודה תקינה בטווח ה-failsafe האחרון.
  bool linkActive = (now - lastCommandMs <= FAILSAFE_TIMEOUT_MS);
  int driveL = 0, driveR = 0;

  if (cmd.mode == 1) {
    // אוטומטי: הניווט המובנה שולט תמיד — עם קשר או בלעדיו. אובדן קשר אינו עוצר.
    computeAutonomousDrive(now, &driveL, &driveR);
  } else {
    // ידני: המנועים מבצעים את הפקודה שהתקבלה. באובדן קשר — עצירה והמתנה.
    if (linkActive) {
      driveL = cmd.leftSpeed;
      driveR = cmd.rightSpeed;
    } else {
      driveL = 0;
      driveR = 0;
      netServo.write(NET_SERVO_NEUTRAL);  // ניטרול הכננת בזמן המתנה
    }
  }
  applyDrive(driveL, driveR);

  // 4) שידור-גיבוי עצמאי של טלמטריה כשקו הפקודות שותק (uplink שקט), כדי שהמכם
  //    ימשיך לשדר תמונת שו"ב גם ללא תקשורת נכנסת.
  if (now - lastCommandMs > UPLINK_SILENCE_MS &&
      now - lastTelemetrySentMs >= TELEMETRY_FALLBACK_MS) {
    sendTelemetry();
    lastTelemetrySentMs = now;
  }

  // 5) חיווי אובדן/חידוש קשר על הטבעת. show() נקרא רק פעם אחת במעבר-למנותק
  //    (אין אז קליטת RF שתישבש), ובחידוש הקשר lastMode=-1 מאלץ ציור-מחדש בשלב 1.
  if (!linkActive && !linkLost) {
    linkLost = true;
    lastMode = -1;  // בחידוש הקשר — לצייר מחדש את צבע המצב
    if (cmd.mode == 1) {
      showPixel(60, 0, 60);  // סגול = אוטונומי ללא קשר (ממשיך לנווט בעצמו)
    } else {
      showPixel(60, 0, 0);   // אדום = ידני ללא קשר (עצר וממתין לחידוש)
    }
  }
}

// מסיע את סרוו המכם אל הזווית המבוקשת בצעדים קטנים ורציפים מתוך המיקום הנוכחי,
// כך שהתנועה חלקה ולא קופצת. הסרוו כבר מחובר (מ-setup) ונשאר מחובר.
void slewRadarServoTo(int target) {
  target = constrain(target, 0, 180);
  radarServoTarget = target;
  while ((int)(radarServoPos + 0.5f) != target) {
    float diff = target - radarServoPos;
    if (diff >  RADAR_SLEW_STEP_DEG) diff =  RADAR_SLEW_STEP_DEG;
    if (diff < -RADAR_SLEW_STEP_DEG) diff = -RADAR_SLEW_STEP_DEG;
    radarServoPos += diff;
    radarServo.write((int)(radarServoPos + 0.5f));
    delay(RADAR_SLEW_STEP_DELAY_MS);
  }
  radarServoPos = target;
  radarServo.write(target);
}

// קורא את כל 4 חיישני האולטרה-סוני במקביל: טריגר אחד, polling על כל ה-echo pins.
// results[0]=radar, [1]=front, [2]=left, [3]=right  (999 = אין מדידה)
// זהו פינג *בודד*; readAllUltrasonic עוטף אותו במדיאן של כמה פינגים לדחיית רעש.
void pingAllUltrasonicOnce(int results[4]) {
  const int echoPins[4] = {
    US_ECHO_RADAR_PIN, US_ECHO_FRONT_PIN, US_ECHO_LEFT_PIN, US_ECHO_RIGHT_PIN
  };

  // טריגר משותף
  digitalWrite(US_TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(US_TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(US_TRIG_PIN, LOW);

  unsigned long riseTime[4] = {0, 0, 0, 0};
  unsigned long pulseWidth[4] = {0, 0, 0, 0};
  bool done[4] = {false, false, false, false};

  unsigned long start = micros();
  while (micros() - start < US_TIMEOUT_US) {
    unsigned long now = micros();
    bool allDone = true;
    for (byte i = 0; i < 4; i++) {
      if (done[i]) continue;
      if (digitalRead(echoPins[i])) {
        if (riseTime[i] == 0) riseTime[i] = now;
      } else {
        if (riseTime[i] != 0) {
          pulseWidth[i] = now - riseTime[i];
          done[i] = true;
        }
      }
      if (!done[i]) allDone = false;
    }
    // יציאה מוקדמת: אם כל 4 החיישנים כבר החזירו הד, אין טעם להמתין עד ה-timeout.
    // כך משחררים את הלולאה מהר וחוזרים לקלוט RF במקום להיחסם 24ms קבועים.
    if (allDone) break;
  }

  for (byte i = 0; i < 4; i++) {
    if (pulseWidth[i] == 0) {
      results[i] = 999;
    } else {
      long cm = (long)pulseWidth[i] / 58;
      results[i] = (cm > 0 && cm <= 400) ? (int)cm : 999;
    }
  }
}

// דוגם כל חיישן NAV_PINGS_PER_SAMPLE פעמים *באותה זווית-סרוו* (הסרוו נייח בזמן
// הדגימה) ומחזיר את המדיאן לכל ערוץ. מדיאן על-פני דגימות באותו כיוון דוחה גם
// דרופאאוט בודד (999) וגם ספייק-פנטום קצר — בלי לטשטש בין כיוונים שונים (מה
// שהיה באג בעבר). מספר קטן וקבוע (3) מספיק: המדיאן מתכנס מהר, וזמן-שהייה ארוך
// מדי בזווית היה ממצע על-פני *מרחב* בזמן תנועה במקום על אותו ערך אמיתי.
void readAllUltrasonic(int results[4]) {
  int samples[4][NAV_PINGS_PER_SAMPLE];
  for (int p = 0; p < NAV_PINGS_PER_SAMPLE; p++) {
    int one[4];
    pingAllUltrasonicOnce(one);
    for (int i = 0; i < 4; i++) samples[i][p] = one[i];
    if (p < NAV_PINGS_PER_SAMPLE - 1) delay(NAV_PING_GAP_MS);  // מרווח קצר → הפחתת ringing
  }
  for (int i = 0; i < 4; i++) {
    // מיון-הכנסה קטן (NAV_PINGS_PER_SAMPLE≈3) ובחירת האיבר האמצעי = מדיאן.
    int a[NAV_PINGS_PER_SAMPLE];
    for (int p = 0; p < NAV_PINGS_PER_SAMPLE; p++) a[p] = samples[i][p];
    for (int x = 1; x < NAV_PINGS_PER_SAMPLE; x++) {
      int key = a[x], y = x - 1;
      while (y >= 0 && a[y] > key) { a[y + 1] = a[y]; y--; }
      a[y + 1] = key;
    }
    results[i] = a[NAV_PINGS_PER_SAMPLE / 2];
  }
}

void controlMotor(int in1, int in2, int pwmPin, int speed) {
  int pwm = abs(constrain(speed, -255, 255));

  if (speed > 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    analogWrite(pwmPin, pwm);
  } else if (speed < 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    analogWrite(pwmPin, pwm);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    analogWrite(pwmPin, 0);
  }
}

// חיווי על כל לדי הטבעת באותו צבע (RGB 0..255, מוגבל על-ידי setBrightness).
void showPixel(uint8_t r, uint8_t g, uint8_t b) {
  for (int i = 0; i < PIXEL_COUNT; i++) {
    pixel.setPixelColor(i, pixel.Color(r, g, b));
  }
  pixel.show();
}

// חיווי מצב על הטבעת: צבע אחיד לכל הלדים לפי המצב שנשלח מהחוף.
// mode==1 (אוטומטי) → כחול; אחרת (ידני) → ירוק. נקרא רק בעת שינוי מצב.
void showModeColor(int mode) {
  if (mode == 1) {
    showPixel(0, 0, 255);   // אוטומטי = כחול
  } else {
    showPixel(0, 255, 0);   // ידני = ירוק
  }
}

// דוחס את הטלמטריה הנוכחית לפריים RfTelemetry, חותם ב-MAC ומשדר. מרחקים
// נדחסים לבית בודד כל אחד (ס"מ/2, 255=אין הד), הזווית בבית, ומצורף seq עולה
// למניעת replay + 2 בתי MAC. החוף מאמת את החתימה לפני שהוא מעביר למחשב.
void sendTelemetry() {
  RfTelemetry out;
  out.seq    = telSeq++;
  out.dRadar = rf_dist_encode(telemetry.usRadar);
  out.dFront = rf_dist_encode(telemetry.usFront);
  out.dLeft  = rf_dist_encode(telemetry.usLeft);
  out.dRight = rf_dist_encode(telemetry.usRight);
  out.angle  = (uint8_t)constrain(telemetry.radarAngle, 0, 180);
  rf_tel_sign(&out);
  driver.send((uint8_t*)&out, sizeof(RfTelemetry));
  driver.waitPacketSent();
}

// ===== ניווט אוטונומי מובנה =====

// מפעיל את שני המנועים בפקודה נתונה (PWM חתום -255..255 לכל צד).
void applyDrive(int left, int right) {
  controlMotor(MOTOR1_IN1, MOTOR1_IN2, MOTOR1_PWM, left);
  controlMotor(MOTOR2_IN1, MOTOR2_IN2, MOTOR2_PWM, right);
}

// מנרמל זווית לטווח (-180, 180].
float navNormalizeDeg(float d) {
  while (d > 180.0f) d -= 360.0f;
  while (d <= -180.0f) d += 360.0f;
  return d;
}

// המרחק הקרוב ביותר (מינימום) על-פני התאים הטריים בחרוט [center±half].
// anyFresh מוחזר true אם נמצא לפחות תא טרי אחד בחרוט (אחרת אין נתון => "לא ידוע").
int navConeMin(float centerDeg, float halfDeg, unsigned long now, bool *anyFresh) {
  int best = 999;
  bool found = false;
  for (int i = 0; i < NAV_BINS; i++) {
    if (navBinT[i] == 0) continue;
    if (now - navBinT[i] > NAV_SECTOR_TTL_MS) continue;  // תא ישן — מתעלמים
    float binBearing = navNormalizeDeg((float)(i * NAV_BIN_DEG));
    if (fabs(navNormalizeDeg(binBearing - centerDeg)) > halfDeg) continue;
    found = true;
    if (navBinDist[i] < best) best = navBinDist[i];
  }
  if (anyFresh) *anyFresh = found;
  return best;
}

// מאתחל את מצב הניווט המובנה (בעת מעבר לאוטומטי): מבטל נעילת-סיבוב ומאפס את
// מפת התאים כך שהניווט לא ינהג לפי נתונים ישנים עד שתגיע סריקה טרייה.
void navReset() {
  navTurning = false;
  navReverseUntil = 0;
  for (int i = 0; i < NAV_BINS; i++) { navBinDist[i] = 999; navBinT[i] = 0; }
}

// מזין קריאת חיישן בודדת למפת התאים: ממפה לפי זווית הסרוו והבסיס הזוויתי של
// החיישן לזווית ביחס לחרטום, ושומר את המרחק בתא המתאים (המגזר האחורי idx=0 נזרק —
// החיישן שבור). "אין הד" (999) הוא מדידה טרייה של "פתוח בטווח מרבי" — נשמר גם הוא
// (לא מדולג) כדי שמים פתוחים = נסיעה קדימה. קיר דק שנתפס בזווית אחת נשאר בתא שלו
// לאורך ה-TTL ואינו נמחק ע"י "פתוח" מזווית סמוכה (תא נפרד) — כך לא נוסעים דרכו.
void navIngest(int idx, int dist, int servoAngle) {
  if (idx == 0) return;  // חיישן אחורי שבור — מתעלמים לחלוטין
  int m = dist;
  if (m <= 0 || m > 999) m = 999;

  // בסיס זוויתי של כל חיישן ביחס לחיישן הקדמי (idx1=front,2=left,3=right):
  float base = (idx == 1) ? 0.0f : (idx == 2) ? -90.0f : 90.0f;
  float bearing = navNormalizeDeg(NAV_SWEEP_SIGN * (float)servoAngle - NAV_BOW_OFFSET_DEG + base);
  int bin = ((int)lroundf(bearing / (float)NAV_BIN_DEG) + NAV_BINS) % NAV_BINS;
  navBinDist[bin] = m;
  navBinT[bin] = millis();
}

// מוצא את הכיוון ה"פתוח ביותר" בקשת [lo,hi]: לכל כיוון-מועמד (בצעדי 15°) מחשב
// את המרחק הפנוי בחרוט סביבו (navConeMin עם חלון), ובוחר את בעל המרחק הגדול ביותר.
// שובר-שוויון: כיוון קרוב יותר לחרטום (|bearing| קטן) — מונע נטייה שרירותית הצידה
// במרחב פתוח אחיד. מחזיר bestClear=-1 אם אין אף תא טרי בקשת.
void navDeepest(float loDeg, float hiDeg, unsigned long now, float *bestBearing, int *bestClear) {
  int best = -1;
  float bestB = 0.0f;
  for (float b = loDeg; b <= hiDeg + 0.5f; b += (float)NAV_BIN_DEG) {
    bool fresh = false;
    int c = navConeMin(b, NAV_GAP_WINDOW_DEG, now, &fresh);
    if (!fresh) continue;
    if (c > best || (c == best && fabs(b) < fabs(bestB))) { best = c; bestB = b; }
  }
  *bestBearing = bestB;
  *bestClear = best;
}

// שיוט קדימה עם מירכוז בתוך מסדרון: נסיעה ישרה בעוצמות שהחוף כייל (magL/magR).
// מירכוז חל *רק* כששני הצדדים הם קירות (שניהם קרובים מ-NAV_SIDE_CORRIDOR_CM) —
// אז מטים בעדינות (פרופורציונלית להפרש) אל הצד הרחוק יותר כדי להתמרכז, וכשמרוכז
// (הפרש~0) נוסעים ישר. קיר יחיד עם מים פתוחים בצד השני => אין תיקון => נסיעה ישרה
// (מונע נטייה אלכסונית על-פני כל הבריכה כשמתחילים ליד קיר). לעולם לא רוורס/סיבוב.
void navCruiseWithSideBalance(unsigned long now, int magL, int magR, int *leftOut, int *rightOut) {
  int l = magL, r = magR;
  bool lFresh = false, rFresh = false;
  int leftD  = navConeMin(-90.0f, NAV_SIDE_HALF_DEG, now, &lFresh);
  int rightD = navConeMin( 90.0f, NAV_SIDE_HALF_DEG, now, &rFresh);

  // שמירת מרווח מכל קיר צד קרוב מ-NAV_SIDE_STANDOFF_CM: מורידים הספק מהמנוע *הרחוק*
  // פרופורציונלית לחוסר => קשת קדימה עדינה המתרחקת מהקיר (מרווח מעבר גם ליד קיר
  // יחיד/קצה באפל). שני צדדים קרובים => שני התיקונים => הנטו מטה אל הצד הפתוח = מירכוז.
  if (lFresh && leftD < NAV_SIDE_STANDOFF_CM) {   // קיר משמאל קרוב => הטה ימינה (הורד מנוע ימין)
    int trim = (int)((NAV_SIDE_STANDOFF_CM - leftD) * NAV_SIDE_GAIN);
    if (trim > magR) trim = magR;
    r -= trim;
  }
  if (rFresh && rightD < NAV_SIDE_STANDOFF_CM) {  // קיר מימין קרוב => הטה שמאלה (הורד מנוע שמאל)
    int trim = (int)((NAV_SIDE_STANDOFF_CM - rightD) * NAV_SIDE_GAIN);
    if (trim > magL) trim = magL;
    l -= trim;
  }
  if (l < 0) l = 0;
  if (r < 0) r = 0;
  *leftOut = l; *rightOut = r;
}

// מחשב את פקודת ההנעה האוטונומית — מכונת-מצבים "שיוט/הימנעות/חירום":
//   • שיוט: כל עוד החרטום פנוי (front >= BLOCK) נוסעים *ישר קדימה* עם איזון
//     קירות-צד (מירכוז בערוץ). לא מכוונים אל פערים רחוקים — כך הסירה מתקדמת
//     במסלול בכוונה ולא נודדת באלכסון.
//   • הימנעות: החרטום חסום (front < BLOCK) => סיבוב-במקום לעבר הצד הפתוח ביותר,
//     מחויב עד שהחרטום עצמו נפתח (front >= CLEAR), ואז חוזרים לשיוט.
//   • חירום: "קופסה" (שום כיוון פתוח) + חזית קרובה מאוד => פרץ נסיעה-לאחור.
//   • אין נתון קדמי טרי => עצירה עד שהסריקה תרענן.
// עוצמת המנוע לכל צד = |המהירות שהחוף שלח|; הניווט קובע רק את הכיוון.
void computeAutonomousDrive(unsigned long now, int *leftOut, int *rightOut) {
  int magL = abs(cmd.leftSpeed);  if (magL > 255) magL = 255;
  int magR = abs(cmd.rightSpeed); if (magR > 255) magR = 255;

  // 0) פרץ נסיעה-לאחור מחויב (מוצא-חירום מ"קופסה"): המשך עד תום הפרץ.
  if ((long)(navReverseUntil - now) > 0) {
    *leftOut = -magL; *rightOut = -magR;
    return;
  }

  // מרחק פנוי בחרוט החרטום (±NAV_FRONT_HALF_DEG).
  bool frontFresh = false;
  int front = navConeMin(0.0f, NAV_FRONT_HALF_DEG, now, &frontFresh);
  if (!frontFresh) { *leftOut = 0; *rightOut = 0; return; }  // אין נתון קדמי טרי => עצור

  // EMERGENCY (overrides everything incl. an ongoing spin): bow too close =>
  //   immediate committed reverse to actively open distance. In-place spinning
  //   does NOT move away from the wall, and drift/waves/wind push us in; back
  //   off at once. MUST precede the turn hysteresis, else a committed spin
  //   swallows this check while we drift into the wall.
  if (front < NAV_FRONT_EMERGENCY_CM) {
    navReverseUntil = now + NAV_REVERSE_BURST_MS;
    navTurning = false;
    *leftOut = -magL; *rightOut = -magR;
    return;
  }

  // 1) התחייבות-לסיבוב (היסטרזיס): ברגע שנכנסנו להימנעות, ממשיכים לסובב-במקום
  //    לעבר הצד שנבחר עד שה*חרטום עצמו* נפתח (front >= CLEAR) — ואז חוזרים לשיוט.
  //    לא מחכים ליישור עם הפער העמוק (זה מה שגרם לנדידה האלכסונית); די בכך שיש
  //    מים פתוחים ישר מלפנים כדי לנסוע.
  // הפער העמוק ביותר בקשת הקדמית של 120° (±NAV_DECISION_HALF_DEG) — יעד הפנייה.
  // מחושב תמיד כדי להחליט *מוקדם* (עוד לפני חסימה) ולוודא שהפנייה נשארת בתוך ה-120°.
  float fwdB; int fwdClear;
  navDeepest(-NAV_DECISION_HALF_DEG, NAV_DECISION_HALF_DEG, now, &fwdB, &fwdClear);

  if (navTurning) {
    if ((fabs(fwdB) <= NAV_ALIGN_DEG && front >= NAV_FRONT_BLOCK_CM) || front >= NAV_FRONT_CLEAR_CM) {
      navTurning = false;
    } else {
      if (navTurnDir > 0) { *leftOut = magL;  *rightOut = -magR; }
      else                { *leftOut = -magL; *rightOut = magR;  }
      return;
    }
  }

  // 2) שיוט: החרטום פנוי => נסיעה *ישר קדימה* (כיוון החרטום) עם איזון קירות-צד.
  //    בלי כיוון אל פערים רחוקים — התקדמות ישרה בכוונה, ממורכזת בערוץ; מכשולים
  //    מטופלים תגובתית בשלב (3).
  bool gapAhead = (fwdClear >= NAV_FRONT_BLOCK_CM && fabs(fwdB) <= NAV_ALIGN_DEG);
  if (front >= NAV_FRONT_DECISION_CM || (gapAhead && front >= NAV_FRONT_BLOCK_CM)) {
    navCruiseWithSideBalance(now, magL, magR, leftOut, rightOut);
    return;
  }

  // 3) הימנעות: החרטום חסום => בוחרים את הצד הפתוח ביותר ומסתובבים-במקום אליו
  //    (מחויב דרך ההיסטרזיס למעלה). סורקים קודם את הקשת הקדמית; אם אין שם פתח,
  //    שוקלים את כל הכיוונים. "קופסה" אמיתית + קרוב מסוכן => פרץ נסיעה-לאחור.
  float target;
  if (fwdClear >= NAV_FRONT_BLOCK_CM) {
    target = fwdB;
  } else {
    float allB; int allClear;
    navDeepest(-180.0f, 180.0f, now, &allB, &allClear);
    target = allB;  // מסתובבים לעבר הפחות-חסום (גם מאחור) עד שייפתח משהו
  }

  navTurnDir = (target >= 0.0f) ? 1 : -1;   // סיבוב לעבר הפער (target>0 => חרטום ימינה)
  navTurning = true;
  if (navTurnDir > 0) { *leftOut = magL;  *rightOut = -magR; }
  else                { *leftOut = -magL; *rightOut = magR;  }
}
