const connectBtn = document.getElementById("connectBtn");
const connectionState = document.getElementById("connectionState");
const modeSwitch = document.getElementById("modeSwitch");
const winchJoystickBase = document.getElementById("winchJoystickBase");
const winchJoystickHandle = document.getElementById("winchJoystickHandle");
const winchSpeedValue = document.getElementById("winchSpeedValue");
const mockSwitch = document.getElementById("mockSwitch");
const portSelect = document.getElementById("portSelect");
const refreshPortsBtn = document.getElementById("refreshPortsBtn");
const serverMessage = document.getElementById("serverMessage");
const downloadNavBtn = document.getElementById("downloadNavBtn");

const replaySelect = document.getElementById("replaySelect");
const replayRefreshBtn = document.getElementById("replayRefreshBtn");
const replayPlayBtn = document.getElementById("replayPlayBtn");
const replayStopBtn = document.getElementById("replayStopBtn");
const replayLoopChk = document.getElementById("replayLoopChk");
const replayStatus = document.getElementById("replayStatus");

const leftSpeedValue = document.getElementById("leftSpeedValue");
const rightSpeedValue = document.getElementById("rightSpeedValue");
const radarAngleValue = document.getElementById("radarAngleValue");
const radarDistanceValue = document.getElementById("radarDistanceValue");

const joystickBase = document.getElementById("joystickBase");
const joystickStick = document.getElementById("joystickStick");

const radarCanvas = document.getElementById("radarCanvas");
const radarCtx = radarCanvas.getContext("2d");

const worldCanvas = document.getElementById("worldCanvas");
const worldCtx = worldCanvas.getContext("2d");
const worldPosValue = document.getElementById("worldPosValue");
const worldInRangeValue = document.getElementById("worldInRangeValue");

const mapCanvas = document.getElementById("mapCanvas");
const mapCtx = mapCanvas.getContext("2d");
const mapPointsValue = document.getElementById("mapPointsValue");
const mapClearBtn = document.getElementById("mapClearBtn");
const motorLeftAbsInput = document.getElementById("motorLeftAbs");
const motorRightAbsInput = document.getElementById("motorRightAbs");
// The mock world picture and the accumulated radar map are meaningful ONLY in
// mock mode (both rely on the simulator's dead-reckoned world pose). On a real
// boat they show stale/garbage, so the whole panels are hidden off-mock.
const worldPanel = worldCanvas.closest(".world-panel");
const mapPanel = mapCanvas.closest(".map-panel");

const state = {
  connected: false,
  mockEnabled: false,
  serialPort: "",
  lastError: "",
  pollId: null,
  eventSource: null,
  commandLoopId: null,
  sendingCommand: false,
  lastRadarAngleSent: 0,
  radarSweepDir: 1,
  radarStepTick: 0,
  manualMode: true,
  // Committed autonomous avoidance turn: 0 = none, -1 = turning left, +1 = right.
  avoidDir: 0,
  // מהירויות המנוע המוחלטות (כיול), נדחפות מהשרת. המנועים 3-מצביים: קדימה / אחורה
  // / עצור, והעוצמה של כל מנוע נקבעת מהכיול (שנועד רק להשוות ביניהם למהירות
  // אפקטיבית שווה). אומדן הפוזה משתמש בהן כדי לשקף את המהירות/התמרון האמיתיים.
  motorLeftAbs: 84,
  motorRightAbs: 88,
  // אודומטריה צד-לקוח (dead-reckoning): אומדן פוזה מתוך אינטגרציה של אותן
  // פקודות המנועים שאנו שולחים, באותו מודל קינמטי כמו הסירה/הסימולטור. זה מה
  // שחומרה אמיתית תעשה (אין GPS/מצפן על החוט), ולכן כל מחסנית התפיסה+הניווט רצה
  // על פוזה משוערת בסימולטור ובמציאות גם יחד — אם זה עובד בסימולטור, זה יעבוד במים.
  // (בסימולטור האומדן עוקב אחר אמת-השרת; boatX/Y/heading משמשים רק לתצוגת אמת לדיבאג.)
  pose: { x: 0, y: 0, headingDeg: 0, speedCms: 0, turnRateRadS: 0 },
  lastPoseT: 0,
  // ניווט מוכוון-יעד (הגעה לקצה השני של הבריכה). מכונת מצבים:
  //   "seek"    - follow-the-gap מוטה אל היעד
  //   "follow"  - עקיבת קו-מתאר של מכשול (Bug2) לחילוץ ממלכודות
  //   "arrived" - הגענו ליעד, עוצרים
  nav: {
    mode: "seek",
    goal: null,            // {x, y} בקואורדינטות עולם
    followHand: 1,         // +1 = שמור מכשול מימין, -1 = משמאל
    wallSince: 0,
    progressAnchor: null,  // {dist, t} תצלום מרחק-ליעד לגילוי תקיעה
    moveAnchor: null,      // {x, y, t} תצלום פוזה לגילוי תקיעה פיזית (חוסר תזוזה)
    escapeUntil: 0,        // חותמת-זמן: עד מתי מבצעים תמרון-חילוץ בנסיעה לאחור
    escapeDir: 0,          // כיוון הפנייה בזמן החילוץ (-1 שמאל, +1 ימין)
    // --- ניווט עולם-אמיתי (מכ"ם מיידי, ללא פוזה) ---
    bowOffsetDeg: 60,      // כיוון החרטום במסגרת הגלם (סרבו+חיישן); נלמד מקוון תוך תנועה
    bowLocked: false,      // האם מעריך-החרטום המקוון התכנס בנסיעה הזו
    rwPhase: "calib",      // "calib" = פעימת-זרע ישרה ללימוד חרטום, "run" = follow-the-gap
    calibActive: false,    // תאימות-לאחור (לא בשימוש עוד)
    calibStart: 0,         // חותמת-זמן תחילת פעימת-הזרע
    // --- פולסי-סיבוב קצרים (סיבוב במקום בציר 0, לא קשתות) ---
    spinUntil: 0,          // חותמת-זמן: עד מתי הפולס הנוכחי מסובב במקום
    spinDir: 0,            // כיוון הסיבוב-במקום הנוכחי (-1 שמאל, +1 ימין)
    settleUntil: 0,        // חותמת-זמן: עד מתי מיישרים/עוצרים בין פולסים
    // --- בריחה מקיר קרוב: אחרי שכיוונּו את החרטום הרחק מהקיר, נוסעים ישר קדימה
    // כדי באמת להתרחק (סיבוב-במקום לבדו לא מזיז אותנו מהקיר וגורם לנדנוד). ---
    wallEscapeUntil: 0,    // חותמת-זמן: עד מתי מחויבים לנסיעה-ישר להתרחקות מקיר
    wallEscapeSide: 0,     // איזה קיר בורחים ממנו (-1 שמאל, +1 ימין)
    reverseUntil: 0,       // חותמת-זמן: עד מתי נמשך פרץ-הנסיעה-לאחור המחויב
    reverseArcDir: 0,      // כיוון קשת-הנסיעה-לאחור (0=ישר, -1 שמאל, +1 ימין)
    sideUntil: 0,          // חותמת-זמן: עד מתי מפעילים מנוע-צד בודד (התחמקות מקיר-צד קרוב)
    sideDir: 0,            // איזה מנוע-צד פעיל כעת (-1 = שמאל, +1 = ימין)
    fwdUntil: 0,           // חותמת-זמן: עד מתי פרץ נסיעה-קדימה (בריחה מקיר שמאחור בזמן נסיגה)
  },
  telemetry: {
    usRadar: null,
    usFront: null,
    usLeft: null,
    usRight: null,
    radarAngle: 0,
    boatX: 0,
    boatY: 0,
    boatHeadingDeg: 0,
  },
  cmd: {
    leftSpeed: 0,
    rightSpeed: 0,
    winchSpeed: 0,
    radarAngle: 90,
  },
  joystick: {
    x: 0,
    y: 0,
    active: false,
  },
  // ניגון חוזר (teach-and-repeat): מנגן מסלול שהוקלט על-ידי שליחת אותן פקודות
  // מנוע/כננת לפי התזמון המקורי, כך שהסירה מבצעת פיזית את הנסיעה מחדש.
  replay: {
    active: false,
    records: [],       // רשומות המסלול המוקלט (t + cmd)
    index: 0,          // מיקום נוכחי במסלול
    baseT: 0,          // חותמת-הזמן של הרשומה הראשונה (עוגן לתזמון יחסי)
    startWallT: 0,     // performance.now() ברגע תחילת הניגון
    name: "",          // שם קובץ המסלול המתנגן
    loop: false,       // ניגון בלולאה אינסופית
  },
};

const sim = {
  maxRange: 200,
};

// Ground-truth mock world (obstacle bodies + boundary), fetched once per mock
// session so the world view can be compared against the radar picture.
const world = {
  loaded: false,
  loading: false,
  boundsHalfX: 600,
  boundsHalfY: 450,
  maxRange: 450,
  targets: [],
  walls: [],     // מכשולי קו-ישר דקים {x1,y1,x2,y2,thickness}
  scenario: null,
  start: null,   // {x, y} נקודת התחלה בעולם (תרחיש מוק)
  goal: null,    // {x, y} נקודת היעד בעולם (תרחיש מוק)
};

const CONTROL_INTERVAL_MS = 250;
const POLL_INTERVAL_MS = 500;

// --- מודל אודומטריה (dead-reckoning) — זהה למודל הפיזיקלי של _mock_loop בשרת ---
// על ידי אינטגרציה של אותן פקודות שאנו שולחים מקבלים אומדן פוזה הזהה למה שהסירה
// האמיתית תעשה. הערכים חייבים להיות זההים לשרת כדי שהאומדן יישאר צמוד לאמת בסימולטור.
const DR_MAX_SPEED_CMS = 45;          // מהירות שיוט יציבה במלוא המצערת (גבול הולל)
const DR_TURN_RATE = Math.PI * 0.25;  // rad/s — קצב סחרור יציב בהיגוי דיפרנציאלי מלא
// תנע: הכלי אינו נקודה קינמטית. הדחף מהמנועים נאבק בגרר המים, ולכן הכלי ממשיך
// להחליק אחרי שחרור המצערת וזקוק לזמן להאיץ/להסתובב. הקבועים זהים לשרת
// (MOCK_LINEAR_DRAG / MOCK_TURN_DRAG ב-control_server.py) כדי שהאומדן יישאר צמוד לאמת.
const DR_LINEAR_DRAG = 1.4;           // 1/s — קבוע זמן שיוט ~0.7 שנ'
const DR_TURN_DRAG = 2.5;             // 1/s — קבוע זמן סחרור ~0.4 שנ'
// דירוג כוח המנוע לפי ערך מוחלט (shape_motor_speed בשרת), מוחל גם באוטונומי וגם
// בידני:
//   |v| < 35  -> 0   (אזור-מת: חלש מדי -> מנוע כבוי)
//   אחרת      -> נחתך לרצועת הכוח השמישה [70, 100] (רציף)
// סקאלת השליטה של המשתמש היא רצועת 70..100 הזו; הכיוון (הסימן) נשמר. חייבים
// לשקף כאן את אותה עיצוב כדי שהאומדן יישאר צמוד לאמת — וכדי שגם הניווט האוטונומי
// (שנוהג דרך אותה פיזיקה) יעבוד באותה רצועה.
// כוח המנוע מוגבל ל-3 מצבים בלבד לכל מנוע: 80 / -80 / 0 (מוחל גם באוטונומי וגם
// בידני). כל ערך פקודה נחתך למדרג הזה — עוצמה מתחת לאזור-המת מכובה, וכל השאר
// מקובע לעוצמה היחידה MOTOR_SPEED תוך שמירת הסימן (כיוון).
const MOTOR_DEADZONE = 40;    // מתחת לעוצמה זו -> 0 (כבוי)
const MOTOR_SPEED = 80;       // העוצמה היחידה בכל מנוע (80 קדימה / -80 אחורה)
function shapeMotorSpeed(value) {
  if (Math.abs(value) < MOTOR_DEADZONE) return 0;
  return value > 0 ? MOTOR_SPEED : -MOTOR_SPEED;
}
// מהירות שיוט אוטונומית. מכוונת בכוונה נמוכה יחסית כדי שהמכ"ם הסורק (מסתובב על
// סרבו, מכסה את המעגל ב~1 שנייה) תמיד "מדביק" גופים שנכנסים לחזית לפני מגע —
// במהירות גבוהה הסירה עלולה לחצות פער-סריקה ולהיכנס לגוף שטרם נסרק.
// עוצמת הניווט נשלחת בסולם המלא (עד 255) והעיצוב חותך אותה לרצועת
// כוח המנוע {0} ∪ [70, 100] (shapeMotorSpeed ב-computeSafeCommand/integratePose +
// shape_motor_speed בשרת), כך שבפועל המנוע רץ ברצועה הזו — גם באוטונומי.
const AUTONOMOUS_SPEED = 135;
const AVOID_SPEED = 170;
// Below this front distance the boat commits to spinning in place toward open
// water. It keeps spinning (hysteresis) until the bow is clear past
// CLEAR_DISTANCE_CM, so it never chatters on/off right at one threshold.
// המרחק נמדד ממרכז הסירה; הסירה באורך 30 ס"מ (חצי-אורך 15), לכן ערך של 40 ס"מ
// משאיר ~25 ס"מ אוויר בין דופן-הסירה לקיר.
const SAFE_DISTANCE_CM = 40;
const CLEAR_DISTANCE_CM = 64;
// While cruising, start gently steering away from an obstacle this far ahead so
// the boat arcs around it smoothly instead of charging up and hard-spinning.
const STEER_LOOKAHEAD_CM = 100;
// עוצמת ההיגוי המרבית. הפקודות נשלחות בסולם המלא ומעוצבות בפיזיקה לרצועת הכוח
// {0}∪[40,100], כך שסיבוב חד מעביר את הגלגל הפנימי לאחור לפיבוט תוך כדי שיוט.
const STEER_MAX = 120;
// --- ניווט יזום לעבר מרחב פתוח ("follow the gap") ---
// במקום לחכות שהחרטום ייחסם ואז להתחמק, הניווט סורק את קשת החזית ובוחר בכל רגע
// את הכיוון עם המרחק הפנוי הגדול ביותר (הכי פחות צפוף) — כולל התחשבות בקירות —
// ומטה את הסירה לעברו. כך היא נמשכת אל מים פתוחים ונמנעת מלהיכנס לצבירי גופים
// מלכתחילה, במקום להסתבך בתוכם ולהיתקע.
const OPEN_SCAN_DEG = 85;        // סורקים ±זווית זו סביב החרטום (קשת חזית)
const OPEN_SCAN_STEP = 10;       // צעד הסריקה במעלות
const OPEN_SCAN_TOL = 12;        // סובלנות זוויתית לכל כיוון-מועמד (חופף לצעד)
const OPEN_CLEAR_CAP = 110;      // ס"מ - מעבר לזה כיוון "פתוח מספיק" (לא מתגמל עוד)
const OPEN_TURN_PENALTY = 0.5;   // ס"מ קנס לכל מעלת סטייה מהחרטום (מאזן בין
                                 // "להמשיך ישר" ל"לפנות אל הפתוח") — פונה חזק רק
                                 // אם הכיוון הפתוח פנוי משמעותית יותר.
const OPEN_STEER_GAIN = 1.5;     // ממיר זווית-יעד (מעלות) לעוצמת היגוי

// --- ניווט מוכוון-יעד: הגעה לקצה השני של הבריכה ---
// המוצא קרוב לאחת הדפנות; היעד הוא הדופן שממול. הסירה נמשכת אל היעד (איבר משיכה
// ב-follow-the-gap), ואם היא נתקעת (חוסר התקדמות נטו) היא עוברת לעקיבת קו-מתאר
// בסגנון Bug2 שמובטח שיחלץ אותה ממלכודות קעורות/פינות — ואז חוזרת לשיוט אל היעד.
const GOAL_MARGIN_CM = 48;        // עד כמה מהקיר הרחוק ממקמים את נקודת היעד
const GOAL_ARRIVE_CM = 40;        // מרחק מהיעד שנחשב "הגענו" -> עצירה
const GOAL_ATTRACT_PENALTY = 2.0; // ס"מ קנס לכל מעלת סטייה מכיוון היעד (מחליף את
                                  // OPEN_TURN_PENALTY במצב seek -> מטה אל היעד)
// כמה "לצדד" אל היעד: כשעדיין רחוקים ממנו אנכית, נקודת-הכיוון מתקרבת אל מעל
// הסירה (טיפוס כמעט-אנכי דרך הפערים; ההתחמקות הריאקטיבית מטפלת בעיקוף הדפנות),
// וככל שמתקרבים ליעד אנכית המשיכה הצדדית גדלה עד לכיוון אל היעד עצמו. זה מונע
// מהמשיכה-אל-פינת-היעד למשוך את הסירה לתוך דופן-הצד לפני שחצתה את הפער.
const GOAL_LATERAL_RANGE = 70;   // ס"מ - סקאלת מרחק-אנכי למשיכה הצדדית
// ציר-האורך (x) של הערוץ המשותף שכל פערי-המחסומים חופפים בו. המחסומים
// סימטריים סביב מרכז הבריכה (ראשית x=0), לכן טיפוס אנכי ב-x=0 חוצה את שלושת
// המחסומים בבטחה. רק לקרבת היעד מסיטים אל עמודת-ה-x שלו.
const CHANNEL_CENTER_X = 0;
// גלאי-תקיעה: מודד תזוזה *מרחבית נטו* (לא ירידה במרחק-האווירי-ליעד). מסלול תקין
// סביב מכשול/מחיצה מחייב לעיתים להתרחק מהיעד לאורך זמן (המרחק-האווירי גדל), ולכן
// מדד מבוסס-מרחק-ליעד היה מזהה "תקיעה" בטעות בכל עיקוף. במקום זה: אם הסירה לא זזה
// במרחב יותר מ-STUCK_TRAVEL_CM נטו בתוך חלון הזמן — היא באמת לכודה (מסתובבת סביב
// עצמה במינימום מקומי) ורק אז עוברים לעקיבת-דופן. עיקוף פרודוקטיבי צובר תזוזה
// מרחבית גדולה ולכן לעולם לא מפעיל אותו.
const STUCK_WINDOW_MS = 6000;
const STUCK_TRAVEL_CM = 38;
// חילוץ מתקיעה פיזית (deadlock): סיבוב-במקום אינו מזיז את הסירה כלל, כך שאם היא
// "כלואה" (כל כיוון בר-השגה חסום בטווח SAFE) היא עלולה להתנדנד לנצח בלי שמיקומה
// ישתנה — והמכשול שלפניה יישאר באותו מרחק. לכן אם אין תזוזה נטו מעבר ל-
// ESCAPE_MOVE_CM בתוך ESCAPE_STALL_MS, מבצעים תמרון-חילוץ: נסיעה לאחור בקשת אל
// הצד הפתוח, שמייצרת תזוזה אמיתית ופותחת מרחב מלפנים. אמין וגם ריאל-טרנספרבילי.
const ESCAPE_STALL_MS = 2600;    // אין תזוזה כזמן הזה -> נכנסים לחילוץ
const ESCAPE_MOVE_CM = 14;       // תזוזה נטו מעבר לזה מאפסת את שעון-התקיעה
const ESCAPE_DURATION_MS = 1400; // משך פרץ הנסיעה-לאחור
const ESCAPE_SPEED = 130;        // עוצמת נסיעה לאחור (שני המנועים)
const ESCAPE_TURN = 55;          // הפרש-היגוי בזמן החילוץ (מטה את הקשת לצד הפתוח)// מצב עקיבת-דופן (Bug2): בקר-P שומר מרחק-סף קבוע מהדופן/גוף עד שכיוון היעד נפתח.
const WALL_FOLLOW_STANDOFF_CM = 45;
const WALL_FOLLOW_SPEED = 120;
const WALL_FOLLOW_GAIN = 1.2;     // המרה משגיאת-מרחק (ס"מ) לעוצמת היגוי
const WALL_LEAVE_CLEAR_CM = 100;  // כיוון היעד נחשב "פתוח" מעל מרחק פנוי זה -> עזיבה
// How often the servo advances one sweep step. The four sensors are only 15°
// wide and sit 90° apart, so at any instant they cover just 4×15°=60° of the
// full circle — everything else is a momentary blind gap. The ONLY thing that
// fills those gaps is the sweep. The servo steps 0→15→30→45→60→75 then back
// 75→60→45→30→15→0 (six 15° slots). The sweep advances one 15° slot every
// RADAR_STEP_EVERY_TICKS *sent* commands, so the rotation rate is perfectly
// even and tied to the data cadence (one fresh angle per sample) instead of a
// wall-clock timer that drifted against the loop and produced uneven jumps.
// At CONTROL_INTERVAL_MS=300ms and 1 tick/step that's ~300ms/slot, so a full
// 0..75..0 cycle (~10 steps) takes ~3s.
const RADAR_STEP_EVERY_TICKS = 1;
// A slot's reading is trusted for this long after it was last measured, then it
// vanishes from the picture. The lifetime is deliberately ONE back-and-forth
// sweep cycle: the servo walks 0..75..0 in ten 15° steps, so at
// CONTROL_INTERVAL_MS=250ms a full round trip is ~2.5s. A short TTL (~3s, just
// over one cycle) means each blip lives for exactly one sweep and then expires —
// so moved/phantom returns don't linger as stale rings while every bearing is
// still refreshed at least once before it dies.
const RADAR_TTL_MS = 3000;
// חלון התמדה לתצוגה בלבד. על החומרה האמיתית הסריקה המסתובבת (רבע-סריקה 0..90°, 4
// חיישנים 90° זה מזה) חוזרת לאותו כיוון רק כל ~5 שניות, ועוד יותר בזמן תקיעות RF.
// אם מוחקים בליפ/קיר אחרי RADAR_TTL_MS(3s) הם פגים לפני שהסריקה חוזרת לרענן אותם —
// ולכן התמונה "מנצנצת": נקודות/קווים נעלמים וחוזרים כל הזמן. לצג נחזיק כל בליפ עד
// חלון של סריקה מלאה + שיהוי-קישור, כדי שהתמונה תישאר רציפה. זה משפיע רק על מה
// שמצויר; הניווט האוטונומי ממשיך לקרוא רק נתונים טריים (RADAR_TTL_MS / liveScan).
// מדידה על הלוגים האמיתיים (nav-*.ndjson): קצב פריימים טריים מהסירה = ~0.9–3.5
// לשנייה עם פערים עד ~8 שניות (תקיעות RF). כל פריים מקדם את הסרבו ב-15° בלבד,
// אז סריקה מלאה (0..90..0, ~14 צעדים) לוקחת ~5–14 שניות. לכן חלון התצוגה חייב
// לכסות סריקה מלאה כדי שהתמונה לא תנצנץ (6s היה קצר מדי לקצב הנמדד).
const RADAR_DISPLAY_TTL_MS = 12000;
// חלון איסוף נקודות להתאמת קו-קיר (נפרד מחלון הנקודות לתצוגה): מספיק ארוך כדי
// לאסוף >=3 נקודות על הסריקה האיטית, אבל לא ארוך מדי כדי שנקודות מפוזרות על פני
// תנועת הסירה (דד-רקונינג גס) ימרחו את הקו. הקטע שהותאם נשמר אח"כ WALL_TTL_MS.
const RADAR_WALL_FIT_TTL_MS = 6000;
// מילוי-פער זוויתי (חומרה אמיתית בלבד): הסרבו סורק קשת מלאה ~פעם בשנייה, אבל ה-RF
// מביא רק ~1-3 פאקטות לשנייה, אז רוב הזוויות שנסרקו אף פעם לא מגיעות. כשמגיעה פאקטה
// חדשה, "צובעים" את הפער הזוויתי מאז הפאקטה הקודמת באינטרפולציה ליניארית בין שתי
// הקריאות — כך פאקטה אחת ממלאת קשת שלמה במקום סלוט 15° בודד. הנקודות הממולאות
// מסומנות filled ומשמשות לתצוגה בלבד (לעולם לא מזינות את הניווט).
const RADAR_GAPFILL_MAX_MS = 1200;       // ממלאים רק אם הפער בזמן קטן (אחרת האינטרפולציה חסרת משמעות)
const RADAR_GAPFILL_STEP_DEG = 5;        // רזולוציית המילוי
const RADAR_GAPFILL_MAX_SPAN_DEG = 80;   // לא ממלאים פער זוויתי רחב מדי (כנראה קפיצה/באונס)
// כשמאבדים תקשורת (לא מגיע פריים טלמטריה חדש) חייבים להמשיך לראות את תמונת המכ"ם
// האחרונה במקום שהיא תדעך לריק. ה-TTL נועד לפוג בלימים שלא רועננו בזמן שהסריקה
// *ממשיכה* — אבל אם הסריקה עצמה נעצרה (אין תקשורת), אין מה שיחליף אותם, אז מקפיאים
// את שעון-ההזדקנות של הצג. אחרי פרק זמן זה ללא פריים טרי, ה-radarClock קופא, כך
// שהקירות והנקודות האחרונים נשארים על המסך; כשהתקשורת חוזרת ההזדקנות ממשיכה
// והבלימים שלא רועננו פגים כרגיל. הערך קטן מ-RADAR_DISPLAY_TTL_MS כך שאף בליפ לא נמחק לפני הקיפאון.
const RADAR_FREEZE_AFTER_MS = 1500;

// --- שער ייצוב-תמונה בזמן סיבוב במקום ---------------------------------------
// אין מצפן/IMU על החוט, ולכן ה-heading מוערך ב-dead-reckoning מהפקודות בלבד. בזמן
// סיבוב במקום ההערכה לא אמינה (כיול קצב-סחרור, החלקה, שיהוי קישור), וכל הד נשמר
// בזווית-עולם שגויה — התמונה "נמרחת"/מסתחררת. לכן: כשקצב-הסחרור עולה מעל הסף
// מקפיאים את צבירת המכ"ם ומנקים את התמונה הישנה (שנשמרה תחת heading שכבר לא תקף),
// ומחזיקים אותה "לא-יציבה" עוד RADAR_SETTLE_MS אחרי שהסיבוב נעצר — כדי שסריקה
// טרייה תבנה אותה מחדש לפני שהניווט/המפעיל מסתמכים עליה שוב. חל על חומרה אמיתית
// בלבד (בסימולטור ה-heading הוא אמת-קרקע, אז אין מריחה ואין צורך לשער).
const RADAR_SPIN_RATE_DEG_S = 12;   // סף קצב-סחרור (°/ש') שמעליו הפוזה לא אמינה — סיבוב-במקום ~15°/ש', קשת ~7.5°/ש'
const RADAR_SETTLE_MS = 1400;       // מינימום זמן-קיר להמתין אחרי שהסיבוב נעצר עד שהתמונה נחשבת יציבה שוב
// הזמן לבדו לא מספיק: קישור ה-RF מספק רק ~1-3 פריימים טריים בשנייה, ואחרי סיבוב
// (שמנקה את radarMemory בחומרה אמיתית) 1400ms מביאים רק ~1-4 צעדי-סריקה — התמונה
// עדיין כמעט ריקה. לכן דורשים גם מספר מינימלי של פריימי-טלמטריה טריים שהצטברו מאז
// שהסיבוב נעצר, כך שהמתנה ב-קישור איטי מתארכת דה-פקטו עד שיש באמת מספיק נתונים
// לבנות תמונה. בקישור מהיר זה כמעט שקוף (הפריימים מגיעים מהר).
const RADAR_SETTLE_MIN_FRAMES = 5;  // מס' פריימי-טלמטריה טריים מינימלי אחרי סיבוב לפני שהתמונה נחשבת יציבה

// --- אינטרפולציה ליניארית פר-צד --------------------------------------------------
// האולטרה-סוניק גס: קיר ישר יוצא כנקודות מפוזרות/קו עקום. כל חיישן (חזית/ימין/
// אחור/שמאל) סורק קשת ~75° משלו, ולכן מעבדים כל צד בנפרד: מתאימים קו ישר יחיד
// לנקודות החיות של אותו צד (רגרסיה אורתוגונלית — עובד גם לקיר "אנכי"), ומציירים
// אותו כקטע רציף על הצג + מזינים את הטווח המתוקן לניווט. כך גם כשהסריקה האיטית
// של החומרה מדלגת על סלוט בודד, מוצג קו מלא ורציף במקום נקודות מנצנצות.
const RADAR_FIT_MIN_POINTS = 3;          // צריך לפחות 3 נקודות כדי להצדיק קו
const RADAR_FIT_MAX_RESIDUAL_CM = 25;    // אם הפיזור סביב הקו גדול מזה — לא קיר, לא מציירים
// דחיית "טבעת"-רפאים: קיר שטוח אמיתי שנסרק על פני קשת רחבה מראה טווח שגדל לעבר
// הקצוות (∝1/cos מהניצב). הד-רפאים (מים/קרקע/cross-talk) מחזיר טווח כמעט-קבוע בכל
// הכיוונים, וקשת בטווח קבוע מתאימה לקו בשיורי נמוך — ולכן עוברת את בדיקת השיורי.
// לכן: אם צד נפרס על קשת רחבה מ-RADAR_RING_ANG_MIN_DEG אך הטווח בקצוות אינו רחוק
// מהמרכז לפחות פי RADAR_RING_EDGE_RATIO — זו טבעת-רפאים, לא קיר. ראה isPhantomRing.
const RADAR_RING_ANG_MIN_DEG = 45;       // מפעילים את הבדיקה רק על קשת רחבה
const RADAR_RING_EDGE_RATIO = 1.12;      // קיר אמיתי ב-45°+: קצה רחוק מהמרכז לפחות פי זה
const WALL_TTL_MS = 10000;               // קטע-קיר מיושר נשמר עד שהסריקה האיטית תחזור לרעננו, כדי שלא ינצנץ
// חיבור קצות-קירות: אם הפער בין קצה של קיר אחד לקצה של קיר אחר קטן מזה, הם
// "נפגשים". הקירות מיושרים למסגרת-רשת משותפת {φ, φ+90°}, ולכן זוג קירות הוא תמיד
// מקביל (מתמזג לקו ישר, ~180°) או ניצב (נפגש בפינה, ~90°). ראה joinNearbyWalls.
const WALL_JOIN_GAP_CM = 30;
// פינה = שני קירות ניצבים (במסגרת-הרשת). קצוותיהם ליד הפינה נמדדים בדלילות ולכן
// עשויים להישאר עם פער קטן, ואז מאריכים אותם עד נקודת-החיתוך כדי שייפגשו לפינה
// נקייה. אבל אסור לחבר קווים שרחוקים זה מזה: פער גדול הוא מעבר/פתח אמיתי במסלול,
// לא פינה. לכן מגבילים את טווח-ההגעה ל-40 ס"מ — מעל זה לעולם לא מחברים.
const RADAR_CORNER_REACH_CM = 40;
// המכ"ם סורק 0..90° ומגלגל את כל 4 החיישנים יחד. אמצע-הסריקה (45°) מיושר עם החרטום,
// ולכן בצג מסובבים את התמונה ב-SWEEP_CENTER_DEG כדי שהחיישן הקדמי (הירוק) יהיה
// למעלה ויסרוק ימינה-שמאלה סימטרית סביב החרטום, ושאר החיישנים בהתאמה.
const SWEEP_CENTER_DEG = 45;
// קו-קיר אמיתי סטטי בעולם, ולכן נקודות-הקצה שלו בקואורדינטות עולם כמעט לא זזות בין
// פריים לפריים — הקפיצות הן רעש-פיטינג. מחליקים את הקצוות ב-EMA (חלק מהמדידה החדשה
// בכל פריים) כדי שהקו יזוז חלק ורציף במקום לקפוץ. זה גם מייצב את הטווח שהניווט קורא
// (שנגזר מאותו קו), ולכן משפר גם את האוטונומי. ערך נמוך = חלק יותר אך איטי יותר.
const RADAR_WALL_SMOOTH = 0.25;

// --- סינון חציון לרעש חיישנים (חומרה מחזירה שגיאות מדידה) --------------------
// חומרה אמיתית זורקת מדי פעם קריאה שגויה: dropout (999) או קפיצת-רפאים קצרה,
// ומעל הכל רעש מדידה של כמה ס"מ. חציון על 3 הקריאות האחרונות של כל ערוץ דוחה
// חריגים בודדים (הן dropout והן spike) לפני שהניווט/המפה מגיבים אליהם, מבלי
// לעכב זיהוי מכשול אמיתי ביותר מ~2 טיקים (~100ms).
const SENSOR_MEDIAN_WINDOW = 3;
const _sensorHistory = { usFront: [], usRight: [], usRadar: [], usLeft: [] };
function filterSensorReading(key, raw) {
  const hist = _sensorHistory[key];
  if (!hist || raw == null) return raw;
  hist.push(raw);
  if (hist.length > SENSOR_MEDIAN_WINDOW) hist.shift();
  const sorted = [...hist].sort((a, b) => a - b);
  return sorted[Math.floor(sorted.length / 2)];
}
function resetSensorHistory() {
  for (const key of Object.keys(_sensorHistory)) _sensorHistory[key].length = 0;
}

// שכבת הגנה והתחמקות אקטיבית
// רשת הביטחון האחרונה חייבת להיות פנימית לטווח שבו הניווט האוטונומי כבר מתחמק
// (SAFE_DISTANCE_CM = 70). אחרת שכבת הבטיחות "חוטפת" את ההגה מהניווט לפני שהוא
// מספיק לתמרן, ושני הבקרים נלחמים ומייצרים זיג-זג. לכן טווח האזהרה קטן מ-70.
const AVOID_SAFETY_RADIUS = 18;  // ס"מ - קו אדום ל-J-Turn, אסור לסירה להיות במרחק כזה ממכשול
const AVOID_WARNING_RADIUS = 23; // ס"מ - טווח הדיפה (קטן מ-SAFE_DISTANCE_CM כדי לתת לניווט להתחמק קודם)
// ניפוח מכשולים (Obstacle Inflation): הניווט מתייחס לסירה כאל נקודה, אך לגוף יש
// אורך פיזי וציר-הסיבוב אינו ממורכז — בסיבוב-במקום זנב הסירה "מטאטא" קשת ועלול
// לפגוע במכשול שהחרטום פינה. לכן מנפחים כל מכשול ברדיוס = חצי אורך הסירה + מרווח
// ביטחון, ומקזזים אותו ממרחק-הפנוי שהניווט רואה. כך כל שכבת התכנון (בחירת כיוון
// פתוח, ספי התחמקות, עקיבת-דופן) שומרת על מעטפת הגוף במקום על נקודה בודדת.
const OBSTACLE_INFLATION_CM = 7;
// מושל מהירות קדימה: אסור לדהור לעבר מכשול שזוהה מהר מכפי שאפשר לפנות ממנו.
// המהירות קדימה נחתכת כפונקציה של המרחק הפנוי בחרוט שלפני החרטום, כך שגוף חזיתי
// דוחף את המצערת לאפס הרבה לפני מגע — הסירה עדיין יכולה להסתובב במקום ולחפש פתח,
// אבל פיזית אינה יכולה להתקדם לתוך גוף. פועל גם בשליטה ידנית וגם באוטונומית.
const GOVERNOR_RANGE_CM = 85;    // ס"מ - מתחילים להאט מהמרחק הזה
const GOVERNOR_STOP_CM = 24;      // ס"מ - מהירות קדימה מתאפסת במרחק (משטח) הזה
const GOVERNOR_FWD_CONE_DEG = 32; // חרוט צר סביב החרטום בלבד — רק מכשול כמעט-חזיתי
                                  // מגביל מהירות. כל ההיגוי (חיפוש מרחב פתוח
                                  // והתחמקות) מטופל ע"י שכבת הניווט, כך שהמושל לא
                                  // כופה כיוון פנייה משלו ולא נאבק עם הניווט.

// Accumulated radar map: unlike the boat-centric radar view, this builds a
// persistent picture in FIXED world coordinates. Each valid sensor reading is
// projected to its absolute (world) hit point using the boat's dead-reckoned
// pose, then quantised into a grid cell so memory stays bounded while repeated
// hits on the same spot raise that cell's confidence.
const MAP_CELL_CM = 12;
const MAP_MAX_CELLS = 4000;
const MAP_TRAIL_MAX = 600;
// A sonar returns a range within a ~15° beam, so one reading is an ARC of
// possible obstacle positions — not a point. We paint the whole arc; a cell is
// trusted as a real obstacle only where arcs from different bearings/positions
// INTERSECT (>= MAP_CONFIRM_HITS). A lone arc leaves each cell well below the
// threshold, so phantom single-beam echoes never register.
const MAP_ARC_FOV_DEG = 7.5;   // half-width of the sensor beam (matches the sim)
const MAP_ARC_STEP_DEG = 2.5;  // angular sampling resolution along the arc
const MAP_HIT_CAP = 15;        // bound per-cell confidence so clearing can erode it
const MAP_CONFIRM_HITS = 2;    // intersecting arc weight required to trust a cell
// The farther a reading is, the wider its 15° beam smears across the grid, so a
// lone far echo might be a single stray point spread over many cells. Weight
// each arc's per-cell contribution DOWN with range: full strength within
// MAP_ARC_REF_CM, then inverse-distance beyond, so near/tight arcs count for
// more than far/sprawling ones.
const MAP_ARC_REF_CM = 100;    // range at which an arc contributes weight 1.0
// A near beam barely smears (15° at 40cm is under one cell), so a single near
// arc is already well-localised and trustworthy — no need to wait for a second
// intersecting arc. Let close arcs earn up to this weight so anything within
// ~MAP_ARC_REF_CM/2 (~50cm) confirms in ONE look, while far arcs stay < 1.0 and
// must still intersect. This is what makes near obstacles register promptly.
const MAP_ARC_MAX_WEIGHT = 2;  // >= MAP_CONFIRM_HITS so a single near arc confirms
// A no-echo (or free ray) doesn't erase a cell outright — it FADES it a little,
// since repeated non-detections steadily raise the odds those points were noise.
const MAP_CLEAR_DECAY = 0.5;   // gentle weight erosion per free/no-echo pass
const mapCells = new Map();
const boatTrail = [];
// Dedupe identical arc observations (same beam, servo step, boat cell, range) so
// a stationary boat re-seeing the same arc every frame can't inflate a lone arc
// above the intersection threshold. Each DISTINCT look counts as exactly one arc.
const lastArcObs = new Map();

connectBtn.addEventListener("click", onConnectClick);
modeSwitch.addEventListener("change", onModeChange);
refreshPortsBtn.addEventListener("click", refreshPorts);
mockSwitch.addEventListener("change", onMockToggle);
mapClearBtn.addEventListener("click", clearRadarMap);
motorLeftAbsInput.addEventListener("change", () => sendMotorConfig("motorLeftAbs", motorLeftAbsInput));
motorRightAbsInput.addEventListener("change", () => sendMotorConfig("motorRightAbs", motorRightAbsInput));
if (downloadNavBtn) {
  downloadNavBtn.addEventListener("click", () => {
    if (window.LocalBridge) LocalBridge.downloadNavLog();
  });
}

if (replayRefreshBtn) replayRefreshBtn.addEventListener("click", loadReplayLogList);
if (replayPlayBtn) {
  replayPlayBtn.addEventListener("click", () => startReplay(replaySelect ? replaySelect.value : ""));
}
if (replayStopBtn) {
  replayStopBtn.addEventListener("click", () => {
    stopReplay();
    setServerMessage("ניגון הופסק.", false);
  });
}

// SERVERLESS = the page is served as a static site (GitHub Pages / opened
// directly) with no control_server.py behind it. In that mode every /api/*
// call and the telemetry stream are served locally by LocalBridge, which talks
// to the shore Arduino over the Web Serial API (USB-OTG on Android Chrome).
// Declared BEFORE loadReplayLogList() runs so that call doesn't read it in the
// temporal dead zone (was throwing a ReferenceError at load).
let SERVERLESS = false;

setupJoystick();
setupWinchJoystick();
initTransport();
updateReplayUI();
loadReplayLogList();
requestAnimationFrame(drawRadar);
requestAnimationFrame(drawWorld);
requestAnimationFrame(drawMap);

async function detectServerMode() {
  try {
    const r = await fetch("/api/state", { cache: "no-store" });
    if (r.ok) {
      SERVERLESS = false;
      return;
    }
  } catch (e) {
    /* no server reachable -> fall through to serverless */
  }
  SERVERLESS = true;
  // The autonomous nav log is written to disk by the server; without one, offer
  // a client-side download of the in-memory NDJSON instead.
  if (downloadNavBtn) downloadNavBtn.hidden = false;
  if (!("serial" in navigator)) {
    setServerMessage(
      "הדפדפן הזה לא תומך ב-Web Serial. השתמש ב-Chrome באנדרואיד, או הדלק 'מוק דאטא' לבדיקה.",
      true
    );
  } else {
    setServerMessage(
      "מצב ללא-שרת: חבר את ארדואינו החוף בכבל ולחץ 'חבר לבקר'. אפשר גם להדליק 'מוק דאטא'.",
      false
    );
  }
}

async function initTransport() {
  await detectServerMode();
  startTelemetryStream();
  refreshPorts();
}

async function onConnectClick() {
  if (state.mockEnabled) {
    setServerMessage("מצב מוק דאטא פעיל. כבה מוק כדי להתחבר לבקר אמיתי.", false);
    return;
  }

  if (SERVERLESS) {
    // Web Serial: connect via the browser device chooser. requestPort must run
    // inside this click gesture, so keep the connect path free of prior awaits.
    if (state.connected) {
      LocalBridge.disconnect();
      applyRemoteState(LocalBridge.snapshot());
      return;
    }
    try {
      const snap = await LocalBridge.connectViaPrompt();
      applyRemoteState(snap, true);
      setServerMessage("מחובר למכשיר USB.", false);
    } catch (err) {
      setServerMessage(`חיבור נכשל: ${err.message}`, true);
    }
    return;
  }

  if (state.connected) {
    await postJson("/api/disconnect", {});
    applyRemoteState(await fetchState());
    return;
  }

  if (!portSelect.value) {
    await refreshPorts();
  }
  if (!portSelect.value) {
    setServerMessage("לא נמצא פורט COM זמין. ודא שהארדואינו מחובר למחשב.", true);
    return;
  }

  try {
    const response = await postJson("/api/connect", {
      port: portSelect.value,
      baudRate: 115200,
    });
    applyRemoteState(response.state, true);
    setServerMessage(`מחובר ל-${response.state.serialPort}.`, false);
  } catch (err) {
    setServerMessage(`חיבור נכשל: ${err.message}`, true);
  }
}

async function onMockToggle() {
  try {
    const response = await postJson("/api/mock", { enabled: mockSwitch.checked });
    applyRemoteState(response.state, true);
  } catch (err) {
    mockSwitch.checked = !mockSwitch.checked;
    setServerMessage(`החלפת מצב מוק נכשלה: ${err.message}`, true);
  }
}

// Push one motor's absolute output speed (0-100) to the server. The value is
// clamped client-side too so a stray/empty box can't send garbage; the server
// applies it to every subsequent serial packet (no boat reflash).
async function sendMotorConfig(key, input) {
  let value = Math.round(Number(input.value));
  if (!Number.isFinite(value)) return;
  value = Math.max(0, Math.min(100, value));
  input.value = String(value);
  try {
    const response = await postJson("/api/motorconfig", { [key]: value });
    applyRemoteState(response.state);
  } catch (err) {
    setServerMessage(`עדכון מהירות מנוע נכשל: ${err.message}`, true);
  }
}

// Reflect the server's per-motor absolute speeds in the text boxes, but never
// overwrite a box the operator is actively editing (focused).
function updateMotorConfigUI(remoteState) {
  // Keep the calibration magnitudes in state so the pose estimator can mirror
  // the real boat's speed/maneuvering (motors are 3-state; magnitude = config).
  if (remoteState.motorLeftAbs != null) state.motorLeftAbs = remoteState.motorLeftAbs;
  if (remoteState.motorRightAbs != null) state.motorRightAbs = remoteState.motorRightAbs;
  if (document.activeElement !== motorLeftAbsInput && remoteState.motorLeftAbs != null) {
    motorLeftAbsInput.value = String(remoteState.motorLeftAbs);
  }
  if (document.activeElement !== motorRightAbsInput && remoteState.motorRightAbs != null) {
    motorRightAbsInput.value = String(remoteState.motorRightAbs);
  }
}

function startPolling() {
  if (state.pollId) {
    clearInterval(state.pollId);
  }
  state.pollId = setInterval(async () => {
    try {
      applyRemoteState(await fetchState());
    } catch (err) {
      setServerMessage(`שגיאת שרת: ${err.message}`, true);
      setConnectedUI(false, "");
    }
  }, POLL_INTERVAL_MS);
}

// Prefer a Server-Sent Events push stream: the backend emits a fresh snapshot
// the instant new telemetry is parsed, so the UI reflects the boat in near real
// time instead of lagging behind a 500ms poll. Falls back to polling if the
// browser lacks EventSource or the stream endpoint can't be reached.
function startTelemetryStream() {
  if (SERVERLESS) {
    // No SSE endpoint in serverless mode; subscribe to the local bridge, which
    // pushes a fresh snapshot on every telemetry/command/state change.
    LocalBridge.subscribe((snap) => applyRemoteState(snap));
    return;
  }

  if (typeof window.EventSource === "undefined") {
    startPolling();
    return;
  }

  if (state.eventSource) {
    state.eventSource.close();
    state.eventSource = null;
  }

  try {
    const es = new EventSource("/api/stream");
    state.eventSource = es;

    es.onmessage = (event) => {
      // A live stream frame means the poll fallback (if any) is redundant.
      if (state.pollId) {
        clearInterval(state.pollId);
        state.pollId = null;
      }
      try {
        const payload = JSON.parse(event.data);
        if (payload && payload.state) {
          applyRemoteState(payload.state);
        }
      } catch (err) {
        // Ignore malformed frames; the next one will resync the UI.
      }
    };

    es.onerror = () => {
      // EventSource reconnects on its own; keep a slow poll running meanwhile
      // so the UI still updates if the stream stays down (e.g. server restart).
      if (!state.pollId) {
        startPolling();
      }
    };
  } catch (err) {
    startPolling();
  }
}

function startCommandLoop() {
  if (state.commandLoopId) {
    clearInterval(state.commandLoopId);
  }

  state.commandLoopId = setInterval(async () => {
    if ((!state.connected && !state.mockEnabled) || state.sendingCommand) {
      return;
    }

    let outgoing;
    if (state.replay.active) {
      // ניגון חוזר: הפקודות נלקחות מהמסלול המוקלט לפי התזמון המקורי (ולא
      // מהג'ויסטיק/הניווט). כך הסירה מבצעת פיזית מחדש את הנסיעה שהוקלטה.
      outgoing = replayStep();
    } else {
      if (!state.manualMode) {
        updateAutonomousCommand();
      }
      // Advance the sweep once every RADAR_STEP_EVERY_TICKS *sent* commands
      // rather than by wall-clock. The old Date.now() gate fired on whichever
      // loop tick first crossed the threshold; because the loop skips ticks
      // while a previous send is still in flight (state.sendingCommand) and
      // 300ms/400ms don't divide evenly, the step landed on uneven 300/600ms+
      // gaps — the visible jump in rotation. Counting actual sends keeps the
      // angular rate constant and locked to the data cadence.
      state.radarStepTick += 1;
      if (state.radarStepTick >= RADAR_STEP_EVERY_TICKS) {
        advanceRadarSweep();
        state.radarStepTick = 0;
      }

      // === יירוט ודריסת פקודות היגוי לצורך בטיחות ===
      // הדריסה מחושבת על עותק בלבד, כך שברגע שיוצאים מהטווח הכתום
      // השליטה חוזרת מיד לניווט האוטונומי/השליטה הידנית.
      outgoing = computeSafeCommand();
    }
    // מצב ההפעלה (0=ידני, 1=אוטומטי) נשלח עם כל פקודה; הסירה מציגה צבע לפיו
    // ומעדכנת את הלד רק כשהערך משתנה (כך לא משבשים את קליטת ה-RF). בניגון
    // חוזר שולחים 0 (ידני) כדי שהסירה לא תריץ ניווט-על עצמאי מעל המסלול.
    outgoing.mode = (state.manualMode || state.replay.active) ? 0 : 1;

    // הקלטה רציפה: כל עוד מחוברים (ולא מנגנים מסלול קיים) רושמים כל טיק — ידני
    // או אוטונומי — כדי שאפשר יהיה לנתח ולנגן מחדש את כל הנסיעה. סשן חדש (קובץ
    // חדש) נפתח אוטומטית בתחילת ההקלטה, ונסגר (flush) כשמתנתקים/מתחילים ניגון.
    const shouldRecord = (state.connected || state.mockEnabled) && !state.replay.active;
    if (shouldRecord) {
      if (!navSessionStarted) {
        startNavLogSession();
        navSessionStarted = true;
      }
      recordNavTick(outgoing);
    } else if (navSessionStarted) {
      flushNavLog();
      navSessionStarted = false;
    }

    // אודומטריה: מקדמים את אומדן הפוזה לפי הפקודה הנשלחת, כך שהתפיסה+הניווט
    // רצים על פוזה משוערת (ריאל-טרנספרבילית) ולא על אמת-שרת שקיימת רק בסימולטור.
    const poseNow = performance.now();
    integratePose(outgoing, state.lastPoseT ? (poseNow - state.lastPoseT) / 1000 : 0);
    state.lastPoseT = poseNow;

    state.sendingCommand = true;
    try {
      const response = await postJson("/api/command", outgoing);
      applyRemoteState(response.state);
    } catch (err) {
      setServerMessage(`שליחת פקודה נכשלה: ${err.message}`, true);
    } finally {
      state.sendingCommand = false;
    }  }, CONTROL_INTERVAL_MS);
}

function stopCommandLoop() {
  if (state.commandLoopId) {
    clearInterval(state.commandLoopId);
    state.commandLoopId = null;
  }
}

// --- Drive recorder (teach-and-repeat) --------------------------------------
// Captures, once per control tick, what the boat SENT (telemetry/sensors), what
// it RECEIVES (motor + winch command), and — in autonomous mode — what the
// navigator perceived/decided. Recording runs CONTINUOUSLY while connected (in
// BOTH manual and autonomous mode) so a whole sail can be analysed AND played
// back later. Records are batched to /api/navlog so the run is saved to disk
// even if the phone/browser closes. Nothing is logged while idle/disconnected
// or while a previously-recorded route is being replayed.
const navLogPending = [];
let navLogFlushing = false;
let navLogResetPending = false;
// True once a recording session (one log file) has been opened for the current
// connection; reset when we disconnect or start a replay so the next connection
// rotates to a fresh file.
let navSessionStarted = false;

function startNavLogSession() {
  // Rotate to a fresh log file on the next flush (new run = new file).
  navLogPending.length = 0;
  navLogResetPending = true;
  flushNavLog();
}

function recordNavTick(outgoing) {
  const t = state.telemetry || {};
  const fc = liveCone(0, RW_FRONT_CONE_DEG);
  const lc = liveCone(-RW_SIDE_BEARING_DEG, RW_SIDE_TOL_DEG);
  const rc = liveCone(RW_SIDE_BEARING_DEG, RW_SIDE_TOL_DEG);
  const esc = bestOpenBearing(-180, 180);
  navLogPending.push({
    t: Date.now(),
    mock: state.mockEnabled,
    manual: state.manualMode,
    mode: state.manualMode ? "manual" : "auto",
    phase: state.nav.rwPhase,
    bowOffsetDeg: Math.round((state.nav.bowOffsetDeg ?? 0) * 10) / 10,
    bowLocked: !!state.nav.bowLocked,
    avoidDir: state.avoidDir | 0,
    headingDeg: Math.round(state.pose.headingDeg || 0),
    // What the boat SENT this frame (raw sensors + servo angle it echoed).
    tel: {
      usFront: t.usFront ?? null,
      usRight: t.usRight ?? null,
      usRadar: t.usRadar ?? null,
      usLeft: t.usLeft ?? null,
      radarAngle: t.radarAngle ?? null,
      boatHeadingDeg: t.boatHeadingDeg ?? null,
      stale: t.stale ?? null,
    },
    // What the boat RECEIVES (the motor/winch/servo command we send). This is
    // the exact stream replayed to physically re-drive the recorded route.
    cmd: {
      left: outgoing.leftSpeed,
      right: outgoing.rightSpeed,
      winch: outgoing.winchSpeed ?? 0,
      radarAngle: outgoing.radarAngle,
    },
    // What the navigator PERCEIVED (bow-relative, 1 m-capped) and its decision.
    perc: {
      front: fc,
      left: lc,
      right: rc,
      bestBearing: esc.bearing,
      bestClear: esc.clear,
      arcCount: esc.count,
    },
  });
  if (navLogPending.length >= 8) flushNavLog();
}

function flushNavLog() {
  if (navLogFlushing) return;
  if (navLogPending.length === 0 && !navLogResetPending) return;
  const batch = navLogPending.splice(0, navLogPending.length);
  const reset = navLogResetPending;
  navLogResetPending = false;
  navLogFlushing = true;
  postJson("/api/navlog", { records: batch, reset })
    .catch(() => {
      // Re-queue on failure so nothing is lost; the reset marker too.
      for (let i = batch.length - 1; i >= 0; i--) navLogPending.unshift(batch[i]);
      if (reset) navLogResetPending = true;
    })
    .finally(() => {
      navLogFlushing = false;
    });
}

// Make sure the tail of a run reaches disk when the tab is hidden/closed.
window.addEventListener("beforeunload", flushNavLog);
window.addEventListener("visibilitychange", () => {
  if (document.visibilityState === "hidden") flushNavLog();
});


// --- Replay (teach-and-repeat playback) -------------------------------------
// Physically re-drive a recorded route: the command loop pulls the recorded
// motor/winch/servo commands from the log and re-sends them at their original
// relative timing, so the boat repeats the movements it was taught. This is the
// foundation for future autonomous route-following (the recorded path becomes a
// reference the navigator can track).
function replayZeroCmd() {
  return {
    leftSpeed: 0,
    rightSpeed: 0,
    winchSpeed: 0,
    radarAngle: state.cmd.radarAngle,
  };
}

// Fetch the list of recorded runs from the server and fill the replay picker.
async function loadReplayLogList() {
  if (SERVERLESS || !replaySelect) return;
  try {
    const data = await getJson("/api/logs");
    const logs = Array.isArray(data.logs) ? data.logs : [];
    const prev = replaySelect.value;
    replaySelect.innerHTML = "";
    if (!logs.length) {
      const o = document.createElement("option");
      o.value = "";
      o.textContent = "אין מסלולים מוקלטים";
      replaySelect.appendChild(o);
    } else {
      for (const lg of logs) {
        const o = document.createElement("option");
        o.value = lg.name;
        o.textContent = `${lg.name} · ${lg.records} רשומות`;
        replaySelect.appendChild(o);
      }
      if (prev && logs.some((l) => l.name === prev)) replaySelect.value = prev;
    }
  } catch (err) {
    // Non-fatal: leave the picker as-is.
  }
}

// Load a recorded run and begin physically replaying its command stream.
async function startReplay(name) {
  if (!name) {
    setServerMessage("בחר מסלול מוקלט לניגון.", true);
    return;
  }
  if (SERVERLESS) {
    setServerMessage("ניגון מסלול זמין רק דרך שרת השליטה המקומי.", true);
    return;
  }
  if (state.replay.active) stopReplay();

  let text;
  try {
    const resp = await fetch(`/api/logs/${encodeURIComponent(name)}`, { cache: "no-store" });
    if (!resp.ok) throw new Error(`HTTP ${resp.status}`);
    text = await resp.text();
  } catch (err) {
    setServerMessage(`טעינת המסלול נכשלה: ${err.message}`, true);
    return;
  }

  const records = [];
  for (const line of text.split("\n")) {
    const s = line.trim();
    if (!s) continue;
    try {
      const rec = JSON.parse(s);
      if (rec && rec.cmd && typeof rec.t === "number") records.push(rec);
    } catch (err) {
      // Skip a malformed line rather than aborting the whole route.
    }
  }
  if (records.length < 2) {
    setServerMessage("המסלול המוקלט ריק או קצר מדי לניגון.", true);
    return;
  }

  // Manual mode so neither the joystick nor the autonomous navigator fights the
  // replayed command stream.
  if (!state.manualMode) {
    modeSwitch.checked = false;
    onModeChange();
  }

  state.replay.active = true;
  state.replay.records = records;
  state.replay.index = 0;
  state.replay.baseT = records[0].t;
  state.replay.startWallT = performance.now();
  state.replay.name = name;
  state.replay.loop = replayLoopChk ? replayLoopChk.checked : false;
  updateReplayUI();
  setServerMessage(`מנגן מסלול: ${name} (${records.length} רשומות).`, false);
}

// Step the replay: return the command that matches the elapsed play time. Ends
// (or loops) once the last recorded frame's time has passed.
function replayStep() {
  const r = state.replay;
  const recs = r.records;
  if (!recs.length) {
    stopReplay();
    return replayZeroCmd();
  }
  const elapsed = performance.now() - r.startWallT;
  const endOffset = recs[recs.length - 1].t - r.baseT;

  // Advance to the latest record whose original offset has been reached.
  while (r.index < recs.length - 1 && recs[r.index + 1].t - r.baseT <= elapsed) {
    r.index += 1;
  }

  // Past the end of the route.
  if (elapsed > endOffset + CONTROL_INTERVAL_MS) {
    if (r.loop) {
      r.index = 0;
      r.startWallT = performance.now();
    } else {
      stopReplay();
      setServerMessage(`ניגון הסתיים: ${r.name}.`, false);
      return replayZeroCmd();
    }
  }

  const c = recs[r.index].cmd || {};
  updateReplayStatus(elapsed, endOffset);
  return {
    leftSpeed: c.left ?? 0,
    rightSpeed: c.right ?? 0,
    winchSpeed: c.winch ?? 0,
    radarAngle: c.radarAngle ?? state.cmd.radarAngle,
  };
}

function stopReplay() {
  if (!state.replay.active) return;
  state.replay.active = false;
  // Leave the boat stopped after playback.
  state.cmd.leftSpeed = 0;
  state.cmd.rightSpeed = 0;
  state.cmd.winchSpeed = 0;
  updateReplayUI();
}

function updateReplayStatus(elapsed, endOffset) {
  if (!replayStatus) return;
  const secs = (ms) => (Math.max(0, ms) / 1000).toFixed(1);
  const r = state.replay;
  replayStatus.textContent =
    `מנגן ${r.name} · ${r.index + 1}/${r.records.length} · ${secs(elapsed)}/${secs(endOffset)} ש'`;
}

function updateReplayUI() {
  const active = state.replay.active;
  if (replayPlayBtn) replayPlayBtn.disabled = active;
  if (replayStopBtn) replayStopBtn.disabled = !active;
  if (replaySelect) replaySelect.disabled = active;
  if (replayRefreshBtn) replayRefreshBtn.disabled = active;
  if (replayStatus && !active) replayStatus.textContent = "לא מנגן";
}


async function refreshPorts() {
  if (SERVERLESS) {
    // Web Serial has no COM-port enumeration; the device is chosen through the
    // browser's own picker when 'חבר לבקר' is pressed.
    portSelect.innerHTML = "";
    const option = document.createElement("option");
    option.value = "";
    option.textContent = "בחירה בלחיצה על 'חבר'";
    portSelect.appendChild(option);
    return;
  }

  try {
    const response = await getJson("/api/ports");
    const selected = portSelect.value;
    portSelect.innerHTML = "";

    for (const port of response.ports) {
      const option = document.createElement("option");
      option.value = port;
      option.textContent = port;
      portSelect.appendChild(option);
    }

    if (selected && response.ports.includes(selected)) {
      portSelect.value = selected;
    }

    if (!response.ports.length) {
      const option = document.createElement("option");
      option.value = "";
      option.textContent = "אין פורטים";
      portSelect.appendChild(option);
    }
  } catch (err) {
    setServerMessage(`רענון פורטים נכשל: ${err.message}`, true);
  }
}

async function fetchState() {
  const response = await getJson("/api/state");
  return response.state;
}

// Load the ground-truth mock world once per mock session. Targets are randomised
// each time mock mode is (re)enabled, so we refetch whenever mock turns on.
async function ensureWorldLoaded() {
  if (!state.mockEnabled) {
    world.loaded = false;
    world.targets = [];
    world.walls = [];
    world.start = null;
    world.goal = null;
    return;
  }
  if (world.loaded || world.loading) return;
  world.loading = true;
  try {
    const w = await getJson("/api/world");
    world.boundsHalfX = w.boundsHalfX ?? world.boundsHalfX;
    world.boundsHalfY = w.boundsHalfY ?? world.boundsHalfY;
    world.maxRange = w.maxRange ?? world.maxRange;
    world.targets = Array.isArray(w.targets) ? w.targets : [];
    world.walls = Array.isArray(w.walls) ? w.walls : [];
    world.scenario = w.scenario ?? null;
    world.start = w.start ?? null;
    world.goal = w.goal ?? null;
    world.loaded = true;
  } catch (err) {
    // Non-fatal: world view simply stays empty until the next attempt.
  } finally {
    world.loading = false;
  }
}

function applyRemoteState(remoteState, syncCmd = false) {
  const wasConnected = state.connected;

  state.connected = Boolean(remoteState.connected);
  state.mockEnabled = Boolean(remoteState.mockEnabled);
  state.serialPort = remoteState.serialPort || "";
  state.lastError = remoteState.lastError || "";

  // Only overwrite the outgoing command on explicit connect / mock-enable.
  // During normal operation the browser is the source of truth for state.cmd;
  // overwriting it from poll responses or command echoes would discard in-flight
  // user input (e.g. winch button pressed between a poll and the 80ms command tick).
  if (syncCmd) {
    state.cmd = {
      leftSpeed: remoteState.command.leftSpeed,
      rightSpeed: remoteState.command.rightSpeed,
      winchSpeed: remoteState.command.winchSpeed,
      radarAngle: remoteState.command.radarAngle,
    };
    // Fresh connect / mock-enable => start the accumulated map from scratch so
    // stale detections from a previous (possibly re-randomised) world are gone.
    clearRadarMap();
    resetPose();
    resetSensorHistory();
  }

  // Always use backend telemetry (both real and mock modes). Each ultrasonic
  // channel is despeckled with a 3-sample median first, so a lone dropout (999)
  // or phantom short spike from the hardware never reaches the nav/map/UI.
  state.telemetry = {
    usRadar: filterSensorReading("usRadar", remoteState.telemetry.usRadar),
    usFront: filterSensorReading("usFront", remoteState.telemetry.usFront),
    usLeft: filterSensorReading("usLeft", remoteState.telemetry.usLeft),
    usRight: filterSensorReading("usRight", remoteState.telemetry.usRight),
    radarAngle: remoteState.telemetry.radarAngle,
    boatX: remoteState.telemetry.boatX ?? 0,
    boatY: remoteState.telemetry.boatY ?? 0,
    boatHeadingDeg: remoteState.telemetry.boatHeadingDeg ?? 0,
  };
  // The sweep source depends on mode: on REAL hardware the boat drives the servo
  // autonomously and reports its actual angle in telemetry (authoritative); in
  // MOCK the PC drives the sweep so the commanded angle is freshest.
  state.lastRadarAngleSent = sweepAngle();

  if (syncCmd) {
    // תרחישי מוק יכולים להתחיל את הסירה בכל מקום (למשל פינה). מזריעים את מסגרת
    // האודומטריה מפוזת-ההתחלה האמיתית של המוק, כדי שהמפה, הקירות והיעד כולם יחיו
    // באותה מסגרת קואורדינטות עקבית. (חומרה אמיתית מדווחת 0,0 — אין שינוי שם.)
    state.pose.x = state.telemetry.boatX ?? 0;
    state.pose.y = state.telemetry.boatY ?? 0;
    state.pose.headingDeg = state.telemetry.boatHeadingDeg ?? 0;
    state.pose.speedCms = 0;
    state.pose.turnRateRadS = 0;
    state.lastPoseT = 0;
  }

  if (state.mockEnabled || state.connected) {
    updateRadarMemory();
    accumulateRadarMap();
    updateLiveScan();
    // חשוב לסדר: קודם updateLiveScan ממלא את liveScan מהקריאות הגלמיות, ורק אז
    // updateRadarWalls דורס את הבינים שנפלו על קו-קיר בטווח המיושר. כך הניווט
    // האוטונומי (liveCone) באמת מסתמך על האינטרפולציה, לא רק הצג.
    updateRadarWalls();
    // ואז מכיילים מול הידע הפיזי על המסלול: מסלקים "מסדרונות" צרים מ-MIN_CORRIDOR_CM
    // שלא ייתכנו במציאות (הד/רפאים) מתוך סריקת-הניווט, אחרי שהיא כבר הורכבה במלואה.
    applyMinCorridorPrior();
    // ובאותה רוח: המסלול לעולם אינו "קופסה סגורה" — תמיד יש צד פתוח. אם כל ארבעת
    // הכיוונים נראים חסומים, פותחים את החלש (הרחוק) שבהם כדי שתמיד יהיה מוצא.
    applyNoClosedBoxPrior();
  } else {
    // Comms lost / not connected: keep the LAST radar picture (radarMemory +
    // radarWalls) on screen instead of wiping it, so the operator still sees
    // what was there before. radarClock freezes its ageing so it won't fade.
    // Only the nav-facing liveScan is cleared, so autonomy never acts on frozen
    // data as if it were a live, confirmed reading.
    liveScan.clear();
  }

  ensureWorldLoaded();

  updateTelemetryUI();
  updateCommandUI();
  setConnectedUI(state.connected, state.serialPort);
  updateMotorConfigUI(remoteState);

  mockSwitch.checked = state.mockEnabled;
  portSelect.disabled = state.mockEnabled || state.connected;
  refreshPortsBtn.disabled = state.mockEnabled || state.connected;

  // Show the mock world picture and the accumulated map only in mock mode.
  // (Use style.display, not [hidden]: .panel sets display:flex which would win.)
  if (worldPanel) worldPanel.style.display = state.mockEnabled ? "" : "none";
  if (mapPanel) mapPanel.style.display = state.mockEnabled ? "" : "none";

  if (state.lastError) {
    setServerMessage(state.lastError, true);
  } else if (state.mockEnabled) {
    setServerMessage("מוק דאטא: 4 חיישנים על סרבו מסתובב (0-90°), גופים אקראיים, תנועה לפי ג'ויסטיק", false);
  } else if (state.connected) {
    setServerMessage(`מחובר ל-${state.serialPort}.`, false);
  } else if (wasConnected && !state.connected) {
    setServerMessage("החיבור לבקר נותק.", true);
  }

  if ((state.connected || state.mockEnabled) && !state.commandLoopId) {
    startCommandLoop();
  }
  if (!state.connected && !state.mockEnabled) {
    stopCommandLoop();
  }
}

function setConnectedUI(connected, portName) {
  const active = connected || state.mockEnabled;
  connectBtn.disabled = state.mockEnabled;
  connectBtn.textContent = connected ? "נתק" : "חבר לבקר";
  connectionState.textContent = state.mockEnabled ? "מוק דאטא" : connected ? `מחובר ${portName}` : "לא מחובר";
  connectionState.classList.toggle("connected", active);
  connectionState.classList.toggle("disconnected", !active);
}

function updateTelemetryUI() {
  radarAngleValue.textContent = String(Math.round(state.lastRadarAngleSent));
  radarDistanceValue.textContent = formatDistance(state.telemetry.usFront, true);
}

function formatDistance(value, outLabel = false) {
  if (value === null || value === undefined) {
    return "--";
  }
  if (value === 999) {
    return outLabel ? "OUT" : "999";
  }
  return String(Math.round(value));
}

function onModeChange() {
  state.manualMode = !modeSwitch.checked;
  state.avoidDir = 0;
  // מאתחלים מחדש את היעד עם כניסה מחודשת למצב אוטונומי (נקבע מהמיקום העדכני).
  state.nav.goal = null;
  state.nav.mode = "seek";
  state.nav.progressAnchor = null;
  state.nav.moveAnchor = null;
  state.nav.escapeUntil = 0;
  resetJoystick();
  if (!state.manualMode) {
    state.cmd.leftSpeed = 0;
    state.cmd.rightSpeed = 0;
    // Real-world autonomous restarts with a fresh online bow estimate + scan.
    if (!state.mockEnabled) {
      state.nav.bowOffsetDeg = BOW_SERVO_OFFSET_DEG;
      state.nav.bowLocked = BOW_FIXED;
      state.nav.rwPhase = "calib";
      state.nav.calibActive = true;
      state.nav.calibStart = performance.now();
      state.nav.spinUntil = 0;
      state.nav.spinDir = 0;
      state.nav.settleUntil = 0;
      state.nav.wallEscapeUntil = 0;
      state.nav.wallEscapeSide = 0;
      state.nav.reverseUntil = 0;
      state.nav.reverseArcDir = 0;
      state.nav.sideUntil = 0;
      state.nav.sideDir = 0;
      state.nav.fwdUntil = 0;
      // Coarse travel direction: heading 0 = "the way the bow points now", which
      // the operator aims at the goal/channel entrance before arming autonomy.
      state.pose.x = 0;
      state.pose.y = 0;
      state.pose.headingDeg = 0;
      state.pose.turnRateRadS = 0;
      state.lastPoseT = 0;
      liveScan.clear();
      rawPrev.clear();
      bowEstSin = 0;
      bowEstCos = 0;
      bowEstWeight = 0;
      bowCandidateDeg = null;
      bowStableHits = 0;
      resetRadarWalls();
    }
    // ההקלטה רציפה לאורך כל החיבור (ידני+אוטונומי), כך שהחלפת מצב לא מפצלת
    // את הקובץ. לא פותחים סשן חדש כאן — הלולאה מנהלת אותו לפי מצב החיבור.
  } else {
    // Autonomy disarmed: push the tail of the run to disk.
    flushNavLog();
  }
  updateCommandUI();
}

function setupJoystick() {
  const radius = () => joystickBase.clientWidth / 2;

  const onPointerMove = (event) => {
    if (!state.joystick.active || !state.manualMode) {
      return;
    }
    const rect = joystickBase.getBoundingClientRect();
    const centerX = rect.left + rect.width / 2;
    const centerY = rect.top + rect.height / 2;
    const dx = event.clientX - centerX;
    const dy = event.clientY - centerY;

    const max = radius() - joystickStick.clientWidth / 2;
    const len = Math.hypot(dx, dy);
    const scale = len > max ? max / len : 1;
    const x = dx * scale;
    const y = dy * scale;

    state.joystick.x = x / max;
    state.joystick.y = y / max;

    joystickStick.style.transform = `translate(calc(-50% + ${x}px), calc(-50% + ${y}px))`;
    updateManualCommandFromJoystick();
  };

  joystickBase.addEventListener("pointerdown", (event) => {
    if (!state.manualMode) {
      return;
    }
    state.joystick.active = true;
    joystickBase.setPointerCapture(event.pointerId);
    onPointerMove(event);
  });

  joystickBase.addEventListener("pointermove", onPointerMove);

  const stop = (event) => {
    if (!state.joystick.active) {
      return;
    }
    state.joystick.active = false;
    if (event?.pointerId !== undefined) {
      joystickBase.releasePointerCapture(event.pointerId);
    }
    resetJoystick();
  };

  joystickBase.addEventListener("pointerup", stop);
  joystickBase.addEventListener("pointercancel", stop);
}

function updateManualCommandFromJoystick() {
  const forward = -state.joystick.y;
  const turn = state.joystick.x;

  // מגבילים את פלט הג'ויסטיק למדרגי המנוע (0 / 70 / 100, עם סימן), כך שהפקודה
  // המוצגת והנשלחת תואמת בדיוק את מה שהמנוע יריץ בפועל.
  state.cmd.leftSpeed = shapeMotorSpeed(Math.round((forward + turn) * 255));
  state.cmd.rightSpeed = shapeMotorSpeed(Math.round((forward - turn) * 255));
  updateCommandUI();
}

function resetJoystick() {
  state.joystick.x = 0;
  state.joystick.y = 0;
  joystickStick.style.transform = "translate(-50%, -50%)";
  if (state.manualMode) {
    state.cmd.leftSpeed = 0;
    state.cmd.rightSpeed = 0;
    // לא שולחים postJson כאן — לולאת הפקודות המחזורית (startCommandLoop) ממילא
    // דוחפת את ה-cmd המעודכן לשרת כל 80ms, כך שאין טעם בקריאת רשת נוספת בשחרור.
  }
  updateCommandUI();
}

function setupWinchJoystick() {
  let active = false;

  const getClampedDy = (e) => {
    const rect = winchJoystickBase.getBoundingClientRect();
    const halfTravel = rect.height / 2 - 30; // 30 = half handle size
    const dy = e.clientY - (rect.top + rect.height / 2);
    return Math.max(-halfTravel, Math.min(halfTravel, dy));
  };

  const update = (dy) => {
    const rect = winchJoystickBase.getBoundingClientRect();
    const halfTravel = rect.height / 2 - 30;
    // Drag up (negative dy) = pull net (+speed); drag down = release (-speed)
    const speed = Math.round((-dy / halfTravel) * 255);
    state.cmd.winchSpeed = Math.max(-255, Math.min(255, speed));
    winchJoystickHandle.style.transform = `translate(-50%, calc(-50% + ${dy}px))`;
    winchSpeedValue.textContent = String(state.cmd.winchSpeed);
  };

  // בשחרור הידית נשארת במקומה והמהירות נשמרת (הכננת אינה חוזרת לאפס מעצמה),
  // כך שאפשר "לנעול" משיכה/שחרור מתמשך בלי להחזיק את הסמן.
  const stop = () => {
    if (!active) return;
    active = false;
  };

  winchJoystickBase.addEventListener("pointerdown", (e) => {
    active = true;
    winchJoystickBase.setPointerCapture(e.pointerId);
    update(getClampedDy(e));
  });
  winchJoystickBase.addEventListener("pointermove", (e) => {
    if (!active) return;
    update(getClampedDy(e));
  });
  winchJoystickBase.addEventListener("pointerup", stop);
  winchJoystickBase.addEventListener("pointercancel", stop);
}


// The 4 sensors sit 90° apart on ONE servo, so sweeping 15..90° already paints
// the full 360° picture. The PC drives the servo so it climbs to 90°, then
// walks gently back down one 15° step at a time to 0° (instead of slamming
// straight from 90° to 0° in a single move, which would stress the servo
// gears). At 0° it resumes climbing. The sweep MUST reach 0° so the four
// sensors (mounted at 0/90/180/270°) actually sample the cardinal directions —
// dead ahead most of all. Stopping at 15° left a permanent blind spot at the
// bow, so head-on obstacles went unseen until the boat had already hit them.
function advanceRadarSweep() {
  const step = 15;
  let next = state.cmd.radarAngle + step * state.radarSweepDir;
  if (next >= 90) {
    next = 90;
    state.radarSweepDir = -1;
  } else if (next <= 0) {
    next = 0;
    state.radarSweepDir = 1;
  }
  state.cmd.radarAngle = next;
}

// Effective servo sweep angle used for all rendering/mapping. On REAL hardware
// the boat now drives the radar sweep autonomously (firmware-local timer) and
// echoes its actual angle in telemetry, so that echoed angle is the source of
// truth. In MOCK the PC drives the sweep, so the commanded angle is freshest
// (telemetry echoes it a frame later).
function sweepAngle() {
  if (state.mockEnabled) return state.cmd.radarAngle ?? state.telemetry.radarAngle ?? 0;
  return state.telemetry.radarAngle ?? state.cmd.radarAngle ?? 0;
}

// Returns the nearest obstacle distance (cm) within toleranceDeg of a given
// bow-relative bearing, taken from the motion-compensated accumulated world map
// (confirmed cells) plus fresh live radar. The map is keyed in fixed world
// coordinates, so a clearance stays correct as the boat closes in; the raw
// per-slot radar range alone goes stale between the slow servo sweeps, which is
// what made the navigator charge into walls it "remembered" as far away.
function getMemoryDistance(bearingDeg, toleranceDeg) {
  const bx = state.pose.x;
  const by = state.pose.y;
  const heading = state.pose.headingDeg;
  const targetAbs = normalizeDeg(heading + bearingDeg);
  let nearest = 999;

  const consider = (px, py) => {
    const d = Math.hypot(px - bx, py - by);
    if (d >= nearest) return;
    const absBearing = (Math.atan2(px - bx, py - by) * 180) / Math.PI;
    if (absAngleDiffDeg(absBearing, targetAbs) <= toleranceDeg) nearest = d;
  };

  // Confirmed obstacles in the accumulated world map (arcs from several bearings
  // intersected here), positioned in world space so the range is never stale.
  for (const cell of mapCells.values()) {
    if (getCellConfidence(cell) >= MAP_CONFIRM_HITS) consider(cell.x, cell.y);
  }

  // Fresh live radar too, so a newly seen obstacle triggers avoidance before the
  // map has had two sweeps to confirm it. Use the world point frozen at measure
  // time (entry.wx/wy) — NOT the stale range re-projected from the current pose,
  // which would drag the obstacle along with the boat.
  const now = performance.now();
  for (const [absSlot, entry] of radarMemory) {
    if (entry.filled) continue; // interpolated gap-fill = display only, never nav
    if (!entry.value || entry.value >= 999 || now - entry.t > RADAR_TTL_MS) continue;
    consider(entry.wx, entry.wy);
  }

  // ניפוח מכשולים: מקזזים את רדיוס-הגוף+מרווח כדי שמרחק-הפנוי שהניווט מקבל ישקף
  // את מעטפת הסירה ולא נקודה. כשלא נמצא מכשול (999) לא מנפחים — נשאר "פתוח".
  if (nearest >= 999) return nearest;
  return Math.max(0, nearest - OBSTACLE_INFLATION_CM);
}

// dir < 0 turns left (heading falls), dir > 0 turns right (heading rises).
// turn = (right - left) in the sim, so right > left => turn right.
function applyAvoidTurn(dir) {
  state.cmd.leftSpeed = -dir * AVOID_SPEED;
  state.cmd.rightSpeed = dir * AVOID_SPEED;
}

// מקדם את אומדן הפוזה לפי הפקודה שאנו עומדים לשלוח (דווקא מהפקודה, לא מטלמטריה)
// — בדיוק כמו שחומרה אמיתית תעשה. מודל זהה ל-_mock_loop בשרת.
function integratePose(cmd, dtSec) {
  // חשוב: אסור לחתוך זמן אמיתי. השרת (_mock_loop) מאנטגר כל 50ms ללא תקרה
  // עליונה, ולכן אם נחתוך dt קטן (0.2) בכל טיק-רשת איטי/לשונית ברקע — האומדן
  // "יפגר" בסיבוב 3-מצבית לכל מנוע: קדימה / אחורה / עצור. הג'ויסטיק (וגם הניווט)
  // קובע רק את הכיוון של כל מנוע; העוצמה נקבעת מהכיול (motorLeftAbs/motorRightAbs)
  // שנועד רק להשוות בין המנועים למהירות אפקטיבית שווה. לכן שני מנועים דולקים
  // נותנים דחף זהה, והתמרון נובע מהכיוונים בלבד:
  //   שניהם קדימה                 -> ישר
  //   אחד קדימה, השני עצור         -> קשת קדימה (פנייה תוך כדי תנועה)
  //   אחד קדימה, השני אחורה        -> סיבוב במקום
  const leftDir = Math.sign(shapeMotorSpeed(cmd.leftSpeed));
  const rightDir = Math.sign(shapeMotorSpeed(cmd.rightSpeed));
  // רמת הכוח האפקטיבית המשותפת (שבר מהסולם 0..255). שתי מהירויות הכיול מושוות
  // בפועל, לכן משתמשים בממוצע כעוצמת דחף אחידה לשני המנועים — זהה ל-_mock_loop.
  const motorLevel = ((state.motorLeftAbs ?? 84) + (state.motorRightAbs ?? 88)) / 2 / 255;
  const throttle = (leftDir + rightDir) / 2 * motorLevel;
  const turn = (rightDir - leftDir) / 2 * motorLevel;
  const thrust = throttle * DR_MAX_SPEED_CMS * DR_LINEAR_DRAG;
  const targetTurnRate = turn * DR_TURN_RATE;
  const HX = world.boundsHalfX || 600;
  const HY = world.boundsHalfY || 450;
  // תת-צעד ב-50ms בדיוק כמו השרת, כך שאינטגרציית אוילר של הלקוח מתקדמת צעד-בצעד
  // עם _mock_loop (אותה צבירת כיוון+מיקום) ולא נפער פער כיוון בטיקים ארוכים.
  const SERVER_STEP_SEC = 0.05;
  let dt = dtSec;
  while (dt > 1e-6) {
    const h = Math.min(SERVER_STEP_SEC, dt);
    // תנע קווי: דחף מנוגד לגרר מים לינארי — הכלי מחליק אחרי ניתוק המצערת.
    state.pose.speedCms += (thrust - DR_LINEAR_DRAG * state.pose.speedCms) * h;
    // תנע זוויתי: קצב הסחרור מתקרב בהדרגה לקצב המבוקש (וממשיך רגע אחרי שינוי
    // הפקודה) במקום להיצמד מיידית — כך סיבובים "מגלשים" כמו כלי אמיתי.
    state.pose.turnRateRadS += (targetTurnRate - state.pose.turnRateRadS) * DR_TURN_DRAG * h;
    const hr = degToRad(state.pose.headingDeg) + state.pose.turnRateRadS * h;
    state.pose.x = clamp(state.pose.x + Math.sin(hr) * state.pose.speedCms * h, -HX, HX);
    state.pose.y = clamp(state.pose.y + Math.cos(hr) * state.pose.speedCms * h, -HY, HY);
    state.pose.headingDeg = normalizeDeg((hr * 180) / Math.PI);
    dt -= h;
  }
}

// מאפס את מסגרת האודומטריה (עם איפוס העולם בסימולטור, כדי שהאומדן יישאר מיושר לאמת).
function resetPose() {
  state.pose.x = 0;
  state.pose.y = 0;
  state.pose.headingDeg = 0;
  state.pose.speedCms = 0;
  state.pose.turnRateRadS = 0;
  state.lastPoseT = 0;
}

// Distance (cm) from the boat to the rectangular world boundary along a world
// bearing. World coords use x = sin(bearing), y = cos(bearing) like the rest of
// the code. Lets the open-water seeker treat walls as "density" too, so it
// steers toward the open interior instead of driving into a corner.
function distanceToWall(bx, by, absBearingDeg, HX, HY) {
  const rad = degToRad(absBearingDeg);
  const dx = Math.sin(rad);
  const dy = Math.cos(rad);
  let t = Infinity;
  if (dx > 1e-6) t = Math.min(t, (HX - bx) / dx);
  else if (dx < -1e-6) t = Math.min(t, (-HX - bx) / dx);
  if (dy > 1e-6) t = Math.min(t, (HY - by) / dy);
  else if (dy < -1e-6) t = Math.min(t, (-HY - by) / dy);
  return t < 0 ? 0 : t;
}

// Proactive open-water seeking. Scans the forward arc and returns the bow-
// relative bearing with the greatest clearance (nearest body OR wall), lightly
// penalised for deviating from a REFERENCE bearing so the boat still makes
// progress. With refBearing = the goal direction the boat is pulled toward the
// least-dense direction that also advances toward the goal, instead of merely
// wandering into open water.
function chooseOpenHeading(refBearing = 0, penalty = OPEN_TURN_PENALTY) {
  const bx = state.pose.x;
  const by = state.pose.y;
  const heading = state.pose.headingDeg;
  const HX = world.boundsHalfX || 600;
  const HY = world.boundsHalfY || 450;

  let bestBearing = 0;
  let bestScore = -Infinity;
  for (let b = -OPEN_SCAN_DEG; b <= OPEN_SCAN_DEG; b += OPEN_SCAN_STEP) {
    const obstacle = getMemoryDistance(b, OPEN_SCAN_TOL);
    const wall = distanceToWall(bx, by, normalizeDeg(heading + b), HX, HY);
    const clear = Math.min(obstacle, wall, OPEN_CLEAR_CAP);
    const score = clear - penalty * Math.abs(b - refBearing);
    if (score > bestScore) {
      bestScore = score;
      bestBearing = b;
    }
  }
  return bestBearing;
}

// ---- \u05e0\u05d9\u05d5\u05d5\u05d8 \u05de\u05d5\u05db\u05d5\u05d5\u05df-\u05d9\u05e2\u05d3: \u05e2\u05d6\u05e8\u05d9\u05dd ----

// \u05e7\u05d5\u05d1\u05e2 \u05d0\u05ea \u05d4\u05d9\u05e2\u05d3 \u05e2\u05dd \u05db\u05e0\u05d9\u05e1\u05d4 \u05dc\u05de\u05e6\u05d1 \u05d0\u05d5\u05d8\u05d5\u05e0\u05d5\u05de\u05d9: \u05d4\u05d3\u05d5\u05e4\u05df \u05d4\u05e7\u05e8\u05d5\u05d1\u05d4 \u05d1\u05d9\u05d5\u05ea\u05e8 \u05d4\u05d9\u05d0 \u05d4\u05de\u05d5\u05e6\u05d0,
// \u05d5\u05d4\u05d9\u05e2\u05d3 \u05d4\u05d5\u05d0 \u05d4\u05d3\u05d5\u05e4\u05df \u05e9\u05de\u05de\u05d5\u05dc \u05d1\u05d0\u05d5\u05ea\u05d4 \u05e7\u05d5\u05d0\u05d5\u05e8\u05d3\u05d9\u05e0\u05d8\u05d4 \u05e6\u05d3\u05d3\u05d9\u05ea ("\u05d4\u05e7\u05e6\u05d4 \u05d4\u05e9\u05e0\u05d9").
function initNavGoal() {
  // תרחיש מוק עם יעד מפורש (למשל מסלול הסירפנטינה: פינה שמאלית-תחתונה -> ימנית-
  // עליונה) — משתמשים ישירות בנקודת היעד בעולם. אחרת: חוצים לדופן הנגדית.
  if (world.goal) {
    state.nav.goal = { x: world.goal.x, y: world.goal.y };
    state.nav.mode = "seek";
    state.nav.progressAnchor = null;
    state.nav.moveAnchor = null;
    state.nav.escapeUntil = 0;
    state.avoidDir = 0;
    return;
  }

  const bx = state.pose.x;
  const by = state.pose.y;
  const HX = world.boundsHalfX || 600;
  const HY = world.boundsHalfY || 450;
  const candidates = [
    { d: bx + HX, goal: { x:  HX - GOAL_MARGIN_CM, y: by } }, // \u05de\u05d5\u05e6\u05d0 \u05dc\u05d9\u05d3 \u05d3\u05d5\u05e4\u05df \u05e9\u05de\u05d0\u05dc -> \u05d9\u05e2\u05d3 \u05d9\u05de\u05d9\u05df
    { d: HX - bx, goal: { x: -HX + GOAL_MARGIN_CM, y: by } }, // \u05de\u05d5\u05e6\u05d0 \u05dc\u05d9\u05d3 \u05d3\u05d5\u05e4\u05df \u05d9\u05de\u05d9\u05df -> \u05d9\u05e2\u05d3 \u05e9\u05de\u05d0\u05dc
    { d: by + HY, goal: { x: bx, y:  HY - GOAL_MARGIN_CM } }, // \u05de\u05d5\u05e6\u05d0 \u05dc\u05d9\u05d3 \u05d3\u05d5\u05e4\u05df \u05ea\u05d7\u05ea\u05d5\u05e0\u05d4 -> \u05d9\u05e2\u05d3 \u05e2\u05dc\u05d9\u05d5\u05df
    { d: HY - by, goal: { x: bx, y: -HY + GOAL_MARGIN_CM } }, // \u05de\u05d5\u05e6\u05d0 \u05dc\u05d9\u05d3 \u05d3\u05d5\u05e4\u05df \u05e2\u05dc\u05d9\u05d5\u05e0\u05d4 -> \u05d9\u05e2\u05d3 \u05ea\u05d7\u05ea\u05d5\u05df
  ];
  candidates.sort((a, b) => a.d - b.d);
  state.nav.goal = candidates[0].goal;
  state.nav.mode = "seek";
  state.nav.progressAnchor = null;
  state.nav.moveAnchor = null;
  state.nav.escapeUntil = 0;
  state.avoidDir = 0;
}

// \u05de\u05e8\u05d7\u05e7 \u05e0\u05d5\u05db\u05d7\u05d9 \u05de\u05d4\u05e1\u05d9\u05e8\u05d4 \u05dc\u05d9\u05e2\u05d3 (\u05e1"\u05de).
function goalDistance() {
  const bx = state.pose.x;
  const by = state.pose.y;
  const g = state.nav.goal;
  return Math.hypot(g.x - bx, g.y - by);
}

// \u05d4\u05d0\u05d6\u05d9\u05de\u05d5\u05ea \u05d4\u05d9\u05d7\u05e1\u05d9 \u05dc\u05d7\u05e8\u05d8\u05d5\u05dd (\u2011180..180, 0 = \u05d9\u05e9\u05e8 \u05e7\u05d3\u05d9\u05de\u05d4) \u05d0\u05dc \u05d4\u05d9\u05e2\u05d3.
function goalBearingRel() {
  const bx = state.pose.x;
  const by = state.pose.y;
  const heading = state.pose.headingDeg;
  const g = state.nav.goal;
  const absBearing = (Math.atan2(g.x - bx, g.y - by) * 180) / Math.PI;
  return ((absBearing - heading + 540) % 360) - 180;
}

// אזימוט-כיוון-שיוט יחסי לחרטום: כמו goalBearingRel אך אל נקודת-כיוון שמצמצמת את
// הרכיב הצדדי (x) של היעד כשהסירה עדיין רחוקה ממנו אנכית. כך הסירה מטפסת כמעט-
// אנכית דרך הפערים במקום להימשך אל דופן-הצד של פינת-היעד, וממקדת אל היעד עצמו רק
// כשהיא כבר סמוכה אליו אנכית.
function aimBearingRel() {
  const bx = state.pose.x;
  const by = state.pose.y;
  const heading = state.pose.headingDeg;
  const g = state.nav.goal;
  const dy = g.y - by;
  const latK = clamp(GOAL_LATERAL_RANGE / Math.max(1, Math.abs(dy)), 0, 1);
  // כשעדיין רחוקים מהיעד אנכית (latK→קטן) מכוונים אל ציר-הערוץ (x=0),
  // שם כל הפערים חופפים — כך הסירה מטפסת במרכז הערוץ וחוצה את כל
  // המחסומים; רק כשמתקרבים ליעד (latK→1) הכיוון נסוג אל x של היעד.
  const aimX = CHANNEL_CENTER_X * (1 - latK) + g.x * latK;
  const absBearing = (Math.atan2(aimX - bx, dy) * 180) / Math.PI;
  return ((absBearing - heading + 540) % 360) - 180;
}

// \u05d2\u05dc\u05d0\u05d9-\u05ea\u05e7\u05d9\u05e2\u05d4: \u05e2\u05d5\u05e7\u05d1 \u05d0\u05d7\u05e8 \u05d4\u05ea\u05e7\u05d3\u05de\u05d5\u05ea \u05e0\u05d8\u05d5 \u05d0\u05dc \u05d4\u05d9\u05e2\u05d3 \u05d1\u05d7\u05dc\u05d5\u05df \u05d6\u05de\u05df \u05e0\u05e2. \u05d0\u05dd \u05d1\u05de\u05e6\u05d1 seek
// \u05dc\u05d0 \u05d4\u05ea\u05e7\u05d3\u05de\u05e0\u05d5 \u2014 \u05e2\u05d5\u05d1\u05e8\u05d9\u05dd \u05dc\u05e2\u05e7\u05d9\u05d1\u05ea-\u05d3\u05d5\u05e4\u05df; \u05d0\u05dd \u05d1\u05de\u05e6\u05d1 follow \u05e2\u05e7\u05d9\u05d1\u05d4 \u05e9\u05dc\u05d0 \u05de\u05ea\u05e7\u05d3\u05de\u05ea \u2014
// \u05de\u05d7\u05dc\u05d9\u05e4\u05d9\u05dd \u05d9\u05d3 \u05db\u05d3\u05d9 \u05dc\u05e9\u05d1\u05d5\u05e8 \u05e1\u05d9\u05de\u05d8\u05e8\u05d9\u05d4 \u05d5\u05dc\u05d4\u05d9\u05de\u05e0\u05e2 \u05de\u05dc\u05d5\u05dc\u05d0\u05d4 \u05d0\u05d9\u05df-\u05e1\u05d5\u05e4\u05d9\u05ea.
function updateStuckDetector() {
  const now = performance.now();
  const bx = state.pose.x;
  const by = state.pose.y;
  const a = state.nav.progressAnchor;
  if (!a) {
    state.nav.progressAnchor = { x: bx, y: by, t: now };
    return;
  }
  // תזוזה מרחבית נטו מעבר לסף -> הסירה מגיעה למשהו (גם דרך עיקוף), לא מסתובבת
  // במקום -> אפס עוגן.
  if (Math.hypot(bx - a.x, by - a.y) >= STUCK_TRAVEL_CM) {
    state.nav.progressAnchor = { x: bx, y: by, t: now };
    return;
  }
  if (now - a.t < STUCK_WINDOW_MS) return;
  // חלון ארוך עם תזוזה מרחבית זעומה. הערה חשובה: במבוכי-מחיצות בבריכה
  // עקיבת-דופן (follow) רק הזיקה — היא “נדדה” את הסירה אחורה והחוצה מהפער.
  // שובר-התקיעה האמין כאן הוא תמרון-החילוץ (updateEscapeManeuver, נסיעה-לאחור)
  // שכבר רץ קודם. לכן איננו עוברים מ-seek ל-follow — רק מאפסים עוגן ונותנים
  // לשיוט/חילוץ להמשיך. (מצב follow נשמר בקוד אך אינו מופעל אוטומטית כאן.)
  state.nav.progressAnchor = { x: bx, y: by, t: now };
}

// \u05db\u05e0\u05d9\u05e1\u05d4 \u05dc\u05de\u05e6\u05d1 \u05e2\u05e7\u05d9\u05d1\u05ea-\u05d3\u05d5\u05e4\u05df: \u05d1\u05d5\u05d7\u05e8\u05d9\u05dd \u05d0\u05ea \u05d4\u05e6\u05d3 \u05e9\u05d1\u05d5 \u05d4\u05de\u05db\u05e9\u05d5\u05dc \u05e7\u05e8\u05d5\u05d1 \u05d9\u05d5\u05ea\u05e8 \u05db"\u05d9\u05d3" \u05dc\u05e2\u05e7\u05d9\u05d1\u05d4.
function enterWallFollow() {
  const distLeft = getMemoryDistance(270, 45);
  const distRight = getMemoryDistance(90, 45);
  state.nav.followHand = distLeft < distRight ? -1 : 1; // -1: \u05de\u05db\u05e9\u05d5\u05dc \u05de\u05e9\u05de\u05d0\u05dc, +1: \u05de\u05d9\u05de\u05d9\u05df
  state.nav.mode = "follow";
  state.nav.wallSince = performance.now();
  state.avoidDir = 0;
  state.nav.progressAnchor = { x: state.pose.x, y: state.pose.y, t: performance.now() };
}

// \u05e2\u05e7\u05d9\u05d1\u05ea \u05e7\u05d5-\u05de\u05ea\u05d0\u05e8 (Bug2). \u05de\u05d7\u05d6\u05d9\u05e8 true \u05db\u05dc \u05e2\u05d5\u05d3 \u05e0\u05e9\u05d0\u05e8\u05d9\u05dd \u05d1\u05e2\u05e7\u05d9\u05d1\u05d4 (\u05d5\u05db\u05d1\u05e8 \u05e0\u05e7\u05d1\u05e2\u05d5
// \u05de\u05d4\u05d9\u05e8\u05d5\u05d9\u05d5\u05ea \u05d4\u05de\u05e0\u05d5\u05e2\u05d9\u05dd), \u05d0\u05d5 false \u05db\u05e9\u05d4\u05d5\u05d7\u05dc\u05d8 \u05dc\u05e2\u05d6\u05d5\u05d1 \u05d5\u05dc\u05d7\u05d6\u05d5\u05e8 \u05dc-seek \u05d1\u05d0\u05d5\u05ea\u05d5 \u05d8\u05d9\u05e7.
function followWall(distFront, goalRel) {
  const hand = state.nav.followHand;
  const sideBearing = hand > 0 ? 90 : 270;
  const sideDist = getMemoryDistance(sideBearing, 45);

  // \u05ea\u05e0\u05d0\u05d9 \u05e2\u05d6\u05d9\u05d1\u05d4 (Bug2): \u05db\u05d9\u05d5\u05d5\u05df \u05d4\u05d9\u05e2\u05d3 \u05e4\u05ea\u05d5\u05d7 \u05e9\u05d5\u05d1 \u05d5\u05d4\u05d7\u05e8\u05d8\u05d5\u05dd \u05e4\u05e0\u05d5\u05d9 -> \u05d7\u05d6\u05d5\u05e8 \u05dc\u05e9\u05d9\u05d5\u05d8 \u05d0\u05dc \u05d4\u05d9\u05e2\u05d3.
  const goalClear = getMemoryDistance(
    clamp(goalRel, -OPEN_SCAN_DEG, OPEN_SCAN_DEG),
    OPEN_SCAN_TOL
  );
  if (
    Math.abs(goalRel) < 70 &&
    goalClear > WALL_LEAVE_CLEAR_CM &&
    distFront > CLEAR_DISTANCE_CM
  ) {
    state.nav.mode = "seek";
    state.nav.progressAnchor = { x: state.pose.x, y: state.pose.y, t: performance.now() };
    return false;
  }

  // \u05d7\u05e1\u05d9\u05de\u05d4 \u05d7\u05d6\u05d9\u05ea\u05d9\u05ea \u05ea\u05d5\u05da \u05db\u05d3\u05d9 \u05e2\u05e7\u05d9\u05d1\u05d4 -> \u05e4\u05e0\u05d4 \u05d7\u05d3 \u05d4\u05e8\u05d7\u05e7 \u05de\u05d4\u05d3\u05d5\u05e4\u05df.
  if (distFront < SAFE_DISTANCE_CM) {
    applyAvoidTurn(-hand);
    return true;
  }

  // \u05d1\u05e7\u05e8-P \u05dc\u05e9\u05de\u05d9\u05e8\u05ea \u05de\u05e8\u05d7\u05e7-\u05e1\u05e3 \u05de\u05d4\u05d3\u05d5\u05e4\u05df. sideDist \u05d2\u05d3\u05d5\u05dc (\u05d0\u05d9\u05df \u05d3\u05d5\u05e4\u05df) -> \u05e9\u05d2\u05d9\u05d0\u05d4 \u05d7\u05d9\u05d5\u05d1\u05d9\u05ea ->
  // \u05e4\u05d5\u05e0\u05d4 \u05d0\u05dc \u05d4\u05e6\u05d3 \u05dc\u05d7\u05e4\u05e9 \u05d0\u05ea \u05d4\u05d3\u05d5\u05e4\u05df; \u05e7\u05e8\u05d5\u05d1 \u05de\u05d3\u05d9 -> \u05e9\u05d2\u05d9\u05d0\u05d4 \u05e9\u05dc\u05d9\u05dc\u05d9\u05ea -> \u05de\u05ea\u05e8\u05d7\u05e7.
  const error = clamp(sideDist, 0, 400) - WALL_FOLLOW_STANDOFF_CM;
  const steer = clamp(hand * error * WALL_FOLLOW_GAIN, -STEER_MAX, STEER_MAX);
  state.cmd.leftSpeed = clamp(WALL_FOLLOW_SPEED - steer, -255, 255);
  state.cmd.rightSpeed = clamp(WALL_FOLLOW_SPEED + steer, -255, 255);
  return true;
}

// חילוץ מתקיעה פיזית (deadlock breaker). מחזיר true כל עוד תמרון-החילוץ פעיל
// (ואז המתקשר עוצר ומשאיר את פקודת הנסיעה-לאחור). מזהה "אין תזוזה נטו" ע"י מעקב
// אחר עוגן-פוזה: כל עוד הסירה מתקדמת מעבר ל-ESCAPE_MOVE_CM אנו מאפסים את העוגן;
// אם עברו ESCAPE_STALL_MS בלי תזוזה כזו — פותחים פרץ נסיעה-לאחור בקשת אל הצד
// הפתוח יותר. הפרץ מסתיים בתום ESCAPE_DURATION_MS או כשהחזית נפתחה מעבר ל-CLEAR.
function updateEscapeManeuver() {
  const now = performance.now();
  const bx = state.pose.x;
  const by = state.pose.y;

  // פרץ-חילוץ פעיל: נוסעים לאחור בקשת עד שהחזית נפתחת או שהזמן תם.
  if (now < state.nav.escapeUntil) {
    const frontClear = getMemoryDistance(0, 45);
    if (frontClear >= CLEAR_DISTANCE_CM) {
      state.nav.escapeUntil = 0;
      state.nav.moveAnchor = { x: bx, y: by, t: now };
      return false;
    }
    const dir = state.nav.escapeDir; // הקשת מוטה אל הצד הפתוח
    state.cmd.leftSpeed = clamp(-ESCAPE_SPEED - dir * ESCAPE_TURN, -255, 255);
    state.cmd.rightSpeed = clamp(-ESCAPE_SPEED + dir * ESCAPE_TURN, -255, 255);
    return true;
  }

  // מעקב תזוזה נטו לזיהוי תקיעה.
  const a = state.nav.moveAnchor;
  if (!a) {
    state.nav.moveAnchor = { x: bx, y: by, t: now };
    return false;
  }
  if (Math.hypot(bx - a.x, by - a.y) >= ESCAPE_MOVE_CM) {
    state.nav.moveAnchor = { x: bx, y: by, t: now }; // התקדמנו — אפס שעון
    return false;
  }
  if (now - a.t < ESCAPE_STALL_MS) return false;

  // נתקענו פיזית: בחר צד פתוח יותר (מאחור-צדדית) ופתח פרץ נסיעה-לאחור.
  const openLeft = getMemoryDistance(270, 45);
  const openRight = getMemoryDistance(90, 45);
  state.nav.escapeDir = openRight >= openLeft ? 1 : -1;
  state.nav.escapeUntil = now + ESCAPE_DURATION_MS;
  state.nav.moveAnchor = { x: bx, y: by, t: now };
  state.avoidDir = 0;
  const dir = state.nav.escapeDir;
  state.cmd.leftSpeed = clamp(-ESCAPE_SPEED - dir * ESCAPE_TURN, -255, 255);
  state.cmd.rightSpeed = clamp(-ESCAPE_SPEED + dir * ESCAPE_TURN, -255, 255);
  return true;
}

// ===================================================================
// Real-world reactive navigator (instantaneous radar, pose-free).
// On the physical boat there is no odometry/compass, the radar sweep is slow
// (~0.15-0.2 Hz) and noisy, and wind/waves make command-based dead reckoning
// (state.pose) and the accumulated world map (mapCells/radarMemory anchored via
// pose) unreliable. So the real-hardware autonomous path IGNORES pose AND the
// accumulated map and steers purely from the FRESHEST radar returns, expressed
// in BOW-relative bearings via a calibrated bow offset, with strong hysteresis
// and gentle speeds. The pose/goal/map stack below is kept only for the mock
// simulator (where pose tracks server truth).
// ===================================================================

// Bow direction expressed in the raw (sensor-mount + servo-angle) frame. The
// servo zero is NOT the bow. This is only a STARTING GUESS; the real offset is
// learned online, during motion, every trip (see the estimator in
// updateLiveScan) and stored in state.nav.bowOffsetDeg.
const BOW_SERVO_OFFSET_DEG = 60;
// The bow offset is now FIXED (user calibrated the servo perpendicular to the
// pool wall => bow at servo 60°). The online estimator wandered/locked onto
// wrong values in the enclosed pool, so it is DISABLED: bowOffsetDeg stays
// pinned at BOW_SERVO_OFFSET_DEG. Set false to re-enable online bow learning.
const BOW_FIXED = true;
// usFront water-surface reflection band: readings in [MIN,MAX] are the phantom
// return off the water (~constant regardless of the real distance) and are
// treated as OPEN water for navigation. Tune to match the observed water value.
const FRONT_WATER_MIN_CM = 38;
const FRONT_WATER_MAX_CM = 60;
// The radar array is a ROTATING quarter-sweep (servo 0..90 in 15 deg steps, one
// step per telemetry frame ~350ms), so any given bow-relative bearing is only
// re-sampled about once every ~6 steps ~ 2-3s (worse when the RF link stalls).
// If the TTL is shorter than that the BOW bin goes stale mid-sweep, liveCone
// returns count 0 => front reads 999 ("open") => the nav cruises straight into a
// wall it had ALREADY seen on an earlier sweep. Trust a bin for a full sweep +
// link jitter (matches the wall-display WALL_TTL_MS 4000). The boat is slow
// (~16 cm/s) so a 4 s-old bearing is still spatially valid enough to avoid.
const LIVE_SCAN_TTL_MS = 4000;   // a bearing bin is trusted only if refreshed this recently
const LIVE_BIN_DEG = 15;         // bin width ~ sensor field of view
const RW_CRUISE = 82;            // gentle forward (real momentum + noise tolerance)
const RW_REVERSE = 80;           // backing-out magnitude when blocked / boxed in
const RW_FRONT_CONE_DEG = 50;    // half-cone treated as "ahead". WIDE on purpose: the online bow estimate is unreliable in the enclosed pool (wanders 54-81°), so a head-on wall can land up to ~45° off the estimated bow. A wide cone catches it despite that error (root cause of the head-on collisions). The block decision uses the cone MIN (nearest), not the median, so one close wall can't be outvoted by open readings beside it.
const RW_SIDE_BEARING_DEG = 55;  // bearing where we probe for the escape side
const RW_SIDE_TOL_DEG = 40;
// המכ"ם הסורב מרענן כיוון נתון רק כל ~2 ש'; במהירות המים הסירה עוברת עד ~30 ס"מ
// בין ריענון לריענון. לכן ספי-החזית מוגדלים כדי שההחלטה לעצור/לפנות תיפול בעודה
// רחוקה מספיק — גם אם הגוף "התקרב" 30 ס"מ מאז המדידה האחרונה.
const RW_BLOCK_CM = 50;          // front closer than this -> reverse. Set BELOW the usFront water-reflection band (~56cm min) so open water never false-blocks, while a real near wall (reported as the nearer echo) does. Front cone uses the MEDIAN so a lone wave dip won't trip it.
const RW_EMERGENCY_CM = 32;      // hard stop -> reverse away, regardless of intent (was 24)
const RW_SIDE_DANGER_CM = 22;    // side wall closer than this -> immediate turn AWAY from itconst RW_ARC_DEG = 70;           // forward hemisphere scanned for a gap to steer into
// Turning is never the default: within this heading error the boat drives dead
// straight; only a gap notably off to a side eases the inner motor into a
// forward ARC (turn WHILE advancing), so it only ever turns in order to travel.
const RW_ARC_DEADBAND_DEG = 20;
const RW_SIDE_TIE_CM = 15;       // |left-right| below this is a tie -> break toward goal
// --- Wave/wind robustness (self-refining, no hardcoded environment size) ---
const RW_MAX_RANGE = 300;        // "no echo" (open water) is stored as this range
// Physical prior about THIS course: no two SEPARATE (unconnected) parallel walls
// are ever closer than this. So an opposing pair of returns whose ranges sum to
// less than this describes a corridor that cannot exist -> one of them is a
// ghost/echo, and applyMinCorridorPrior() calibrates it out of the live scan.
const MIN_CORRIDOR_CM = 75;
// The course is never a closed box (always an open way in/out). A cardinal side
// counts as "walled" for that test when its clearance is below this.
const CLOSED_BOX_WALL_CM = 60;
// Navigation reacts ONLY to obstacles inside this range. Anything farther is
// noisy and irrelevant to a decision, so every clearance is capped here: all
// bearings >= 1 m read as equally "open", and steering is driven purely by the
// sub-metre returns. Differences BELOW the cap (i.e. real nearby obstacles) are
// preserved, so "aim at the farthest direction" still works when boxed in.
const RW_DECISION_RANGE_CM = 200;
const RW_OPEN_FRAC = 0.55;       // open gap = clearance >= this fraction of the deepest in view
const RW_OPEN_MIN_CM = 45;       // floor for the adaptive open threshold (small pools)
const RW_SPIKE_RATE_CM_S = 130;  // implied closing speed above this = wave spike, never votes
// --- Online bow calibration (runs during motion, recalibrates every trip) ---
const CALIB_DURATION_MS = 3000;  // max length of the initial straight seed pulse
const RW_FWD_EST_NET = 120;      // min |L|+|R| that counts as "driving forward"
const RW_FWD_EST_DIFF = 40;      // max |L-R| that still counts as "roughly straight"
const RW_CLOSE_RATE_MIN = 5;     // cm/s: min closing rate for a bearing to vote for the bow
const RW_CLOSE_RATE_CAP = 60;    // cm/s: clamp a single vote's weight
const BOW_EST_DECAY = 0.9;       // per-update memory of the circular accumulator
const BOW_EST_MIN_CONF = 50;     // accumulator magnitude required before adopting the estimate
// The accumulator magnitude ALONE always grows past BOW_EST_MIN_CONF given enough
// votes (geometric sum of the decay), even from a flat/ambiguous distribution — so
// magnitude by itself will happily "lock" onto noise. In a small enclosed pool the
// walls close in from EVERY side while driving, so the closing-rate votes are near
// uniform (verified on real logs: mean resultant length ~0.1-0.3 for the runs with
// enough samples). Adopt the online bow ONLY when the votes genuinely AGREE on a
// direction: require the mean resultant length (|resultant| / accumulated weight)
// to clear this consensus floor AND a minimum accumulated weight. Otherwise keep
// the calibrated seed offset (stable, slightly-off bow beats a randomly-rotating
// world-view every trip). An open course with a real dominant bow signal can still
// clear it; a walled pool correctly won't, and falls back to the seed.
const BOW_EST_MIN_CONSENSUS = 0.6;  // mean resultant length required to trust the estimate
const BOW_EST_MIN_WEIGHT = 200;     // min accumulated (decayed) vote weight before adopting
// Even a genuine consensus can appear for a moment from a lucky burst of aligned
// votes. A FIXED hardware bow does not wander, so also demand TEMPORAL STABILITY:
// the instantaneous estimate must stay within a tight tolerance for several
// consecutive qualifying updates before we adopt it. A transient burst resets the
// streak, so only a bow that is simultaneously well-supported, agreed-upon, AND
// steady over time is trusted — exactly what a real fixed bow looks like and what
// pool noise cannot fake.
const BOW_EST_STABLE_DEG = 18;      // max wander of the estimate to count as "steady"
const BOW_EST_STABLE_HITS = 8;      // consecutive steady qualifying updates to lock

// bowRel bin (integer, -180..180 step LIVE_BIN_DEG) -> { dist, t }
const liveScan = new Map();
// Online bow estimator state: last range per raw bin + a decaying circular
// accumulator of "which raw bearing is closing fastest while moving forward".
const rawPrev = new Map();
let bowEstSin = 0;
let bowEstCos = 0;
// Decaying SUM of vote weights (same BOW_EST_DECAY), so |resultant|/bowEstWeight is
// the mean resultant length (vote consensus in 0..1). Distinguishes "many votes
// agreeing" (real bow) from "many votes cancelling out" (pool noise).
let bowEstWeight = 0;
// Temporal-stability tracker for the bow lock: the last accepted candidate angle
// and how many consecutive qualifying updates have agreed with it.
let bowCandidateDeg = null;
let bowStableHits = 0;

function wrap180(deg) {
  return (((deg % 360) + 540) % 360) - 180;
}

function binBearing(deg) {
  return wrap180(Math.round(wrap180(deg) / LIVE_BIN_DEG) * LIVE_BIN_DEG);
}

// Fold the current telemetry frame into the pose-free live scan AND run the
// online bow estimator. Called on every telemetry refresh. Uses the ECHOED
// servo angle so a bearing is only stored where the sensor array actually
// pointed.
function updateLiveScan() {
  const servo = state.telemetry.radarAngle ?? state.cmd.radarAngle ?? 0;
  const now = performance.now();
  const bowOff = state.nav.bowOffsetDeg ?? BOW_SERVO_OFFSET_DEG;
  // Only a roughly-straight forward run makes the bow the fastest-closing
  // bearing, so gate the estimator on it.
  const L = state.cmd.leftSpeed;
  const R = state.cmd.rightSpeed;
  const forwardMotion =
    L > 0 && R > 0 && L + R >= RW_FWD_EST_NET && Math.abs(L - R) <= RW_FWD_EST_DIFF;
  for (const beam of SENSOR_BEAMS) {
    // NOTE: usFront is fed to nav (NOT skipped here). Its fault is a ~65cm water
    // reflection when the bow is OPEN, but ultrasonic reports the NEAREST echo,
    // so a real wall CLOSER than ~65cm is reported correctly. RW_BLOCK_CM is set
    // below the water-reflection band so open water never false-blocks, while a
    // genuine near wall ahead still triggers the reverse. (It IS skipped in the
    // DISPLAY loops via MASK_FRONT_DISPLAY so the open-water reflection doesn't
    // draw a phantom wall on the radar picture.)
    const raw = state.telemetry[beam.key];
    if (raw == null || raw < 0) continue; // no data / invalid this beam
    // FILTER the usFront water-surface reflection: it reports a roughly-constant
    // ~38-60cm regardless of the real distance ahead. Treat readings inside that
    // band as OPEN water (no echo) so the phantom "wall" never blocks navigation.
    // A genuinely close wall reads BELOW the band (nearest echo) and still
    // triggers the reactions; anything beyond the band is real too.
    const isFrontWater =
      beam.key === "usFront" && raw >= FRONT_WATER_MIN_CM && raw <= FRONT_WATER_MAX_CM;
    // "No echo" (0/999) is real information: open water at max range. Storing it
    // (instead of skipping) lets freshness detection work AND lets a momentary
    // wave DROPOUT be seen as "open at max" rather than an unknown/stale gap.
    const measured = raw === 0 || raw >= 999 || isFrontWater ? RW_MAX_RANGE : raw;
    const rawBearing = normalizeDeg(beam.dir + servo); // sensor-mount + servo (raw frame)
    const bowRel = binBearing(rawBearing - bowOff);    // 0 = bow (current estimate)
    liveScan.set(bowRel, { dist: measured, t: now });

    // --- Online bow calibration: the raw bearing whose range CLOSES fastest
    // while driving forward IS the bow. Votes feed a decaying circular
    // accumulator, so the offset self-calibrates and re-learns every trip with
    // no hardcoded servo/bow value. Works in the RAW frame, independent of the
    // current estimate, so it never feeds back on itself.
    const key = binBearing(rawBearing);
    const prev = rawPrev.get(key);
    rawPrev.set(key, { dist: measured, t: now });
    if (forwardMotion && prev) {
      const dt = (now - prev.t) / 1000;
      if (dt >= 0.1 && dt <= 4) {
        const rate = (measured - prev.dist) / dt; // cm/s; negative = closing
        // Vote only for a plausible closing; reject wave spikes whose implied
        // speed no real boat could make.
        if (rate < -RW_CLOSE_RATE_MIN && rate > -RW_SPIKE_RATE_CM_S) {
          const w = Math.min(-rate, RW_CLOSE_RATE_CAP);
          const rad = degToRad(rawBearing);
          bowEstSin = BOW_EST_DECAY * bowEstSin + w * Math.sin(rad);
          bowEstCos = BOW_EST_DECAY * bowEstCos + w * Math.cos(rad);
          bowEstWeight = BOW_EST_DECAY * bowEstWeight + w;
        }
      }
    }
  }
  // Adopt the running estimate ONLY when the votes both accumulate enough evidence
  // AND genuinely agree on a direction (mean resultant length). Magnitude alone
  // would lock onto the flat closing-rate distribution of a walled pool; the
  // consensus test rejects that and keeps the calibrated seed offset instead.
  const resultant = Math.hypot(bowEstSin, bowEstCos);
  const consensus = bowEstWeight > 1e-6 ? resultant / bowEstWeight : 0;
  if (
    resultant >= BOW_EST_MIN_CONF &&
    bowEstWeight >= BOW_EST_MIN_WEIGHT &&
    consensus >= BOW_EST_MIN_CONSENSUS
  ) {
    const estDeg = normalizeDeg((Math.atan2(bowEstSin, bowEstCos) * 180) / Math.PI);
    // Temporal stability: the estimate must persist near the same angle for
    // several consecutive qualifying updates before it is trusted.
    if (
      bowCandidateDeg != null &&
      Math.abs(wrap180(estDeg - bowCandidateDeg)) <= BOW_EST_STABLE_DEG
    ) {
      bowStableHits++;
    } else {
      bowStableHits = 1;
    }
    // Track a slowly-updated candidate (light smoothing keeps it centred).
    bowCandidateDeg =
      bowCandidateDeg == null
        ? estDeg
        : normalizeDeg(bowCandidateDeg + wrap180(estDeg - bowCandidateDeg) * 0.3);
    if (bowStableHits >= BOW_EST_STABLE_HITS && !BOW_FIXED) {
      state.nav.bowOffsetDeg = bowCandidateDeg;
      state.nav.bowLocked = true;
    }
  } else {
    // Evidence faded below the gate — decay the streak so a later burst must
    // re-establish stability rather than resume an old count.
    bowStableHits = Math.max(0, bowStableHits - 1);
  }
}

// Calibrate the live scan against the course prior in MIN_CORRIDOR_CM: no two
// SEPARATE parallel walls in this course are closer than that. So whenever a
// return on some bearing and an OPPOSING return (~180° away) sum to less than
// MIN_CORRIDOR_CM, that pair describes a physically impossible narrow corridor —
// one of the two is a ghost (multipath/echo). Ultrasonic ghosts are the weaker,
// FARTHER return (the extra bounce adds path length), while the nearest echo is
// the true nearest surface, so we KEEP the nearer wall and open the farther bin
// back up to max range. This scrubs phantom narrow corridors out of the scan the
// navigator reads, so a wall that isn't there can't make the boat brake/turn.
//
// SAFETY: we never open a bin in the FORWARD cone (|bin| <= RW_FRONT_CONE_DEG).
// Blinding ourselves straight ahead is the one unacceptable failure, so if the
// bin we would open faces forward we leave the pair untouched (stay conservative)
// and rely on the reverse/turn logic instead.
function applyMinCorridorPrior() {
  const now = performance.now();
  const done = new Set();
  for (const [bin, e] of liveScan) {
    if (done.has(bin)) continue;
    if (now - e.t > LIVE_SCAN_TTL_MS || e.dist >= RW_MAX_RANGE) continue;
    const opp = wrap180(bin + 180);
    const o = liveScan.get(opp);
    done.add(bin);
    if (!o) continue;
    done.add(opp);
    if (now - o.t > LIVE_SCAN_TTL_MS || o.dist >= RW_MAX_RANGE) continue;
    if (e.dist + o.dist >= MIN_CORRIDOR_CM) continue; // corridor is physically possible
    // Impossible corridor: keep the nearer (real) wall, open the farther (ghost).
    const farBin = e.dist <= o.dist ? opp : bin;
    if (Math.abs(farBin) <= RW_FRONT_CONE_DEG) continue; // never blind the bow
    liveScan.set(farBin, { dist: RW_MAX_RANGE, t: now });
  }
}

// Calibrate against the course prior that it NEVER forms a closed box: there is
// always at least one open side (the way in / the way out) — walls run as
// continuous lines out to sensor range or until they meet another wall at a 90°
// corner, but they never seal the boat in on all four sides. So if all FOUR
// cardinal directions (front, right, rear, left) read a wall within
// CLOSED_BOX_WALL_CM, the picture is physically impossible: the weakest (FARTHEST)
// of the four is the most likely ghost, so we open it back up to max range. This
// gives the navigator a real escape instead of the false "boxed-in" that makes it
// stall/reverse for a wall that isn't there.
//
// SAFETY: never open the FORWARD cone. If the farthest happens to be dead ahead
// we open the next-farthest non-forward cardinal instead, so we never blind the
// bow — a real wall astern/beside us is re-checked the moment we move toward it.
function applyNoClosedBoxPrior() {
  const now = performance.now();
  const reads = [0, 90, 180, -90].map((d) => {
    const c = liveCone(d, RW_WALL_TOL_DEG);
    return { d, dist: c.count ? c.median : RW_MAX_RANGE };
  });
  if (!reads.every((r) => r.dist < CLOSED_BOX_WALL_CM)) return; // not a closed box
  const openable = reads
    .filter((r) => Math.abs(r.d) > RW_FRONT_CONE_DEG) // never the bow
    .sort((a, b) => b.dist - a.dist); // farthest (weakest) first
  if (!openable.length) return;
  const target = openable[0].d;
  for (const [bin, e] of liveScan) {
    if (now - e.t > LIVE_SCAN_TTL_MS) continue;
    if (Math.abs(wrap180(bin - target)) <= RW_WALL_TOL_DEG) {
      liveScan.set(bin, { dist: RW_MAX_RANGE, t: now });
    }
  }
}

// Fresh returns within +/-tolDeg of a bow-relative bearing, summarised. The
// MEDIAN rejects isolated wave/spray spikes (a lone near- or far-blip is
// outvoted by its neighbours) without adding any temporal latency at the slow
// link cadence; min is kept for the hard emergency stop; count distinguishes
// "open water" (fresh, large) from "no data yet / dropout" (count 0).
function liveCone(bearingRel, tolDeg) {
  const now = performance.now();
  const vals = [];
  for (const [bin, e] of liveScan) {
    if (now - e.t > LIVE_SCAN_TTL_MS) continue;
    // Cap every clearance at the 1 m decision range: obstacles farther than this
    // are irrelevant noise and must not sway steering (they all read "open").
    if (Math.abs(wrap180(bin - bearingRel)) <= tolDeg)
      vals.push(Math.min(e.dist, RW_DECISION_RANGE_CM));
  }
  if (vals.length === 0) return { min: 999, median: 999, count: 0 };
  vals.sort((a, b) => a - b);
  return { min: vals[0], median: vals[Math.floor(vals.length / 2)], count: vals.length };
}

// Spike-robust distance (median of the fresh cone). 999 = open / no data.
function liveDistance(bearingRel, tolDeg) {
  return liveCone(bearingRel, tolDeg).median;
}

// Deepest (farthest) fresh clearance over a bearing range, full-circle capable.
// Used to pick an escape heading when boxed in: we always move toward the most
// open water, even if that water is behind us.
function bestOpenBearing(loB, hiB) {
  let best = -1;
  let bestB = 0;
  let count = 0;
  for (let b = loB; b <= hiB; b += LIVE_BIN_DEG) {
    const c = liveCone(b, LIVE_BIN_DEG * 0.8);
    if (c.count === 0) continue; // no fresh data at this bearing
    count += c.count;
    if (c.median > best) {
      best = c.median;
      bestB = b;
    }
  }
  return { bearing: bestB, clear: best, count };
}

// --- Wall AVOIDANCE (stay OFF the walls, roam the open middle) --------------
// Replaces the old left-wall FOLLOWING. React when ANY wall is within the buffer
// and steer AWAY from it toward the more-open side. The buffer is deliberately
// large so the boat keeps a comfortable margin from walls instead of hugging.
const RW_KEEPAWAY_CM = 55;         // side wall within this -> turn away toward open water
const RW_KEEPAWAY_CLEAR_CM = 75;   // after turning away, drive straight until the near side recovers to this
const RW_WALL_TOL_DEG = 30;       // סובלנות זוויתית לחישת דופן צד
// אחרי שכיוונּו את החרטום הרחק מקיר קרוב, מחויבים לנסיעה-ישר קדימה עד שהמרווח
// חוזר ליעד — כדי *להתרחק בפועל* (תזוזה), במקום עוד סיבוב-במקום שרק מנדנד. חלון
// זמן מקסימלי כבטיחות; בפועל הבריחה מסתיימת מוקדם ברגע שהמרווח התאושש.
const RW_WALL_ESCAPE_MS = 2500;
// נסיעה-לאחור מחויבת כ"פרץ" בעל משך מינימלי, לא נדנוד של טיק בודד. בקצב-קישור
// איטי ורועש קריאת-החזית מתנדנדת סביב סף הנסיעה-לאחור, וכל טיק שמחליט מחדש
// לאחור/קדימה גורם לסירה לנוע קדימה-אחורה במקום בלי להתקדם. פרץ מחויב נסוג מרחק
// אמיתי, ואז ההערכה הבאה מוצאת מקום להסתובב לעבר הפתח ולצאת.
const RW_REVERSE_BURST_MS = 800;
// פרץ סיבוב-במקום מחויב: קצר בכוונה (~צעד בודד ≈ טיק אחד ב-CONTROL_INTERVAL_MS).
// אחרי כל פרץ הסירה נעצרת וממתינה שהמכ"ם יתרענן לפני שתסתובב שוב, כדי לא לפנות
// לתוך גוף שטרם נסרק מחדש. סיבוב-במקום מועדף על קשת כי טביעת-הרגל מזערית.
const RW_SPIN_BURST_MS = 250;
// סיבוב-במקום מחויב במקום קשת. עדיף על קשת: הסירה שומרת טביעת-רגל מזערית (מסתובבת
// סביב מרכזה) במקום "לטאטא" מרחב עם הגוף תוך כדי פנייה, כך שהסיכוי להיתקע/להיחכך
// בגוף סמוך קטן משמעותית. dir>0 = החרטום מסתובב ימינה (turn>0: ימין קדימה, שמאל
// אחורה); dir<0 = שמאלה.
function spinInPlaceCmd(dir) {
  return dir > 0
    ? { left: -MOTOR_SPEED, right: MOTOR_SPEED }
    : { left: MOTOR_SPEED, right: -MOTOR_SPEED };
}

// קשת-נסיעה-לאחור: הסירה *נוסעת אחורה* (תזוזה אמיתית, לא סיבוב-במקום עקר) ותוך
// כדי כך מסובבת את החרטום לכיוון dir (הרחק מהקיר). מנוע אחד לאחור, השני ב-0, כך
// שיש גם תזוזה אחורה וגם פנייה. dir>0 = החרטום מסתובב ימינה (turn=right-left>0);
// dir<0 = שמאלה. זה מחליף את הסיבוב-במקום כתגובת-התחמקות, לפי בקשת המשתמש —
// "אם יש קיר, שיסע אחורה, שלא יסתובב במקום".
function reverseArcCmd(dir) {
  return dir > 0
    ? { left: -RW_REVERSE, right: 0 } // אחורה + חרטום ימינה
    : { left: 0, right: -RW_REVERSE }; // אחורה + חרטום שמאלה
}

// פותח פרץ סיבוב-במקום מחויב לכיוון dir. הפרץ קצר (RW_SPIN_BURST_MS ≈ צעד בודד):
// אחריו הסירה נעצרת וממתינה שהתמונה תתייצב ותתרענן (radarPictureStable) לפני
// ההחלטה הבאה — סובב → עצור → חכה לתמונה → הערך מחדש, כדי לא לסובב לתוך קיר.
function startSpinBurst(dir, now) {
  state.nav.spinDir = dir;
  state.nav.spinUntil = now + RW_SPIN_BURST_MS;
}


// Real-world autonomous tick. GUIDING PRINCIPLE: heading changes are made by
// COMMITTED IN-PLACE SPIN bursts, not arcs. Pivoting about the boat's own centre
// keeps a minimal footprint, so it is far less likely to snag the hull on a
// nearby body than an arc that sweeps the hull sideways through space. Each spin
// burst is short (RW_SPIN_BURST_MS): after it the boat STOPS and waits for the
// radar picture to settle and refresh before deciding again, so it never turns
// blind into a body it has not re-scanned. It TRAVELS straight forward when the
// bow is already aimed at open water, and backs STRAIGHT out only when boxed in
// (no open heading to spin toward). Front-block + boxed-in checks keep it from
// grinding into a wall.
function updateAutonomousRealworld() {
  const now = performance.now();

  // --- פרץ סיבוב-במקום מחויב בעיצומו: מכבדים אותו עד סופו גם אם התמונה כרגע
  // "לא-יציבה" (הסחרור עצמו הופך אותה ללא-יציבה). זה סיבוב מכוון וקצר, ולכן הוא
  // נבדק לפני מחסום ה-radarPictureStable — אחרת הסחרור היה עוצר את עצמו מיד. ---
  if (now < state.nav.spinUntil) {
    const sc = spinInPlaceCmd(state.nav.spinDir);
    state.avoidDir = 0;
    state.cmd.leftSpeed = sc.left;
    state.cmd.rightSpeed = sc.right;
    return;
  }

  // אחרי סיבוב במקום התמונה עוד נמרחת/מתייצבת (radarPictureStable=false): אל
  // תחליט על סמך מכ"ם לא-אמין — עוצרים במקום עד שהסחרור נעצר וסריקה טרייה בנתה
  // מחדש את התמונה. העצירה גם מאיצה את דעיכת קצב-הסחרור (DR_TURN_DRAG), כך שחלון
  // ההתייצבות מתחיל לספור מיד.
  if (!radarPictureStable()) {
    state.avoidDir = 0;
    state.cmd.leftSpeed = 0;
    state.cmd.rightSpeed = 0;
    return;
  }

  // --- Seed phase: gentle straight pulse until the bow is learned online ---
  // (all bearings are bow-relative via the online-calibrated bow offset).
  if (state.nav.rwPhase === "calib") {
    const frontNow = liveCone(0, RW_FRONT_CONE_DEG).min;
    const done = now - (state.nav.calibStart || now) >= CALIB_DURATION_MS;
    if (frontNow < RW_BLOCK_CM || done || state.nav.bowLocked) {
      state.nav.rwPhase = "run";
      liveScan.clear(); // reindex fresh returns under the learned offset
    } else {
      state.cmd.leftSpeed = MOTOR_SPEED; // straight, gentle
      state.cmd.rightSpeed = MOTOR_SPEED;
      return;
    }
  }

  const fc = liveCone(0, RW_FRONT_CONE_DEG);
  // Use the cone MIN (nearest fresh return), not the median: with the WIDE front
  // cone a single close wall dead-ahead must not be outvoted by open water beside
  // it. Upstream median-of-3 per sensor already despikes, so min is not spiky.
  const front = fc.min;
  // Bow-relative clearances on the four cardinal sides. LEFT/RIGHT drive the
  // reactive turn-away and the left-wall follower; REAR gates reversing so we
  // never back into a wall astern ("wall behind -> go forward").
  const leftCone = liveCone(-90, RW_WALL_TOL_DEG);
  const rightCone = liveCone(90, RW_WALL_TOL_DEG);
  const leftD = leftCone.median;
  const rightD = rightCone.median;
  const rear = liveDistance(180, RW_FRONT_CONE_DEG);

  // --- Committed reverse burst in progress: keep BACKING AWAY for a real
  // distance UNLESS a wall appears astern — a wall behind us means we cannot back
  // up, so we abort and re-decide this tick instead of backing into it. The burst
  // may be STRAIGHT back (front wall) or a REVERSE ARC (side wall: backs while
  // swinging the bow off the wall). Never an in-place spin. ---
  if (now < state.nav.reverseUntil) {
    if (rear < RW_BLOCK_CM) {
      state.nav.reverseUntil = 0; // wall behind -> stop reversing, re-decide
    } else {
      state.avoidDir = 0;
      const rc = state.nav.reverseArcDir
        ? reverseArcCmd(state.nav.reverseArcDir)
        : { left: -RW_REVERSE, right: -RW_REVERSE };
      state.cmd.leftSpeed = rc.left;
      state.cmd.rightSpeed = rc.right;
      return;
    }
  }

  // --- Wall AHEAD (front is the most dangerous, we travel forward). Response:
  // back STRAIGHT out — NEVER turn/spin when a wall is dead ahead (user rule).
  // Commit a reverse burst so the boat gains real distance. Only if the stern is
  // ALSO blocked (can't back up without hitting it) do we HOLD and wait for a
  // fresh scan instead of spinning. Steering away happens later, via the
  // side keep-away logic, once the bow is no longer blocked. ---
  if (fc.count > 0 && front < RW_BLOCK_CM) {
    state.avoidDir = 0;
    if (rear >= RW_BLOCK_CM) {
      state.nav.reverseArcDir = 0; // straight back
      state.nav.reverseUntil = now + RW_REVERSE_BURST_MS; // wall ahead -> straight reverse
      state.cmd.leftSpeed = -RW_REVERSE;
      state.cmd.rightSpeed = -RW_REVERSE;
    } else {
      state.cmd.leftSpeed = 0; // boxed front+stern -> hold (never spin), wait for scan
      state.cmd.rightSpeed = 0;
    }
    return;
  }

  // --- KEEP AWAY FROM WALLS (the core of wall-avoidance): a side wall is within
  // the buffer -> BACK AWAY with a committed REVERSE ARC (drive backward while
  // swinging the bow toward the more-open side) — NOT an in-place spin. The user
  // wants the boat to reverse out, not pivot on the spot. The reverse-burst honor
  // above then keeps backing (aborting only if the stern is blocked). In a
  // corridor where both sides are within the buffer this swings toward the
  // farther side, backing the boat toward the open middle. ---
  if (Math.min(leftD, rightD) < RW_KEEPAWAY_CM && rear >= RW_BLOCK_CM) {
    const nearSide = leftD <= rightD ? -1 : 1; // which wall is too close
    const openDir = -nearSide;                 // swing the bow away from it
    state.nav.reverseArcDir = openDir;
    state.nav.reverseUntil = now + RW_REVERSE_BURST_MS;
    const rc = reverseArcCmd(openDir);
    state.avoidDir = 0;
    state.cmd.leftSpeed = rc.left;
    state.cmd.rightSpeed = rc.right;
    return;
  }

  // --- No FRESH knowledge straight ahead: the rotating scanner has not
  // re-swept the bow within LIVE_SCAN_TTL_MS (slow quarter-sweep + RF stalls).
  // "Unknown ahead" is NOT "open water" — cruising blind is exactly how the boat
  // drove into a wall it had already seen on an earlier sweep. Hold still (a
  // safe, in-place wait) until the sweep refreshes the bow, then decide next
  // tick with real data instead of charging forward. ---
  if (fc.count === 0) {
    state.cmd.leftSpeed = 0;
    state.cmd.rightSpeed = 0;
    return;
  }

  // --- Open water on all sides: cruise straight forward down the middle. ---
  state.cmd.leftSpeed = RW_CRUISE;
  state.cmd.rightSpeed = RW_CRUISE;
}

// Instantaneous-radar safety envelope for the physical boat (no pose, no map).
function realworldSafeCommand(cmd) {
  const fc = liveCone(0, RW_FRONT_CONE_DEG);
  const netForward = cmd.leftSpeed + cmd.rightSpeed;
  // Use the fresh-cone MIN here (not the median): the hard stop must react even
  // to a single genuinely-close return, at the cost of an occasional spray stop.
  if (fc.count > 0 && fc.min < RW_EMERGENCY_CM && netForward > 0) {
    // Something is right on the bow and we are still driving into it. Back
    // STRAIGHT away (both motors) to gain real clearance — a straight reverse
    // has a minimal footprint and won't sweep the bow sideways into whatever is
    // beside us the way an arc or a spin would when a body is this close. Commit
    // a reverse burst so the next ticks keep backing off instead of lunging
    // forward again.
    state.avoidDir = 0;
    state.nav.spinUntil = 0; // cancel any in-progress spin — clearance comes first
    state.nav.reverseArcDir = 0; // straight back
    state.nav.reverseUntil = performance.now() + RW_REVERSE_BURST_MS;
    cmd.leftSpeed = -RW_REVERSE;
    cmd.rightSpeed = -RW_REVERSE;
  }
  cmd.leftSpeed = shapeMotorSpeed(cmd.leftSpeed);
  cmd.rightSpeed = shapeMotorSpeed(cmd.rightSpeed);
  return cmd;
}

// ============================ PROTOTYPE NAVIGATOR ==========================
// Deliberately DUMB reactive navigator built from the boat's REAL motion
// primitives: ROTATE-IN-PLACE and DRIVE-STRAIGHT (no arcs, no reverse). Each
// iteration: find the most-open bearing in the 120° arc in front of the bow
// (±60°); if the bow is not yet pointed at it, ROTATE IN PLACE toward it; once
// aligned, DRIVE STRAIGHT forward. Then scan again and repeat. Over successive
// iterations the incremental rotate-toward-the-open-edge lets the bow swing past
// 60° when needed, so the boat can turn away from a blocked front. Toggle with
// PROTO_NAV. All bearings are bow-relative via liveScan.
const PROTO_NAV = true;             // true = use this prototype instead of the sophisticated nav
const PROTO_ARC_HALF_DEG = 60;      // scan ±60° => the 120° arc in front of the bow
const PROTO_BIN_DEG = 15;           // scan resolution (~sensor FOV)
const PROTO_ALIGN_DEG = 18;         // bow within this of the open gap => drive straight; else pulse-rotate
const PROTO_CLEAR_CM = 70;          // front counts as "open enough to drive" only if the best bearing is at least this clear; below it the front is BLOCKED -> rotate to search, never drive into it
const PROTO_SIDE_HALF_DEG = 90;     // when blocked, look this wide each side to pick which way to rotate toward open water
const PROTO_SPEED = 80;             // single drive magnitude (shaped to the 40-100 band on the wire)
const PROTO_SPIN_PULSE_MS = 150;    // ONE short rotate pulse (holds ~1 control tick), then stop
const PROTO_SETTLE_MS = 550;        // after a pulse: STOP this long so rotation dies + a fresh scan arrives, THEN re-decide
const PROTO_EMERGENCY_CM = 40;      // wall closer than this DEAD AHEAD -> back up (the only allowed reverse). Raised so it retreats from farther and never grinds head-on into the wall.
const PROTO_DEAD_HALF_DEG = 25;     // half-cone treated as "directly in front of the bow" for the emergency check. Kept below +30 so usRight (bowRel ~30+servo with bow fixed at 60) never leaks into the front dead cone and triggers an endless false reverse.
const PROTO_REVERSE_MS = 250;       // how long the emergency back-up lasts (~0.25 second)
const PROTO_SIDE_CM = 30;           // wall closer than this on a SIDE -> run that side's motor ~1s to push the bow away (raised +5cm so the boat keeps more clearance from side walls)
const PROTO_SIDE_TOL_DEG = 35;      // ± tolerance around ±90° for the side check
const PROTO_SIDE_MS = 1000;         // how long the single side-motor burst lasts (~1 second)
const PROTO_REAR_CM = 30;           // while reversing, wall closer than this BEHIND -> abort reverse, pulse FORWARD (raised +5cm for more rear clearance)
const PROTO_REAR_TOL_DEG = 40;      // ± tolerance around 180° (rear) for the astern check
const PROTO_FWD_PULSE_MS = 300;     // how long the escape-forward pulse lasts when a wall is close astern
// Turn polarity: the physical boat rotated OPPOSITE to the code's turn convention
// (observed live: it turned TOWARD a wall instead of away). -1 flips every turn
// (in-place spin AND single-motor arc) so the bow rotates the intended way. If
// turns ever come out reversed again, set this to +1.
const PROTO_TURN_SIGN = -1;

// In-place spin that rotates the BOW toward bowDir (+1=right,-1=left), honoring
// the boat's real turn polarity.
function protoSpinCmd(bowDir) {
  return spinInPlaceCmd(bowDir * PROTO_TURN_SIGN);
}
// Single forward motor (arc) that rotates the BOW toward bowDir, same polarity.
function protoOneMotorCmd(bowDir) {
  const d = bowDir * PROTO_TURN_SIGN;
  return d > 0 ? { left: 0, right: PROTO_SPEED } : { left: PROTO_SPEED, right: 0 };
}

// Best (max) fresh clearance over a bearing range, with the bearing that gave it.
function protoBestGap(loDeg, hiDeg) {
  let best = -1;
  let bestB = 0;
  let any = false;
  for (let b = loDeg; b <= hiDeg; b += PROTO_BIN_DEG) {
    const c = liveCone(b, PROTO_BIN_DEG * 0.8);
    if (c.count === 0) continue;
    any = true;
    if (c.median > best) {
      best = c.median;
      bestB = b;
    }
  }
  return { best, bestB, any };
}

// Rotate toward the most-open bearing in SHORT PULSES (never a continuous spin),
// then drive straight. Each cycle: pulse-rotate one small step -> stop & wait for
// a fresh scan -> re-decide. If the whole front is BLOCKED, rotate toward the
// more-open SIDE (searching wider than the 120° front) instead of driving into
// the wall — this is why it stops crashing when open water is beside it.
function updateAutonomousProto() {
  const now = performance.now();
  // 0) EMERGENCY back-up in progress -> keep backing STRAIGHT until the burst
  //    ends. BUT if a wall is closing in BEHIND us, abort the reverse and pulse
  //    FORWARD instead so we don't back into the rear wall.
  if (now < state.nav.reverseUntil) {
    const rear = liveCone(180, PROTO_REAR_TOL_DEG);
    if (rear.count > 0 && rear.min < PROTO_REAR_CM) {
      state.nav.reverseUntil = 0;
      state.nav.fwdUntil = now + PROTO_FWD_PULSE_MS; // escape forward
      state.avoidDir = 0;
      state.cmd.leftSpeed = PROTO_SPEED;
      state.cmd.rightSpeed = PROTO_SPEED;
      return;
    }
    state.avoidDir = 0;
    state.cmd.leftSpeed = -PROTO_SPEED;
    state.cmd.rightSpeed = -PROTO_SPEED;
    return;
  }
  // 0a) Escape-forward pulse in progress (started because a wall was close astern
  //     during a reverse) -> keep driving forward until it ends.
  if (now < state.nav.fwdUntil) {
    state.avoidDir = 0;
    state.cmd.leftSpeed = PROTO_SPEED;
    state.cmd.rightSpeed = PROTO_SPEED;
    return;
  }
  // 0b) Wall closer than PROTO_EMERGENCY_CM DIRECTLY in front of the bow -> back
  //     straight for ~1 second, cancelling any rotate pulse/settle in progress.
  const dead = liveCone(0, PROTO_DEAD_HALF_DEG);
  if (dead.count > 0 && dead.min < PROTO_EMERGENCY_CM) {
    state.nav.reverseUntil = now + PROTO_REVERSE_MS;
    state.nav.spinUntil = 0;
    state.nav.settleUntil = 0;
    state.avoidDir = 0;
    state.cmd.leftSpeed = -PROTO_SPEED;
    state.cmd.rightSpeed = -PROTO_SPEED;
    return;
  }
  // 0c) SIDE-motor burst in progress -> keep running that one motor until it ends.
  //     state.nav.sideDir holds the desired bow-turn direction (away from the wall).
  if (now < state.nav.sideUntil) {
    const sc = protoOneMotorCmd(state.nav.sideDir);
    state.avoidDir = 0;
    state.cmd.leftSpeed = sc.left;
    state.cmd.rightSpeed = sc.right;
    return;
  }
  // 0d) Wall closer than PROTO_SIDE_CM on a SIDE -> run a single motor for ~1s to
  //     turn the bow AWAY from the wall. Left wall -> turn bow right; right wall ->
  //     turn bow left. If both are close, handle the nearer side first.
  const leftClose = liveCone(-90, PROTO_SIDE_TOL_DEG);
  const rightClose = liveCone(90, PROTO_SIDE_TOL_DEG);
  const leftNear = leftClose.count > 0 && leftClose.min < PROTO_SIDE_CM;
  const rightNear = rightClose.count > 0 && rightClose.min < PROTO_SIDE_CM;
  if (leftNear || rightNear) {
    const nearSide = leftNear && (!rightNear || leftClose.min <= rightClose.min) ? -1 : 1;
    const awayDir = -nearSide; // turn the bow away from the near wall
    state.nav.sideDir = awayDir;
    state.nav.sideUntil = now + PROTO_SIDE_MS;
    state.nav.spinUntil = 0;
    state.nav.settleUntil = 0;
    state.avoidDir = 0;
    const sc = protoOneMotorCmd(awayDir);
    state.cmd.leftSpeed = sc.left;
    state.cmd.rightSpeed = sc.right;
    return;
  }
  // 1) A short rotate pulse is in progress -> keep it until it ends.
  if (now < state.nav.spinUntil) {
    const sc = protoSpinCmd(state.nav.spinDir);
    state.avoidDir = 0;
    state.cmd.leftSpeed = sc.left;
    state.cmd.rightSpeed = sc.right;
    return;
  }
  // 2) Post-pulse settle -> STOP so the rotation dies and a fresh sweep arrives
  //    before we look again (this is what stops the over-rotation).
  if (now < state.nav.settleUntil) {
    state.avoidDir = 0;
    state.cmd.leftSpeed = 0;
    state.cmd.rightSpeed = 0;
    return;
  }

  state.avoidDir = 0;
  // 3) Decide. Look at the ±60° frontal arc first.
  const front = protoBestGap(-PROTO_ARC_HALF_DEG, PROTO_ARC_HALF_DEG);
  // No fresh knowledge in front -> brief stop, wait for the sweep (not reverse).
  if (!front.any) {
    state.cmd.leftSpeed = 0;
    state.cmd.rightSpeed = 0;
    return;
  }

  // FRONT BLOCKED: even the most-open front bearing is too close. Do NOT drive
  // into it — rotate (short pulse) toward the more-open SIDE (looked at wider
  // than the front arc) to search for the opening beside us. Keep pulsing until
  // the front opens up, then drive.
  if (front.best < PROTO_CLEAR_CM) {
    const right = protoBestGap(0, PROTO_SIDE_HALF_DEG);
    const left = protoBestGap(-PROTO_SIDE_HALF_DEG, 0);
    const dir = right.best >= left.best ? 1 : -1; // turn toward the more-open side
    startProtoPulse(dir, now);
    return;
  }

  // FRONT OPEN: bow already pointed at the open gap -> DRIVE STRAIGHT.
  if (Math.abs(front.bestB) <= PROTO_ALIGN_DEG) {
    state.cmd.leftSpeed = PROTO_SPEED;
    state.cmd.rightSpeed = PROTO_SPEED;
    return;
  }
  // FRONT OPEN but off to a side -> one short rotate pulse toward it, then re-scan.
  startProtoPulse(front.bestB > 0 ? 1 : -1, now);
}

// Fire ONE short rotate pulse toward dir, then a settle stop before re-deciding.
function startProtoPulse(dir, now) {
  state.nav.spinDir = dir;
  state.nav.spinUntil = now + PROTO_SPIN_PULSE_MS;
  state.nav.settleUntil = now + PROTO_SPIN_PULSE_MS + PROTO_SETTLE_MS;
  const sc = protoSpinCmd(dir);
  state.cmd.leftSpeed = sc.left;
  state.cmd.rightSpeed = sc.right;
}

function updateAutonomousCommand() {
  // Prototype navigator (forward-only, drive-to-the-biggest-front-gap) takes over
  // BOTH in mock and on real hardware when enabled, so it can be evaluated safely
  // in the simulator and behaves identically on the water. The sophisticated
  // sim/real stacks below only run when PROTO_NAV is off.
  if (PROTO_NAV) {
    updateAutonomousProto();
    return;
  }
  // Physical hardware: use the pose-free instantaneous-radar reactive navigator.
  // The accumulated-map / goal-seeking stack below runs only in the simulator.
  if (!state.mockEnabled) {
    updateAutonomousRealworld();
    return;
  }

  if (!state.nav.goal) initNavGoal();
  // הגענו לקצה השני של הבריכה -> עצירה מלאה.
  if (goalDistance() <= GOAL_ARRIVE_CM) {
    state.nav.mode = "arrived";
    state.avoidDir = 0;
    state.nav.escapeUntil = 0;
    state.cmd.leftSpeed = 0;
    state.cmd.rightSpeed = 0;
    return;
  }

  // חילוץ מתקיעה פיזית: קודם-כל בודקים אם הסירה בכלל זזה. סיבוב-במקום נותן תזוזה
  // אפס, כך שהגלאים מבוססי-מרחק-ליעד/עקיבת-דופן שלמטה יכולים להתנדנד לנצח. אם אין
  // תזוזה נטו — נוסעים לאחור בקשת עד שנפתח מרחב, מה שמייצר תזוזה אמיתית ושובר את
  // המלכודת. גובר על כל שאר הלוגיקה כל עוד פרץ-החילוץ פעיל.
  if (updateEscapeManeuver()) return;

  const distFront = getMemoryDistance(0, 45);
  const distLeft = getMemoryDistance(270, 45);
  const distRight = getMemoryDistance(90, 45);
  const goalRel = goalBearingRel();
  // כיוון-שיוט עם משיכה-צדדית ממותנת-מרחק (טיפוס אנכי דרך הפערים; ראה aimBearingRel).
  const aimRel = aimBearingRel();

  // גלאי-תקיעה: מנטר התקדמות נטו אל היעד; עשוי להעביר ל"follow" (עקיבת-דופן).
  updateStuckDetector();

  // מצב עקיבת-דופן (Bug2): מחלץ ממלכודות קעורות/פינות. אם החליט לעזוב הוא מחזיר
  // false ונופלים למטה למצב seek באותו טיק.
  if (state.nav.mode === "follow") {
    if (followWall(distFront, goalRel)) return;
  }

  // --- מצב seek: follow-the-gap מוטה-יעד + התחמקות מיידית ---
  // התחמקות מחויבת: החרטום קרוב מדי -> מסתובב במקום לצד הפתוח עד שהחזית נפתחת
  // (hysteresis), כדי למנוע ריצוד שמאל/ימין כשהצדדים כמעט שווים.
  if (state.avoidDir !== 0) {
    if (distFront >= CLEAR_DISTANCE_CM) {
      state.avoidDir = 0;
    } else {
      applyAvoidTurn(state.avoidDir);
      return;
    }
  }

  if (distFront < SAFE_DISTANCE_CM) {
    // בחירת צד עקיפה מוטת-יעד: במבוך סרפנטינה הפער של המכשול הבא נמצא לרוב בצד
    // ה"סגור" יותר (קרוב ליעד) בעוד המים הפתוחים הם בצד ההפוך. עקיפה עיוורת אל
    // הצד הפתוח שולחת את הסירה הרחק מהפער וגורמת ללולאות. לכן: אם צד-היעד פנוי
    // מספיק — מתחייבים אליו (זוחלים אל הפער); רק אם הוא צר באמת בורחים לצד הפתוח.
    const goalSide = aimRel < 0 ? -1 : 1; // -1 שמאל, +1 ימין (יחסית לחרטום)
    const goalSideDist = goalSide < 0 ? distLeft : distRight;
    if (goalSideDist > SAFE_DISTANCE_CM * 1.3) {
      state.avoidDir = goalSide;
    } else {
      state.avoidDir = distLeft > distRight ? -1 : 1;
    }
    applyAvoidTurn(state.avoidDir);
    return;
  }

  // שיוט: בוחר את הכיוון הפתוח ביותר בקשת החזית, מוטה אל אזימוט היעד. כך הסירה
  // נמשכת אל הקצה השני תוך התחמקות מגופים, במקום לשוטט למים פתוחים סתם.
  const targetBearing = chooseOpenHeading(
    clamp(aimRel, -OPEN_SCAN_DEG, OPEN_SCAN_DEG),
    GOAL_ATTRACT_PENALTY
  );
  const steer = clamp(targetBearing * OPEN_STEER_GAIN, -STEER_MAX, STEER_MAX);

  // אין שליטה במהירות: מהירות-השיוט קבועה (AUTONOMOUS_SPEED). ההיגוי מתבטא רק
  // דרך ה-steer שמעביר מנוע פנימי לעצירה/לאחור אחרי הקוונטיזציה ל-3 מצבים — בדיוק
  // כמו קשת בשליטה ידנית. מהירות ההתקדמות בפועל לעולם אינה מותאמת לפי המרחק.
  const speed = AUTONOMOUS_SPEED;

  // steer > 0 raises the right motor above the left => turns right.
  state.cmd.leftSpeed = clamp(speed - steer, -255, 255);
  state.cmd.rightSpeed = clamp(speed + steer, -255, 255);
}

function updateCommandUI() {
  // המנועים 3-מצביים בלבד (קדימה / אחורה / מנוחה) — גם בשליטה ידנית וגם אוטונומית.
  // מציגים את המצב אחרי הכימות (shapeMotorSpeed) כדי שהתצוגה תשקף בדיוק את מה
  // שהמנוע יריץ בפועל, ללא מהירות-ביניים.
  const label = (v) => {
    const s = shapeMotorSpeed(v);
    return s > 0 ? "קדימה" : s < 0 ? "אחורה" : "מנוחה";
  };
  leftSpeedValue.textContent = label(state.cmd.leftSpeed);
  rightSpeedValue.textContent = label(state.cmd.rightSpeed);
}

// Safety envelope: intercepts outgoing steering commands (manual or autonomous)
// right before they're sent. Scans every accumulated map point and every live
// radar reading for the nearest obstacle; if it breaches the warning/safety
// radius, returns a COPY of the command with a repulsive steering response.
// The base state.cmd is never mutated, so the moment the boat leaves the orange
// zone control returns cleanly to the autonomous navigator / manual joystick.
function computeSafeCommand() {
  const cmd = {
    leftSpeed: state.cmd.leftSpeed,
    rightSpeed: state.cmd.rightSpeed,
    winchSpeed: state.cmd.winchSpeed,
    radarAngle: state.cmd.radarAngle,
  };

  // בשליטה ידנית — מנטרלים (בינתיים) את הגבלת הקרבה: הפקודה נשלחת כמות שהיא
  // ללא דריסת בטיחות. מושל המהירות/ההתחמקות פעילים רק בניווט האוטונומי.
  if (state.manualMode) {
    cmd.leftSpeed = shapeMotorSpeed(cmd.leftSpeed);
    cmd.rightSpeed = shapeMotorSpeed(cmd.rightSpeed);
    return cmd;
  }

  // Prototype navigator is self-contained and FORWARD-ONLY (in both mock and
  // real). Skip every safety override so nothing can inject a reverse/spin; just
  // pass the shaped command through.
  if (PROTO_NAV) {
    cmd.leftSpeed = shapeMotorSpeed(cmd.leftSpeed);
    cmd.rightSpeed = shapeMotorSpeed(cmd.rightSpeed);
    return cmd;
  }

  // Physical hardware: safety comes from the instantaneous radar, not from the
  // pose-projected accumulated map (which drifts with dead reckoning).
  if (!state.mockEnabled) {
    // Prototype navigator is FORWARD-ONLY by request (straight / fwd-right /
    // fwd-left, never reverse). Skip the emergency-reverse safety override so it
    // can't drive the boat backward; just pass the shaped command through.
    if (PROTO_NAV) {
      cmd.leftSpeed = shapeMotorSpeed(cmd.leftSpeed);
      cmd.rightSpeed = shapeMotorSpeed(cmd.rightSpeed);
      return cmd;
    }
    return realworldSafeCommand(cmd);
  }

  const bx = state.pose.x;
  const by = state.pose.y;
  const heading = state.pose.headingDeg;

  let minDist = Infinity;
  let leftThreat = 0;
  let rightThreat = 0;
  let nearestRelBearing = 0; // הזווית היחסית של המכשול הקרוב ביותר (0 = מלפנים, ±180 = מאחור)
  // המרחק הפנוי הקרוב ביותר בחרוט שלפני החרטום — מזין את מושל המהירות. נבדק על
  // טווח רחב יותר מטווח האזהרה, כדי שהסירה תתחיל להאט מבעוד מועד.
  let forwardClearance = Infinity;

  // פונקציית עזר להערכת רמת איום של נקודה במרחב
  const evaluateThreat = (px, py, confidence) => {
    const dist = Math.hypot(px - bx, py - by);

    // חישוב הזווית היחסית לחרטום הסירה
    const absBearing = (Math.atan2(px - bx, py - by) * 180) / Math.PI;
    let relBearing = ((absBearing - heading + 540) % 360) - 180;

    // מרחק פנוי בחרוט הקדמי — למושל המהירות (טווח רחב יותר מהתחמקות).
    if (
      dist <= GOVERNOR_RANGE_CM &&
      Math.abs(relBearing) < GOVERNOR_FWD_CONE_DEG &&
      dist < forwardClearance
    ) {
      forwardClearance = dist;
    }

    if (dist > AVOID_WARNING_RADIUS) return;

    // שמירת הזווית של המכשול הקרוב ביותר, כדי שהבריחה תדע לאיזה כיוון להימלט
    if (dist < minDist) {
      minDist = dist;
      nearestRelBearing = relBearing;
    }

    // רלוונטי רק לעצמים שנמצאים מולנו (100 מעלות לכל כיוון מהחרטום)
    if (Math.abs(relBearing) < 100) {
      // איום גדל ככל שהמכשול קרוב יותר לקו האדום, ומוכפל ברמת הביטחון (Solidness)
      const threatLevel = (AVOID_WARNING_RADIUS - dist) * confidence;

      if (relBearing > 0) {
        rightThreat += threatLevel; // המכשול מימין
      } else {
        leftThreat += threatLevel;  // המכשול משמאל
      }
    }
  };

  // 1. סריקת המפה המצטברת — רק תאים שבהם מספר קשתות נחתכו (לא רפאים)
  for (const cell of mapCells.values()) {
    const confidence = getCellConfidence(cell);
    if (confidence >= MAP_CONFIRM_HITS) {
      evaluateThreat(cell.x, cell.y, confidence);
    }
  }

  // 2. סריקת המכ"ם החי (מקבל משקל גבוה כי אלו נתונים טריים)
  // משתמשים במיקום העולם שקופא ברגע המדידה (entry.wx/wy) ולא בטווח ישן
  // המוקרן מחדש מהפוזה הנוכחית — אחרת רוח-רפאים נגררת עם הסירה ומייצרת תמרוני התחמקות שווא.
  const now = performance.now();
  for (const [absSlot, entry] of radarMemory) {
    if (!entry.value || entry.value >= 999 || (now - entry.t > RADAR_TTL_MS)) continue;
    evaluateThreat(entry.wx, entry.wy, 5); // משקל של 5 למידע חי
  }

  // הכל פנוי — גם החזית פתוחה לגמרי.
  if (minDist > AVOID_WARNING_RADIUS && forwardClearance >= GOVERNOR_RANGE_CM) {
    return cmd;
  }

  // "קדימה" נמדד לפי המהירות הנטו (סכום המנועים), לא לפי מנוע בודד חיובי.
  // סיבוב במקום (למשל 170,-170 של התחמקות אוטונומית) הוא בעל מהירות נטו 0 —
  // הוא אינו מקדם את הסירה לעבר המכשול, ולכן אסור להתייחס אליו כפקודת התקדמות.
  // אחרת שכבת הבטיחות מייצרת היגוי-נגדי שמבטל את הסיבוב, והסירה נתקעת ומסתובבת
  // כמעט במקום מול קיר לנצח (deadlock).
  const netForward = cmd.leftSpeed + cmd.rightSpeed;
  const isForwardCommand = netForward > 20;

  if (minDist <= AVOID_SAFETY_RADIUS) {
    // --- חציית קו אדום: התנגשות מיידית ---
    // קריטי: שכבת הבטיחות מגיבה רק לאיום שנמצא *בכיוון התנועה בפועל*, אחרת היא
    // עלולה לזרוק את הסירה לרוורס אגרסיבי ישר לתוך קיר שמאחוריה ("מלכודת הרוורס").
    if (isForwardCommand && Math.abs(nearestRelBearing) < 90) {
      // ביצוע J-Turn: נסיעה לאחור תוך הפניית החרטום לכיוון הפנוי.
      // רק כשהמכשול הקרוב ביותר באמת מלפנים (±90°) — אחרת הרוורס נכנס לתוך מכשול אחורי.
      // בסימולציה turn = (right - left): heading גדל (פנייה ימינה) כש-right > left.
      if (rightThreat > leftThreat) {
        // סכנה מימין -> החרטום צריך לפנות שמאלה (heading יורד => right < left)
        cmd.leftSpeed = -80;
        cmd.rightSpeed = -180;
      } else {
        // סכנה משמאל -> החרטום צריך לפנות ימינה (heading עולה => right > left)
        cmd.leftSpeed = -180;
        cmd.rightSpeed = -80;
      }
    } else if (netForward < -20 && Math.abs(nearestRelBearing) > 90) {
      // אם הסירה באמת נוסעת רוורס (מהירות נטו שלילית) לתוך משהו שמאחור, בריחה חזקה
      // קדימה. רק כשהמכשול הקרוב ביותר באמת מאחור — אחרת דחיפה קדימה רק תיסע לתוכו.
      cmd.leftSpeed = 150;
      cmd.rightSpeed = 150;
    }
    // מכשול קרוב מלפנים והפקודה אינה קדימה: הסירה כבר מתרחקת (או עומדת),
    // אין צורך בבריחה — משאירים את הפקודה המקורית ולא דוחפים לתוך המכשול.
  } else if (minDist <= AVOID_WARNING_RADIUS && isForwardCommand && Math.abs(nearestRelBearing) < 90) {
    // --- כניסה לטווח הכתום: התחמקות אקטיבית למכשולים מלפנים בלבד ---
    const evasionForce = 1 - ((minDist - AVOID_SAFETY_RADIUS) / (AVOID_WARNING_RADIUS - AVOID_SAFETY_RADIUS));
    const turnSpeed = Math.round(evasionForce * 255);

    // היגוי נגדי על בסיס *סך כל האיומים* ולא רק הנקודה הקרובה ביותר.
    // פנייה שמאלה = מנוע ימין מהיר יותר; פנייה ימינה = מנוע שמאל מהיר יותר.
    if (rightThreat > leftThreat) {
      // אגף ימין חסום יותר -> שבור שמאלה
      cmd.leftSpeed = clamp(cmd.leftSpeed - turnSpeed, -255, 255);
      cmd.rightSpeed = clamp(cmd.rightSpeed + turnSpeed, -255, 255);
    } else {
      // אגף שמאל חסום יותר -> שבור ימינה
      cmd.leftSpeed = clamp(cmd.leftSpeed + turnSpeed, -255, 255);
      cmd.rightSpeed = clamp(cmd.rightSpeed - turnSpeed, -255, 255);
    }
  }

  // --- עצירת התקדמות בינארית (אנטי-נגיחה, ללא שליטה במהירות) ---
  // הניווט האוטונומי מוגבל בדיוק לאותה קשת פקודות כמו השליטה הידנית: קדימה /
  // אחורה / עצירה לכל מנוע — בלי ויסות-מהירות פרופורציונלי. לכן המושל אינו מאט
  // בהדרגה אלא מבטל את רכיב-ההתקדמות המשותף בבת-אחת כשמזוהה גוף בתוך
  // GOVERNOR_STOP_CM: מורידים סכום שווה משני המנועים עד שהמהירות הנטו קדימה
  // מתאפסת. ההפרש בין המנועים נשמר, כך שהסירה עדיין יכולה להסתובב אל הפתוח
  // (בשילוב הקוונטיזציה למטה — פנייה/סיבוב), רק לא לדהור לתוך המכשול. תנועה
  // לאחור אינה מוגבלת (בריחה). פועל גם בשליטה ידנית וגם באוטונומית.
  const netOut = cmd.leftSpeed + cmd.rightSpeed;
  if (netOut > 0 && forwardClearance < GOVERNOR_STOP_CM) {
    const reduce = netOut / 2; // מבטל את מלוא רכיב-ההתקדמות המשותף (הנטו קדימה -> 0)
    cmd.leftSpeed = clamp(cmd.leftSpeed - reduce, -255, 255);
    cmd.rightSpeed = clamp(cmd.rightSpeed - reduce, -255, 255);
  }

  // קוונטיזציה סופית של פקודת המנוע האוטונומית למדרגי הכוח (0 / 70 / 100, עם
  // סימן) — בדיוק כמו בשליטה הידנית ובשרת, כך שהמנוע רץ באותם מדרגים בכל מצב.
  cmd.leftSpeed = shapeMotorSpeed(cmd.leftSpeed);
  cmd.rightSpeed = shapeMotorSpeed(cmd.rightSpeed);

  return cmd;
}

function drawRadar() {
  const w = radarCanvas.width;
  const h = radarCanvas.height;
  const cx = w / 2;
  const cy = h / 2;
  const maxR = Math.min(w, h) * 0.43;

  radarCtx.clearRect(0, 0, w, h);
  radarCtx.fillStyle = "rgba(0, 20, 16, 0.9)";
  radarCtx.fillRect(0, 0, w, h);

  radarCtx.strokeStyle = "rgba(0, 255, 180, 0.25)";
  radarCtx.lineWidth = 1;

  for (let i = 1; i <= 4; i += 1) {
    radarCtx.beginPath();
    radarCtx.arc(cx, cy, (maxR / 4) * i, 0, Math.PI * 2);
    radarCtx.stroke();
  }

  for (let deg = 0; deg < 360; deg += 30) {
    const rad = degToRad(deg);
    radarCtx.beginPath();
    radarCtx.moveTo(cx, cy);
    radarCtx.lineTo(cx + Math.sin(rad) * maxR, cy - Math.cos(rad) * maxR);
    radarCtx.stroke();
  }

  drawBoat(cx, cy, maxR / sim.maxRange);
  drawSensorArcs(cx, cy, maxR);
  drawSensorBeams(cx, cy, maxR);
  drawRangeLabels(cx, cy, maxR);

  // בזמן סיבוב/התייצבות התמונה נבנית מחדש מסריקה טרייה — מסמנים למפעיל שהמכ"ם
  // מתייצב, כדי שהתצוגה הדלילה לא תיראה כתקלה.
  if (!radarPictureStable()) {
    radarCtx.save();
    radarCtx.fillStyle = "rgba(255, 214, 10, 0.92)";
    radarCtx.font = "bold 16px system-ui, sans-serif";
    radarCtx.textAlign = "center";
    radarCtx.textBaseline = "middle";
    radarCtx.fillText("מייצב תמונת מכ\"ם…", cx, cy - maxR - 6);
    radarCtx.restore();
  }

  requestAnimationFrame(drawRadar);
}

// Distance scale: label each range ring with its real-world radius (cm). The
// rings sit at 1/4 steps of maxR, which maps to sim.maxRange cm, so ring i marks
// (sim.maxRange / 4) * i. Labels are placed on the north (up) axis and drawn
// last so they stay readable on top of the sensor beams.
function drawRangeLabels(cx, cy, maxR) {
  radarCtx.save();
  radarCtx.font = '11px system-ui, sans-serif';
  radarCtx.textAlign = "center";
  radarCtx.textBaseline = "middle";
  for (let i = 1; i <= 4; i += 1) {
    const r = (maxR / 4) * i;
    const cm = Math.round((sim.maxRange / 4) * i);
    const label = `${cm} ס"מ`;
    const ty = cy - r;
    const tw = radarCtx.measureText(label).width;
    radarCtx.fillStyle = "rgba(0, 20, 16, 0.85)";
    radarCtx.fillRect(cx - tw / 2 - 3, ty - 8, tw + 6, 16);
    radarCtx.fillStyle = "rgba(120, 255, 210, 0.9)";
    radarCtx.fillText(label, cx, ty);
  }
  radarCtx.restore();
}

// Top-down "ground truth" of the mock world: boundary walls, obstacle bodies,
// the boat at its real pose, and the circle marking the ultrasonic range. Lets
// the operator compare the actual world against the radar reconstruction.
function drawWorld() {
  const w = worldCanvas.width;
  const h = worldCanvas.height;

  worldCtx.clearRect(0, 0, w, h);
  worldCtx.fillStyle = "rgba(3, 20, 30, 0.95)";
  worldCtx.fillRect(0, 0, w, h);

  if (!world.loaded || !state.mockEnabled) {
    worldCtx.fillStyle = "rgba(150, 200, 220, 0.7)";
    worldCtx.font = "16px system-ui, sans-serif";
    worldCtx.textAlign = "center";
    worldCtx.fillText("הפעל מצב מוק דאטא לצפייה בעולם", w / 2, h / 2);
    worldPosValue.textContent = "--";
    worldInRangeValue.textContent = "--";
    requestAnimationFrame(drawWorld);
    return;
  }

  // Fit the whole world rectangle into the canvas (with a small margin) and
  // keep the boat centred so it never scrolls out of view.
  const margin = 16;
  const bx = state.telemetry.boatX ?? 0;
  const by = state.telemetry.boatY ?? 0;
  const heading = state.telemetry.boatHeadingDeg ?? 0;
  const scale = Math.min(
    (w - 2 * margin) / (2 * world.boundsHalfX),
    (h - 2 * margin) / (2 * world.boundsHalfY)
  );
  const originX = w / 2;
  const originY = h / 2;
  // World (wx, wy) -> screen. World +y is north (up); canvas y grows down.
  const toScreen = (wx, wy) => ({
    x: originX + (wx - bx) * scale,
    y: originY - (wy - by) * scale,
  });

  // Boundary walls
  const tl = toScreen(-world.boundsHalfX, world.boundsHalfY);
  const br = toScreen(world.boundsHalfX, -world.boundsHalfY);
  worldCtx.strokeStyle = "rgba(120, 170, 200, 0.8)";
  worldCtx.lineWidth = 2;
  worldCtx.strokeRect(tl.x, tl.y, br.x - tl.x, br.y - tl.y);

  const boat = toScreen(bx, by);
  const rangePx = world.maxRange * scale;

  // Radar range circle
  worldCtx.strokeStyle = "rgba(44, 255, 197, 0.55)";
  worldCtx.setLineDash([6, 6]);
  worldCtx.lineWidth = 1.5;
  worldCtx.beginPath();
  worldCtx.arc(boat.x, boat.y, rangePx, 0, Math.PI * 2);
  worldCtx.stroke();
  worldCtx.setLineDash([]);

  // Obstacle bodies; highlight the ones the sensors could actually reach.
  let inRange = 0;
  // Thin straight-line walls (baffles). Drawn as thick segments at their true
  // 5 cm width so the ground-truth view matches the "obstacles are lines" model.
  for (const wall of world.walls) {
    const a = toScreen(wall.x1, wall.y1);
    const b = toScreen(wall.x2, wall.y2);
    // In range if any point on the segment is within sensor reach of the boat.
    const midDist = Math.hypot((wall.x1 + wall.x2) / 2 - bx, (wall.y1 + wall.y2) / 2 - by);
    const within = midDist - Math.hypot(wall.x2 - wall.x1, wall.y2 - wall.y1) / 2 <= world.maxRange;
    if (within) inRange += 1;
    worldCtx.beginPath();
    worldCtx.moveTo(a.x, a.y);
    worldCtx.lineTo(b.x, b.y);
    worldCtx.lineCap = "round";
    worldCtx.lineWidth = Math.max(2, (wall.thickness ?? 5) * scale);
    worldCtx.strokeStyle = within ? "rgba(44, 255, 197, 0.85)" : "rgba(120, 140, 150, 0.5)";
    worldCtx.stroke();
  }
  worldCtx.lineCap = "butt";
  for (const t of world.targets) {
    const p = toScreen(t.x, t.y);
    const dist = Math.hypot(t.x - bx, t.y - by);
    const within = dist - t.radius <= world.maxRange;
    if (within) inRange += 1;
    worldCtx.beginPath();
    worldCtx.arc(p.x, p.y, Math.max(2, t.radius * scale), 0, Math.PI * 2);
    worldCtx.fillStyle = within ? "rgba(44, 255, 197, 0.85)" : "rgba(120, 140, 150, 0.5)";
    worldCtx.fill();
  }

  // Start / goal markers for scenarios that define them (e.g. the serpentine).
  if (world.start) {
    const s = toScreen(world.start.x, world.start.y);
    worldCtx.strokeStyle = "rgba(120, 200, 255, 0.9)";
    worldCtx.lineWidth = 2;
    worldCtx.beginPath();
    worldCtx.arc(s.x, s.y, 7, 0, Math.PI * 2);
    worldCtx.stroke();
  }
  if (world.goal) {
    const g = toScreen(world.goal.x, world.goal.y);
    worldCtx.fillStyle = "rgba(255, 99, 132, 0.9)";
    worldCtx.beginPath();
    worldCtx.arc(g.x, g.y, 6, 0, Math.PI * 2);
    worldCtx.fill();
    worldCtx.strokeStyle = "rgba(255, 99, 132, 0.6)";
    worldCtx.lineWidth = 2;
    worldCtx.beginPath();
    worldCtx.arc(g.x, g.y, 11, 0, Math.PI * 2);
    worldCtx.stroke();
  }

  // Sensor beam directions in the world frame (heading + servo sweep + offset).
  // Use the effective sweep angle (boat-driven telemetry on real hardware,
  // commanded on mock) so the beams track the physical servo.
  const sweep = sweepAngle();
  worldCtx.strokeStyle = "rgba(255, 241, 118, 0.35)";
  worldCtx.lineWidth = 1;
  for (const beam of SENSOR_BEAMS) {
    const rad = degToRad(heading + sweep + beam.dir);
    worldCtx.beginPath();
    worldCtx.moveTo(boat.x, boat.y);
    worldCtx.lineTo(boat.x + Math.sin(rad) * rangePx, boat.y - Math.cos(rad) * rangePx);
    worldCtx.stroke();
  }

  // Boat, rotated to its real heading (0 = north / up, clockwise positive).
  // Drawn to true scale: a 30 cm × 30 cm hull (±15 cm from its centre).
  worldCtx.save();
  worldCtx.translate(boat.x, boat.y);
  worldCtx.rotate(degToRad(heading));
  worldCtx.fillStyle = "rgba(255, 241, 118, 0.95)";
  worldCtx.beginPath();
  worldCtx.moveTo(0, -15 * scale);
  worldCtx.lineTo(15 * scale, 15 * scale);
  worldCtx.lineTo(0, 6 * scale);
  worldCtx.lineTo(-15 * scale, 15 * scale);
  worldCtx.closePath();
  worldCtx.fill();
  worldCtx.restore();

  worldPosValue.textContent = `${Math.round(bx)}, ${Math.round(by)} · ${Math.round(heading)}°`;
  worldInRangeValue.textContent = String(inRange);

  requestAnimationFrame(drawWorld);
}

// Fold each sensor reading into the accumulated grid map as an ARC (the beam is
// ~15° wide, so the true obstacle can lie anywhere along that arc at the
// measured range). Also clears the empty cone in front of the arc and records
// the boat's path (trail). Cells only become "real" where arcs intersect.
function accumulateRadarMap() {
  const sweep = sweepAngle();
  const heading = state.pose.headingDeg;
  const bx = state.pose.x;
  const by = state.pose.y;

  // Trail: append when the boat has moved a little, to keep the array bounded.
  const last = boatTrail[boatTrail.length - 1];
  if (!last || Math.hypot(last.x - bx, last.y - by) > 3) {
    boatTrail.push({ x: bx, y: by });
    if (boatTrail.length > MAP_TRAIL_MAX) boatTrail.shift();
  }

  const boatKx = Math.round(bx / MAP_CELL_CM);
  const boatKy = Math.round(by / MAP_CELL_CM);

  for (const beam of SENSOR_BEAMS) {
    const dist = state.telemetry[beam.key];
    if (dist == null) continue;

    // Skip a duplicate look (same beam, servo step, boat cell and range) so a
    // stationary boat can't pump a lone arc past the intersection threshold.
    const distBucket = dist >= 999 ? 999 : Math.round(dist / MAP_CELL_CM);
    const sig = `${sweep}:${boatKx}:${boatKy}:${distBucket}`;
    if (lastArcObs.get(beam.key) === sig) continue;
    lastArcObs.set(beam.key, sig);

    // Absolute world bearing of the beam centre = heading + servo sweep + offset.
    const center = heading + sweep + beam.dir;
    const clearTo = dist >= 999 ? sim.maxRange : dist;

    // Empty cone: every ray across the beam width, up to just before the arc, is
    // confirmed free water. Collect the cells once (a Set) so overlapping rays
    // don't decrement the same cell several times in a single frame.
    const clearKeys = new Set();
    for (let a = -MAP_ARC_FOV_DEG; a <= MAP_ARC_FOV_DEG; a += MAP_ARC_STEP_DEG) {
      collectRayCells(bx, by, degToRad(center + a), clearTo, clearKeys);
    }
    for (const key of clearKeys) {
      const cell = mapCells.get(key);
      if (cell) {
        cell.hits -= MAP_CLEAR_DECAY;
        if (cell.hits <= 0) mapCells.delete(key);
      }
    }

    // A 999 reading only clears space — there is no obstacle arc to record.
    if (dist >= 999) continue;

    // Occupied arc: paint every cell the arc passes through, but only ONCE per
    // arc (a Set), so a single arc contributes at most its (range-scaled) weight
    // to each cell and thus stays below MAP_CONFIRM_HITS until another arc
    // crosses it. Far arcs weigh less than near ones (see arcCellWeight).
    const weight = arcCellWeight(dist);
    const arcKeys = new Set();
    for (let a = -MAP_ARC_FOV_DEG; a <= MAP_ARC_FOV_DEG; a += MAP_ARC_STEP_DEG) {
      const rad = degToRad(center + a);
      const wx = bx + Math.sin(rad) * dist;
      const wy = by + Math.cos(rad) * dist;
      const kx = Math.round(wx / MAP_CELL_CM);
      const ky = Math.round(wy / MAP_CELL_CM);
      const key = `${kx},${ky}`;
      if (arcKeys.has(key)) continue;
      arcKeys.add(key);
      markArcCell(wx, wy, kx, ky, key, weight);
    }
  }
}

// Weight = MAP_ARC_REF_CM / range, so a reading at MAP_ARC_REF_CM scores 1.0,
// nearer readings score MORE (a tight, well-localised beam earns extra trust and
// can confirm on its own), and farther readings score LESS. Capped at
// MAP_ARC_MAX_WEIGHT so a near arc confirms in one look, floored so distant arcs
// still smear thinly and must intersect before a lone far echo is believed.
function arcCellWeight(dist) {
  return Math.min(MAP_ARC_MAX_WEIGHT, MAP_ARC_REF_CM / Math.max(dist, 1));
}

// Add a single arc's contribution to one grid cell. Each arc adds at most its
// range-scaled `weight`. A weighted running average keeps the stored point near
// the true intersecting surface, and total weight is capped so ray-clearing can
// still erode a cell that goes empty.
function markArcCell(wx, wy, kx, ky, key, weight) {
  const cell = mapCells.get(key);
  if (cell) {
    const newHits = Math.min(MAP_HIT_CAP, cell.hits + weight);
    const added = newHits - cell.hits;
    if (added > 0) {
      cell.x += (wx - cell.x) * (added / newHits);
      cell.y += (wy - cell.y) * (added / newHits);
      cell.hits = newHits;
    }
  } else if (mapCells.size < MAP_MAX_CELLS) {
    mapCells.set(key, { x: wx, y: wy, hits: weight, kx, ky });
  } else {
    // Map is full: rather than silently refusing ALL new cells (which would
    // blind the boat to obstacles in freshly explored ground), evict the
    // weakest existing cell — typically a low-confidence "ghost" — to make room,
    // but only if the newcomer is at least as strong as it.
    let weakestKey = null;
    let weakestHits = Infinity;
    for (const [k, c] of mapCells) {
      if (c.hits < weakestHits) {
        weakestHits = c.hits;
        weakestKey = k;
      }
    }
    if (weakestKey !== null && weakestHits <= weight) {
      mapCells.delete(weakestKey);
      mapCells.set(key, { x: wx, y: wy, hits: weight, kx, ky });
    }
  }
}

// Ray clearing (inverse sensor model): the straight line between the boat and a
// hit point must be empty water, otherwise the beam would have bounced sooner.
// Collect every cell along that free segment into `out`; a margin of one cell
// before the hit is left out so the real surface is never eroded by its own arc.
function collectRayCells(bx, by, rad, clearDist, out) {
  const sinR = Math.sin(rad);
  const cosR = Math.cos(rad);
  const end = clearDist - MAP_CELL_CM;
  for (let t = MAP_CELL_CM; t < end; t += MAP_CELL_CM) {
    const kx = Math.round((bx + sinR * t) / MAP_CELL_CM);
    const ky = Math.round((by + cosR * t) / MAP_CELL_CM);
    out.add(`${kx},${ky}`);
  }
}

function clearRadarMap() {
  mapCells.clear();
  boatTrail.length = 0;
  lastArcObs.clear();
}

// Arc-intersection confidence: with arc mapping each DISTINCT arc adds at most
// +1 to a cell, so cell.hits is literally the number of independent arcs that
// crossed it. A lone arc leaves 1; a genuine obstacle where arcs intersect
// climbs to >= MAP_CONFIRM_HITS. Returning the raw hit count keeps that meaning
// intact (a neighbourhood sum would wrongly boost a single arc's straight tail).
function getCellConfidence(cell) {
  return cell.hits;
}

// World-fixed reconstruction: draws the accumulated grid, the boat's trail and
// its live pose, auto-fitting the view to everything collected so far. North is
// up (unlike the boat-centric radar view, this frame does NOT rotate).
function drawMap() {
  const w = mapCanvas.width;
  const h = mapCanvas.height;

  mapCtx.clearRect(0, 0, w, h);
  mapCtx.fillStyle = "rgba(3, 16, 24, 0.95)";
  mapCtx.fillRect(0, 0, w, h);

  mapPointsValue.textContent = String(mapCells.size);

  if (mapCells.size === 0) {
    mapCtx.fillStyle = "rgba(150, 200, 220, 0.7)";
    mapCtx.font = "16px system-ui, sans-serif";
    mapCtx.textAlign = "center";
    mapCtx.fillText('נוע עם הסירה כדי לבנות מפה מנתוני המכ"ם', w / 2, h / 2);
    requestAnimationFrame(drawMap);
    return;
  }

  const bx = state.pose.x;
  const by = state.pose.y;

  // Fit bounds around all cells, the trail and the boat.
  let minX = Infinity;
  let minY = Infinity;
  let maxX = -Infinity;
  let maxY = -Infinity;
  const consider = (x, y) => {
    if (x < minX) minX = x;
    if (x > maxX) maxX = x;
    if (y < minY) minY = y;
    if (y > maxY) maxY = y;
  };
  for (const c of mapCells.values()) consider(c.x, c.y);
  for (const p of boatTrail) consider(p.x, p.y);
  consider(bx, by);

  const margin = 22;
  const minSpan = 200;
  const spanX = Math.max(maxX - minX, minSpan);
  const spanY = Math.max(maxY - minY, minSpan);
  const cxWorld = (minX + maxX) / 2;
  const cyWorld = (minY + maxY) / 2;
  const scale = Math.min((w - 2 * margin) / spanX, (h - 2 * margin) / spanY);
  const originX = w / 2;
  const originY = h / 2;
  // World +y is north (up); canvas y grows down.
  const toScreen = (wx, wy) => ({
    x: originX + (wx - cxWorld) * scale,
    y: originY - (wy - cyWorld) * scale,
  });

  // Faint 100 cm grid for scale reference.
  mapCtx.strokeStyle = "rgba(80, 130, 150, 0.18)";
  mapCtx.lineWidth = 1;
  const gridCm = 100;
  const gx0 = Math.floor((cxWorld - spanX / 2) / gridCm) * gridCm;
  const gx1 = Math.ceil((cxWorld + spanX / 2) / gridCm) * gridCm;
  for (let gx = gx0; gx <= gx1; gx += gridCm) {
    const a = toScreen(gx, cyWorld - spanY);
    const b = toScreen(gx, cyWorld + spanY);
    mapCtx.beginPath();
    mapCtx.moveTo(a.x, a.y);
    mapCtx.lineTo(b.x, b.y);
    mapCtx.stroke();
  }
  const gy0 = Math.floor((cyWorld - spanY / 2) / gridCm) * gridCm;
  const gy1 = Math.ceil((cyWorld + spanY / 2) / gridCm) * gridCm;
  for (let gy = gy0; gy <= gy1; gy += gridCm) {
    const a = toScreen(cxWorld - spanX, gy);
    const b = toScreen(cxWorld + spanX, gy);
    mapCtx.beginPath();
    mapCtx.moveTo(a.x, a.y);
    mapCtx.lineTo(b.x, b.y);
    mapCtx.stroke();
  }

  // Boat trail.
  if (boatTrail.length > 1) {
    mapCtx.strokeStyle = "rgba(255, 241, 118, 0.45)";
    mapCtx.lineWidth = 1.5;
    mapCtx.beginPath();
    boatTrail.forEach((p, i) => {
      const s = toScreen(p.x, p.y);
      if (i === 0) mapCtx.moveTo(s.x, s.y);
      else mapCtx.lineTo(s.x, s.y);
    });
    mapCtx.stroke();
  }

  // Accumulated detections; brighter = more arcs intersect there. Lone-arc
  // cells (below MAP_CONFIRM_HITS) are drawn as faint grey "ghosts" so the
  // operator still sees them but knows the navigation logic is ignoring them.
  for (const c of mapCells.values()) {
    const s = toScreen(c.x, c.y);
    const confidence = getCellConfidence(c);
    const isGhost = confidence < MAP_CONFIRM_HITS;
    const alpha = isGhost ? 0.1 : Math.min(1, 0.3 + confidence * 0.1);
    mapCtx.fillStyle = isGhost
      ? `rgba(150, 150, 150, ${alpha})`
      : `rgba(44, 255, 197, ${alpha})`;
    const size = Math.max(2, MAP_CELL_CM * scale * 0.5);
    mapCtx.beginPath();
    mapCtx.arc(s.x, s.y, size, 0, Math.PI * 2);
    mapCtx.fill();
  }

  // Boat at its live pose, rotated to heading (0 = north / up).
  // Drawn to true scale: a 30 cm × 30 cm hull (±15 cm from its centre).
  const boat = toScreen(bx, by);
  mapCtx.save();
  mapCtx.translate(boat.x, boat.y);
  mapCtx.rotate(degToRad(state.pose.headingDeg));
  mapCtx.fillStyle = "rgba(255, 241, 118, 0.95)";
  mapCtx.beginPath();
  mapCtx.moveTo(0, -15 * scale);
  mapCtx.lineTo(15 * scale, 15 * scale);
  mapCtx.lineTo(0, 6 * scale);
  mapCtx.lineTo(-15 * scale, 15 * scale);
  mapCtx.closePath();
  mapCtx.fill();
  mapCtx.restore();

  // שרטוט טווחי בטיחות והתחמקות סביב הסירה
  const sBoat = toScreen(bx, by);

  // מעגל פנייה/הדיפה (כתום מקווקו)
  mapCtx.strokeStyle = "rgba(255, 165, 0, 0.4)";
  mapCtx.setLineDash([4, 4]);
  mapCtx.lineWidth = 1.5;
  mapCtx.beginPath();
  mapCtx.arc(sBoat.x, sBoat.y, AVOID_WARNING_RADIUS * scale, 0, Math.PI * 2);
  mapCtx.stroke();
  mapCtx.setLineDash([]);

  // מעגל סכנה (אדום)
  mapCtx.strokeStyle = "rgba(255, 100, 100, 0.4)";
  mapCtx.beginPath();
  mapCtx.arc(sBoat.x, sBoat.y, AVOID_SAFETY_RADIUS * scale, 0, Math.PI * 2);
  mapCtx.stroke();

  requestAnimationFrame(drawMap);
}

function drawBoat(cx, cy, pxPerCm) {
  // Drawn to true scale: a 30 cm × 30 cm hull (±15 cm from its centre).
  const s = pxPerCm;
  radarCtx.save();
  radarCtx.translate(cx, cy);
  // Boat always points up - no rotation
  radarCtx.fillStyle = "rgba(255, 241, 118, 0.95)";
  radarCtx.beginPath();
  radarCtx.moveTo(0, -15 * s);
  radarCtx.lineTo(15 * s, 15 * s);
  radarCtx.lineTo(0, 6 * s);
  radarCtx.lineTo(-15 * s, 15 * s);
  radarCtx.closePath();
  radarCtx.fill();
  radarCtx.restore();
}

// All 4 ultrasonic sensors sit 90° apart on ONE servo axis and rotate together.
// dir = their bow-relative bearing at servo home (0); the live bearing adds the
// current servo angle (radarAngle). Boat is drawn pointing up (front = 0°).
const SENSOR_BEAMS = [
  { dir: 0, key: "usFront", color: "rgba(44, 255, 197, 1)", masked: true },
  { dir: 90, key: "usRight", color: "rgba(255, 150, 100, 1)" },
  { dir: 180, key: "usRadar", color: "rgba(200, 120, 255, 1)" },
  { dir: 270, key: "usLeft", color: "rgba(100, 200, 255, 1)" },
];

// חיישן החזית (usFront / echo A0) פגום: כשהחרטום פתוח הוא מחזיר ~65 ס"מ כמעט-קבוע
// (החזר ממשטח המים כי הוא מוטה מעט מטה). אבל חיישן אולטרסוני מדווח את ההד הקרוב
// ביותר, ולכן קיר אמיתי קרוב מ~65 ס"מ *כן* מדווח נכון. לכן: לניווט הוא לא ממוסך
// (RW_BLOCK_CM מוגדר מתחת לרצועת החזר-המים כך שמים פתוחים לא יבלמו שקר, אבל קיר
// קרוב מלפנים כן יעצור ויסע לאחור). לתצוגה בלבד הוא ממוסך (MASK_FRONT_DISPLAY) כדי
// שהחזר-המים בפתוח לא יצייר "קיר מדומה" על המכ"ם. אחרי תיקון החומרה — false.
const MASK_FRONT_DISPLAY = true;

// Radar persistence: each boat-relative angle slot keeps its last scan for a
// short while (RADAR_TTL_MS). Fresh sweeps refresh it; stale readings expire so
// that blips left behind by boat movement/rotation don't linger as phantom
// rings or arcs that never existed in the world.
const radarMemory = new Map();

// Gap-fill bookkeeping: the last received servo angle + per-sensor value + time,
// so a newly arrived packet can paint the angular arc swept since the previous
// one (real-HW only; see updateRadarMemory).
let prevSweepDeg = null;
let prevSweepT = 0;
const prevBeamVal = {};

// Wall-clock (performance.now) of the last GENUINE telemetry frame folded into
// the radar picture. Only updated inside updateRadarMemory, which runs solely
// when the boat is connected/mock, so a comms dropout stops it advancing.
let lastRadarFreshAt = 0;

// Display ageing clock. During normal operation it IS performance.now(), so the
// radar TTLs work exactly as before. When telemetry stops arriving for longer
// than RADAR_FREEZE_AFTER_MS (comms loss), it freezes at that point so the last
// picture (walls + blips) stays on screen instead of ageing out to blank; it
// resumes the moment a fresh frame arrives. This is a DISPLAY concern only — the
// navigator keeps using real time / liveScan freshness and never acts on frozen
// data as if it were live.
function radarClock() {
  const real = performance.now();
  if (lastRadarFreshAt && real - lastRadarFreshAt > RADAR_FREEZE_AFTER_MS) {
    return lastRadarFreshAt + RADAR_FREEZE_AFTER_MS;
  }
  return real;
}

// Fitted wall segments, one per side (0=front,1=right,2=back,3=left sensor
// sector). Each holds the two endpoints of the straight line fitted to that
// side's recent points, FROZEN in world coordinates (like radarMemory) plus a
// timestamp. Recomputed every telemetry frame from the live raw points; drawn as
// a solid, gap-free line and kept alive for WALL_TTL_MS so it doesn't flicker
// when a single 15° slot is momentarily missed by the slow real-HW sweep.
const radarWalls = new Map();

function resetRadarWalls() {
  radarWalls.clear();
  prevSweepDeg = null;
  prevSweepT = 0;
  for (const k of Object.keys(prevBeamVal)) delete prevBeamVal[k];
}

// בודק אם קבוצת נקודות היא "טבעת"-רפאים ולא קיר שטוח. קלט: xy במסגרת-חרטום עם
// שדה .b (אזימוט יחסי, מעלות). קיר שטוח על פני קשת רחבה מראה טווח שגדל לעבר
// הקצוות (∝1/cos מהניצב); הד-רפאים מחזיר טווח כמעט-קבוע. מפעילים רק על קשת רחבה
// (>= RADAR_RING_ANG_MIN_DEG). מחזיר true אם הטווח בקצוות אינו רחוק מהמרכז לפחות
// פי RADAR_RING_EDGE_RATIO — כלומר טבעת-רפאים שאין לצייר.
function isPhantomRing(xy) {
  if (xy.length < 4) return false; // מעט מדי נקודות כדי להכריע — לא דוחים
  const pts = xy.map((q) => ({ b: q.b, r: Math.hypot(q.x, q.y) }));
  let bMin = Infinity;
  let bMax = -Infinity;
  for (const p of pts) {
    if (p.b < bMin) bMin = p.b;
    if (p.b > bMax) bMax = p.b;
  }
  const span = bMax - bMin;
  if (span < RADAR_RING_ANG_MIN_DEG) return false; // קשת צרה — משאירים ללוגיקה הרגילה
  const mid = (bMin + bMax) / 2;
  const median = (arr) => {
    if (!arr.length) return null;
    const s = [...arr].sort((a, b) => a - b);
    return s[s.length >> 1];
  };
  const centerR = median(pts.filter((p) => Math.abs(p.b - mid) <= 10).map((p) => p.r));
  const edgeR = median(pts.filter((p) => Math.abs(p.b - mid) >= span * 0.35).map((p) => p.r));
  if (centerR == null || edgeR == null) return false;
  // קיר שטוח: הקצה רחוק מהמרכז (edgeR גדול). טבעת: edgeR ≈ centerR (או קטן) -> רפאים.
  return edgeR < centerR * RADAR_RING_EDGE_RATIO;
}

// Per-side linear interpolation. Groups the currently-live raw detections by
// sensor side (each sensor owns its own ~75° arc), fits ONE straight line per
// side (orthogonal / total-least-squares, so a wall at any orientation works,
// including one seen edge-on), rejects sides that aren't actually a straight
// surface, and publishes:
//   * a world-space wall SEGMENT (radarWalls) for the display, and
//   * range corrected onto the line (liveScan bins) for the autonomous nav.
// It reads radarMemory RAW and never writes back to it, so the fit can't feed on
// its own previous output (no drift). Called at the end of every updateRadarMemory.
function updateRadarWalls() {
  const now = performance.now();
  const heading = state.pose.headingDeg;
  // Real HW: the radar picture is BOAT-FIXED — the dots are drawn at their
  // measured range and deliberately do NOT slide as the boat advances. So the
  // walls must ignore the boat's TRANSLATION too, otherwise the world-frozen
  // segment drifts off the very returns it was fit from (heading is already
  // handled identically for dots and walls, so only the position matters). Zero
  // the boat position here (and matchingly in drawSensorArcs) so walls live in
  // the same boat-fixed frame as the dots. The simulator keeps the true pose to
  // accumulate a genuine world map.
  const bx = state.mockEnabled ? state.pose.x : 0;
  const by = state.mockEnabled ? state.pose.y : 0;
  const bowOff = state.nav.bowOffsetDeg ?? BOW_SERVO_OFFSET_DEG;

  // Bucket live, valid detections into the 4 sensor sides. slot - heading is
  // heading-independent (all sensors rotate with the boat), so a side always
  // collects the same sensor's arc even while turning. Only REAL measured
  // returns feed the geometry (filled gap-fill points are display-only blips).
  // Each point is flagged `fresh` (<= RADAR_TTL_MS) so only genuinely fresh
  // returns feed the nav-facing liveScan below — autonomy never acts on a stale
  // bearing.
  const sides = [[], [], [], []];
  for (const [slot, e] of radarMemory) {
    if (e.filled) continue;
    if (!e.value || e.value >= 999 || now - e.t > RADAR_WALL_FIT_TTL_MS) continue;
    if (e.value > sim.maxRange) continue;
    let rel;
    let r;
    if (state.mockEnabled && e.wx != null) {
      // Derive the point's CURRENT bow-relative bearing/range from its FROZEN
      // world position (e.wx/wy) reprojected from the current pose — exactly like
      // the world-anchored dots. Using the stored (slot - heading, value) instead
      // places the point at its range from the OLD pose, so as the boat TRANSLATES
      // the fit warps and the walls drift off the dots (the "distortion during
      // movement"). Now walls + dots share one world frame and stay put while the
      // boat moves.
      const dx = e.wx - bx;
      const dy = e.wy - by;
      r = Math.hypot(dx, dy);
      rel = normalizeDeg((Math.atan2(dx, dy) * 180) / Math.PI - heading);
    } else {
      rel = normalizeDeg(slot - heading); // 0..360 from bow (boat-fixed, real HW)
      r = e.value;
    }
    const side = Math.floor(rel / 90) % 4;
    sides[side].push({ bearingRel: wrap180(rel), r, fresh: now - e.t <= RADAR_TTL_MS });
  }

  // --- PHASE 1: per-side geometry stats (bow-frame cartesian) --------------
  // The 4 ultrasonic sensors are mounted 90° apart, so in a rectangular basin
  // the walls they see are constrained to lie along a common grid (each wall is
  // parallel or perpendicular to the others). We exploit that: first measure
  // each side's raw orientation independently, then in PHASE 2 fuse them into a
  // single grid orientation φ, so even a side with only 1–2 points can be placed
  // by borrowing the orientation the OTHER sensors agreed on.
  const stat = [null, null, null, null];
  for (let side = 0; side < 4; side += 1) {
    const pts = sides[side];
    if (pts.length === 0) continue;
    const xy = pts.map((p) => {
      const a = degToRad(p.bearingRel);
      return { b: p.bearingRel, x: Math.sin(a) * p.r, y: Math.cos(a) * p.r, fresh: p.fresh };
    });
    let mx = 0;
    let my = 0;
    let bs = 0;
    let bc = 0;
    for (const q of xy) {
      mx += q.x;
      my += q.y;
      bs += Math.sin(degToRad(q.b));
      bc += Math.cos(degToRad(q.b));
    }
    mx /= xy.length;
    my /= xy.length;
    const repBearing = Math.atan2(bs, bc); // circular-mean view bearing (rad)
    let theta = null;
    let res = 0;
    if (xy.length >= 2) {
      let Sxx = 0;
      let Syy = 0;
      let Sxy = 0;
      for (const q of xy) {
        const dx = q.x - mx;
        const dy = q.y - my;
        Sxx += dx * dx;
        Syy += dy * dy;
        Sxy += dx * dy;
      }
      theta = 0.5 * Math.atan2(2 * Sxy, Sxx - Syy);
      const nnx = -Math.sin(theta);
      const nny = Math.cos(theta);
      for (const q of xy) res += ((q.x - mx) * nnx + (q.y - my) * nny) ** 2;
      res = Math.sqrt(res / xy.length);
    }
    stat[side] = { xy, count: xy.length, theta, res, repBearing };
  }

  // --- PHASE 2: shared grid orientation φ ---------------------------------
  // ROBUST estimate from NEARBY POINT PAIRS: two points on the same wall define
  // that wall's direction, so folding each close pair's edge angle ×4 (the
  // axis-aligned grid repeats every 90°) and taking the circular mean yields ONE
  // orientation the whole world snaps to. This is far steadier than averaging
  // the 4 per-side TLS fits — that averaging is exactly what left the walls at
  // inconsistent angles (the "picture not good enough"). Falls back to the
  // per-side estimate only when too few nearby pairs exist (very sparse frame).
  const allPts = [];
  for (let side = 0; side < 4; side += 1) {
    if (stat[side]) for (const q of stat[side].xy) allPts.push(q);
  }
  let Psin = 0;
  let Pcos = 0;
  let pairN = 0;
  const PAIR_MAX2 = 45 * 45; // only pairs within ~45 cm are "same wall" evidence
  for (let i = 0; i < allPts.length; i += 1) {
    for (let j = i + 1; j < allPts.length; j += 1) {
      const dpx = allPts[j].x - allPts[i].x;
      const dpy = allPts[j].y - allPts[i].y;
      const d2 = dpx * dpx + dpy * dpy;
      if (d2 < 1 || d2 > PAIR_MAX2) continue;
      const th = Math.atan2(dpy, dpx);
      Psin += Math.sin(4 * th);
      Pcos += Math.cos(4 * th);
      pairN += 1;
    }
  }
  // Per-side fallback (original method): circular mean of the per-side TLS
  // orientations folded ×4, weighted by point count.
  let Ssin = 0;
  let Scos = 0;
  for (let side = 0; side < 4; side += 1) {
    const s = stat[side];
    if (!s || s.theta === null) continue;
    if (s.count >= 3 && s.res > RADAR_FIT_MAX_RESIDUAL_CM) continue;
    Ssin += s.count * Math.sin(4 * s.theta);
    Scos += s.count * Math.cos(4 * s.theta);
  }
  // Prefer the robust pair estimate; fall back to per-side only when too sparse.
  const phi =
    pairN >= 4 && (Psin !== 0 || Pcos !== 0)
      ? Math.atan2(Psin, Pcos) / 4
      : Ssin !== 0 || Scos !== 0
      ? Math.atan2(Ssin, Scos) / 4
      : null;

  // Snap an orientation onto the nearest of the two grid axes {φ, φ+90°}.
  const snapToGrid = (thetaRad) => {
    let best = phi;
    let bestD = Infinity;
    for (const cand of [phi, phi + Math.PI / 2]) {
      let d = ((thetaRad - cand) % Math.PI + Math.PI) % Math.PI;
      if (d > Math.PI / 2) d = Math.PI - d;
      if (d < bestD) {
        bestD = d;
        best = cand;
      }
    }
    return best;
  };

  const toWorld = (px, py) => {
    const r = Math.hypot(px, py);
    const b = (Math.atan2(px, py) * 180) / Math.PI; // bow-relative bearing
    const wb = degToRad(heading + b);
    return { x: bx + Math.sin(wb) * r, y: by + Math.cos(wb) * r };
  };
  const toBow = (wx, wy) => {
    const dx = wx - bx;
    const dy = wy - by;
    const r = Math.hypot(dx, dy);
    const b = degToRad(normalizeDeg((Math.atan2(dx, dy) * 180) / Math.PI) - heading);
    return { x: Math.sin(b) * r, y: Math.cos(b) * r };
  };

  // --- PHASE 3: place each side's wall ------------------------------------
  // Obstacle walls are 1 m long × 5 cm THICK, so a wall can legitimately be seen
  // as a large face (~up to 1 m broadside) OR as a tiny ~5 cm edge when viewed
  // end-on — a short detection is NOT automatically wrong. So we do NOT force a
  // 1 m minimum (that stretched real short/edge returns into fake metre-long
  // walls); we draw the OBSERVED extent, with only a small visible floor when a
  // side is sparse. Free ends are still run out to the range edge at draw time
  // (extendWallFreeEndsToRange), and grid-enforce + corner-join tidy the ends.
  const SEG_MIN_LEN_CM = 20; // just a visible floor; real extent otherwise
  for (let side = 0; side < 4; side += 1) {
    const s = stat[side];
    if (!s) {
      radarWalls.delete(side);
      continue;
    }
    const xy = s.xy;
    // Decide the wall orientation O and whether to draw at all.
    let O = null;
    if (s.count >= RADAR_FIT_MIN_POINTS) {
      if (s.res > RADAR_FIT_MAX_RESIDUAL_CM) {
        // Points don't lie on a line (scattered bodies / open water) — not a wall.
        radarWalls.delete(side);
        continue;
      }
      // הד-רפאים בטווח-קבוע (מים/קרקע) מתאים לקו בשיורי נמוך אך אינו קיר: הטווח
      // כמעט אינו משתנה לאורך הקשת. דוחים אותו כדי לא לצייר קיר במקום פנוי.
      if (isPhantomRing(xy)) {
        radarWalls.delete(side);
        continue;
      }
      O = phi !== null ? snapToGrid(s.theta) : s.theta;
    } else if (phi !== null) {
      // Sparse side (1–2 pts): rescue it using the shared grid orientation. With
      // 2 pts use their own chord; with 1 pt assume the wall is perpendicular to
      // the sensor's view bearing (a head-on wall), then snap to the grid.
      const prior = s.count >= 2 && s.theta !== null ? s.theta : s.repBearing + Math.PI / 2;
      O = snapToGrid(prior);
    } else {
      radarWalls.delete(side);
      continue;
    }
    // Fixed-orientation line: direction u, normal n, absolute offset c (median of
    // the point projections onto n) so it's stable even with a single point.
    const ux = Math.cos(O);
    const uy = Math.sin(O);
    const nx = -uy;
    const ny = ux;
    const cs = xy.map((q) => q.x * nx + q.y * ny).sort((a, b) => a - b);
    const c = cs[cs.length >> 1];
    let tMin = Infinity;
    let tMax = -Infinity;
    for (const q of xy) {
      const t = q.x * ux + q.y * uy;
      if (t < tMin) tMin = t;
      if (t > tMax) tMax = t;
    }
    if (tMax - tMin < SEG_MIN_LEN_CM) {
      const tc = (tMin + tMax) / 2;
      tMin = tc - SEG_MIN_LEN_CM / 2;
      tMax = tc + SEG_MIN_LEN_CM / 2;
    }
    // Segment endpoints (bow frame) -> frozen world coordinates.
    let w1 = toWorld(c * nx + tMin * ux, c * ny + tMin * uy);
    let w2 = toWorld(c * nx + tMax * ux, c * ny + tMax * uy);

    // Temporal smoothing in WORLD frame: a real wall barely moves, so EMA the
    // endpoints (matching new ends to the nearest previous ends, since the fit's
    // orientation can flip) to stop the line jumping on small range changes.
    const prev = radarWalls.get(side);
    if (prev) {
      const dDirect =
        Math.hypot(w1.x - prev.x1, w1.y - prev.y1) + Math.hypot(w2.x - prev.x2, w2.y - prev.y2);
      const dSwap =
        Math.hypot(w1.x - prev.x2, w1.y - prev.y2) + Math.hypot(w2.x - prev.x1, w2.y - prev.y1);
      if (dSwap < dDirect) {
        const tmp = w1;
        w1 = w2;
        w2 = tmp;
      }
      const A = RADAR_WALL_SMOOTH;
      w1 = { x: prev.x1 + (w1.x - prev.x1) * A, y: prev.y1 + (w1.y - prev.y1) * A };
      w2 = { x: prev.x2 + (w2.x - prev.x2) * A, y: prev.y2 + (w2.y - prev.y2) * A };
    }
    radarWalls.set(side, { x1: w1.x, y1: w1.y, x2: w2.x, y2: w2.y, t: now });

    // Feed the nav from the SMOOTHED line: convert the (smoothed) world endpoints
    // back to the current bow frame, rebuild the line there, then push each FRESH
    // bearing's corrected range into liveScan. This makes liveCone rely on the
    // same steady wall the display shows, so smoothing helps autonomy too.
    const p1 = toBow(w1.x, w1.y);
    const p2 = toBow(w2.x, w2.y);
    let sux = p2.x - p1.x;
    let suy = p2.y - p1.y;
    const slen = Math.hypot(sux, suy) || 1;
    sux /= slen;
    suy /= slen;
    const snx = -suy;
    const sny = sux;
    const sMn = p1.x * snx + p1.y * sny;
    // Feed the NAVIGATOR from the reconstructed wall, not just the raw samples.
    // A covered RUN (consecutive FRESH points <= RADAR_CORNER_REACH_CM apart
    // ALONG the wall) is a CONTINUOUS barrier — no passage that narrow can exist
    // (min corridor is 75 cm) — so we fill EVERY bearing bin the run subtends
    // with the wall's ray-cast range, not only the few sampled bearings. This is
    // the reconstructed WORLD driving autonomy: the boat reacts to the wall as
    // the solid, continuous obstacle it really is, while gaps > 40 cm are left
    // OPEN as genuine passages. Only FRESH-backed runs feed nav (older points are
    // display-only), so autonomy never acts on a stale bearing.
    const rayRange = (bDeg) => {
      const a = degToRad(bDeg);
      const denom = Math.sin(a) * snx + Math.cos(a) * sny;
      if (Math.abs(denom) < 1e-3) return null;
      const rr = sMn / denom;
      return rr > 0 ? rr : null;
    };
    const fresh = xy
      .filter((q) => q.fresh)
      .map((q) => ({ b: q.b, t: q.x * sux + q.y * suy }))
      .sort((a, b) => a.t - b.t);
    for (let i = 0; i < fresh.length; i += 1) {
      const rr = rayRange(fresh[i].b);
      if (rr != null) liveScan.set(binBearing(fresh[i].b - bowOff), { dist: rr, t: now });
      // Fill the bins BETWEEN this fresh sample and the next one IF they belong
      // to the same continuous run (<= 40 cm apart along the wall).
      if (i + 1 < fresh.length && fresh[i + 1].t - fresh[i].t <= RADAR_CORNER_REACH_CM) {
        const d = wrap180(fresh[i + 1].b - fresh[i].b);
        const steps = Math.floor(Math.abs(d) / LIVE_BIN_DEG);
        const dir = d >= 0 ? 1 : -1;
        for (let s = 1; s <= steps; s += 1) {
          const bb = fresh[i].b + dir * s * LIVE_BIN_DEG;
          const rrb = rayRange(bb);
          if (rrb != null) liveScan.set(binBearing(bb - bowOff), { dist: rrb, t: now });
        }
      }
    }

    // --- DISPLAY coverage RUNS: never draw the fitted line across an internal
    // coverage GAP > RADAR_CORNER_REACH_CM (the user's "don't connect points >40
    // cm apart" rule, applied ALONG the wall, not just at its ends). Project each
    // point onto the smoothed segment (p1->p2) to a fraction in [0,1], sort, and
    // break into runs wherever consecutive points are more than 40 cm apart. The
    // OUTER ends of the first/last run are pinned to 0 and 1 so the endpoint
    // corner-join/grid-enforce still connect the wall to its neighbours; only
    // genuine interior gaps (passages) become breaks. Fractions are relative to
    // w.x1..w.y2, so later join/grid-enforce endpoint edits carry the runs along.
    // radarWalls itself (nav feed, smoothing) is untouched — this is display only.
    const wobj = radarWalls.get(side);
    if (wobj && xy.length >= RADAR_FIT_MIN_POINTS && slen > 1) {
      const fr = xy
        .map((q) =>
          clamp(((q.x - p1.x) * (p2.x - p1.x) + (q.y - p1.y) * (p2.y - p1.y)) / (slen * slen), 0, 1)
        )
        .sort((a, b) => a - b);
      const runs = [];
      let f0 = 0; // pin the first run's outer end to the segment start
      let fp = fr[0];
      for (let i = 1; i < fr.length; i += 1) {
        if ((fr[i] - fp) * slen > RADAR_CORNER_REACH_CM) {
          runs.push([f0, fp]);
          f0 = fr[i];
        }
        fp = fr[i];
      }
      runs.push([f0, 1]); // pin the last run's outer end to the segment end
      wobj.runs = runs;
    }
  }

  // Drop walls we haven't been able to refresh for a while.
  for (const [side, w] of radarWalls) {
    if (now - w.t > WALL_TTL_MS) radarWalls.delete(side);
  }

  // יישור-רשת סופי: ההחלקה (EMA על הקצוות) עלולה להטות קטע במעט מזווית-הרשת
  // המדויקת. כופים על כל קיר להיות מיושר בדיוק ל-{φ, φ+90°} סביב מרכזו (מרכז+אורך
  // נשמרים), כדי שכל הקווים יישארו ניצבים או מקבילים זה לזה בדיוק (90°/180°).
  if (phi !== null) {
    for (const [, w] of radarWalls) {
      const cxw = (w.x1 + w.x2) / 2;
      const cyw = (w.y1 + w.y2) / 2;
      const half = Math.hypot(w.x2 - w.x1, w.y2 - w.y1) / 2;
      const O = snapToGrid(Math.atan2(w.y2 - w.y1, w.x2 - w.x1));
      const ux = Math.cos(O);
      const uy = Math.sin(O);
      w.x1 = cxw - ux * half;
      w.y1 = cyw - uy * half;
      w.x2 = cxw + ux * half;
      w.y2 = cyw + uy * half;
    }
  }

  // חיבור קצוות: אחרי שכל הקירות מוקמו, הוחלקו ויושרו-לרשת, מפגישים זוגות שקצותיהם
  // קרובים (< WALL_JOIN_GAP_CM) — קו ישר (~180°) או פינה (~90°) — תוך שמירת היישור.
  joinNearbyWalls();
}

// חיתוך שני ישרים אינסופיים: (px,py)+t·(dx,dy) עם (qx,qy)+s·(ex,ey). מחזיר null
// אם הם מקבילים (מכנה ~0).
function lineIntersect(px, py, dx, dy, qx, qy, ex, ey) {
  const denom = dx * ey - dy * ex;
  if (Math.abs(denom) < 1e-6) return null;
  const t = ((qx - px) * ey - (qy - py) * ex) / denom;
  return { x: px + dx * t, y: py + dy * t };
}

// מפגיש זוגות קירות שקצותיהם קרובים זה לזה. לכל זוג בוחרים את הזוג הקרוב ביותר של
// קצוות; אם הפער קטן מ-WALL_JOIN_GAP_CM מחברים אותם לנקודת-מפגש אחת:
//   * קירות ~מקבילים  -> אמצע שני הקצוות (מתמזגים לקו ישר, ~180°)
//   * קירות ~ניצבים   -> חיתוך הישרים (פינה בזווית ישרה, ~90°)
// פועל על הקצוות הקפואים-בעולם של radarWalls (תצוגה + החלקת-הפריים הבא).
function joinNearbyWalls() {
  const walls = [...radarWalls.values()];
  for (let i = 0; i < walls.length; i += 1) {
    for (let j = i + 1; j < walls.length; j += 1) {
      const A = walls[i];
      const B = walls[j];
      // ארבעה שילובי קצוות — בוחרים את הזוג הקרוב ביותר.
      const aEnds = [[A.x1, A.y1, 1], [A.x2, A.y2, 2]];
      const bEnds = [[B.x1, B.y1, 1], [B.x2, B.y2, 2]];
      let best = null;
      for (const ae of aEnds) {
        for (const be of bEnds) {
          const d = Math.hypot(ae[0] - be[0], ae[1] - be[1]);
          if (!best || d < best.d) best = { d, ae, be };
        }
      }
      if (!best) continue;

      // כיווני הקטעים (מנורמלים).
      let adx = A.x2 - A.x1;
      let ady = A.y2 - A.y1;
      let bdx = B.x2 - B.x1;
      let bdy = B.y2 - B.y1;
      const alen = Math.hypot(adx, ady) || 1;
      const blen = Math.hypot(bdx, bdy) || 1;
      adx /= alen;
      ady /= alen;
      bdx /= blen;
      bdy /= blen;
      const parallelness = Math.abs(adx * bdx + ady * bdy); // 1=מקביל, 0=ניצב

      // מקבילים: מיזוג רק על פני פער צר (זהירות מפני מיזוג שני קירות שונים). ניצבים
      // (פינה): מרשים טווח-הגעה גדול יותר כדי להאריך אותם עד נקודת-המפגש ב-90°.
      const gapLimit = parallelness >= 0.5 ? WALL_JOIN_GAP_CM : RADAR_CORNER_REACH_CM;
      if (best.d >= gapLimit) continue;

      if (parallelness >= 0.5) {
        // ~מקבילים -> קו ישר אחד (180°). לא מספיק להזיז קצה בודד — זה מטה את הקו;
        // מיישרים את שני הקירות לאותו היסט (ממוצע) לאורך כיוון A, כך שהם קו-לינאריים
        // בדיוק (הכיוון נשמר -> היישור לרשת נשמר), ואז מפגישים את הקצוות הקרובים
        // בנקודת-אמצע שנמצאת על אותו קו.
        const ux = adx;
        const uy = ady;
        const nx = -uy;
        const ny = ux;
        const aMx = (A.x1 + A.x2) / 2;
        const aMy = (A.y1 + A.y2) / 2;
        const bMx = (B.x1 + B.x2) / 2;
        const bMy = (B.y1 + B.y2) / 2;
        const cShared = (aMx * nx + aMy * ny + (bMx * nx + bMy * ny)) / 2;
        const reoff = (px, py) => {
          const t = px * ux + py * uy; // מיקום לאורך הקו נשמר
          return { x: t * ux + cShared * nx, y: t * uy + cShared * ny };
        };
        const a1 = reoff(A.x1, A.y1);
        const a2 = reoff(A.x2, A.y2);
        const b1 = reoff(B.x1, B.y1);
        const b2 = reoff(B.x2, B.y2);
        A.x1 = a1.x;
        A.y1 = a1.y;
        A.x2 = a2.x;
        A.y2 = a2.y;
        B.x1 = b1.x;
        B.y1 = b1.y;
        B.x2 = b2.x;
        B.y2 = b2.y;
        // הקצוות הקרובים (על אותו קו כעת) -> נקודת-אמצע משותפת, כדי לגשר על הפער.
        const aNx = best.ae[2] === 1 ? A.x1 : A.x2;
        const aNy = best.ae[2] === 1 ? A.y1 : A.y2;
        const bNx = best.be[2] === 1 ? B.x1 : B.x2;
        const bNy = best.be[2] === 1 ? B.y1 : B.y2;
        const mx = (aNx + bNx) / 2;
        const my = (aNy + bNy) / 2;
        if (best.ae[2] === 1) {
          A.x1 = mx;
          A.y1 = my;
        } else {
          A.x2 = mx;
          A.y2 = my;
        }
        if (best.be[2] === 1) {
          B.x1 = mx;
          B.y1 = my;
        } else {
          B.x2 = mx;
          B.y2 = my;
        }
      } else {
        // ~ניצבים -> פינה ב-90°: חיתוך הישרים דרך הקטעים. נקודת-החיתוך נמצאת על
        // שני הקווים, כך שהזזת הקצה הקרוב אליה שומרת על כיוון (יישור) שני הקירות.
        const X = lineIntersect(A.x1, A.y1, adx, ady, B.x1, B.y1, bdx, bdy);
        const mx = X ? X.x : (best.ae[0] + best.be[0]) / 2;
        const my = X ? X.y : (best.ae[1] + best.be[1]) / 2;
        if (best.ae[2] === 1) {
          A.x1 = mx;
          A.y1 = my;
        } else {
          A.x2 = mx;
          A.y2 = my;
        }
        if (best.be[2] === 1) {
          B.x1 = mx;
          B.y1 = my;
        } else {
          B.x2 = mx;
          B.y2 = my;
        }
      }
    }
  }
}

// --- שער ייצוב-תמונה בסחרור --------------------------------------------------
// ראה RADAR_SPIN_RATE_DEG_S. בזמן סיבוב במקום הנקודות הישנות (השמורות במסגרת-עולם)
// היו מסתובבות סביב הסירה בתצוגה החרטום-למעלה, ובחומרה אמיתית גם נמרחות כי
// ה-heading (dead-reckoning) לא אמין. לכן בזמן ייצוב מקפיאים את הצבירה ומשאירים
// את התמונה האחרונה על המסך — מצוירת דהוי וקפואה סביב ה-heading שנתפס בתחילת
// הסחרור (radarFreezeHeading), כך שהיא לא מסתובבת. כשהתמונה מתייצבת שוב הצבירה
// והציור החי חוזרים כרגיל.
let radarSpinning = false;
let radarSettleUntil = 0;
let radarFreezeHeading = 0; // ה-heading שנתפס בתחילת הסחרור, לציור התמונה האחרונה קפואה
let radarFreshFramesSinceSpin = RADAR_SETTLE_MIN_FRAMES; // פריימי-טלמטריה טריים שהצטברו מאז שהסיבוב נעצר (מתחיל "רווי" כדי שהתמונה תיחשב יציבה לפני הסיבוב הראשון)

// האם תמונת המכ"ם יציבה כעת? שלושה תנאים: (1) לא מסתחררים; (2) חלף מינימום זמן-הקיר
// אחרי שהסיבוב נעצר; (3) הצטברו מספיק פריימי-טלמטריה טריים מאז — כדי שבקישור איטי
// ההמתנה תתארך עד שיש באמת נתונים לבנות תמונה, ולא רק שיעבור זמן.
function radarPictureStable() {
  return (
    !radarSpinning &&
    performance.now() >= radarSettleUntil &&
    radarFreshFramesSinceSpin >= RADAR_SETTLE_MIN_FRAMES
  );
}

// מעדכן את מצב-הסחרור מקצב-הפוזה המוערך. מחזיר true אם מסתחררים כעת. ברגע-העלייה
// לסחרור תופס את ה-heading הנוכחי (כדי לצייר את התמונה האחרונה קפואה) ומנקה את
// liveScan (נתוני-ניווט) כדי שהאוטונומיה לא תפעל על זווית ישנה; התמונה עצמה
// (radarMemory/קירות) נשמרת ומוצגת דהויה. יש היסטרזיס (חצי-סף) נגד ריצוד סביב הסף.
function updateRadarSpinState() {
  const rateDegS = Math.abs(state.pose.turnRateRadS) * 180 / Math.PI;
  const now = performance.now();
  if (!radarSpinning && rateDegS >= RADAR_SPIN_RATE_DEG_S) {
    radarSpinning = true;
    radarFreezeHeading = state.pose.headingDeg; // תופסים כיוון כדי שהתמונה לא תסתובב
    radarFreshFramesSinceSpin = 0; // מתחילים לספור מחדש פריימים טריים אחרי הסחרור
    liveScan.clear(); // נתוני-ניווט בלבד — לא לפעול על זווית ישנה בזמן סחרור
    // חומרה אמיתית: ה-heading הוא dead-reckoning ואינו מסוגל לעקוב אחרי סיבוב
    // במקום, כך שהתמונה הצבורה במסגרת-עולם הייתה "נמרחת"/מסתובבת שגוי כשהצבירה
    // חוזרת (מיזוג פריימים לא-תואמים). מכיוון שאחרי כל סיבוב הסירה ממילא עוצרת
    // וממתינה לסריקה טרייה — מוחקים את התמונה בתחילת הסחרור ובונים אותה מחדש
    // נקי מהסריקה שאחרי, במקום למרוח. (בסימולטור ה-heading אמין, אז לא נוגעים.)
    if (!state.mockEnabled) {
      radarMemory.clear();
      resetRadarWalls();
    }
  } else if (radarSpinning && rateDegS <= RADAR_SPIN_RATE_DEG_S * 0.5) {
    radarSpinning = false;
  }
  // כל עוד מסתחררים (או ברגע-העלייה) דוחפים את חלון-ההתייצבות קדימה, כך שהוא
  // מתחיל לספור רק מרגע שהסחרור נעצר בפועל.
  if (radarSpinning) radarSettleUntil = now + RADAR_SETTLE_MS;
  return radarSpinning;
}

function updateRadarMemory() {
  // The whole sensor array is rotated by the current servo angle AND by the
  // boat's heading. Store each reading at its ABSOLUTE world bearing so that a
  // rotating boat leaves the world fixed in place (the radar view then rotates
  // the world back around the boat at draw time, keeping the bow pointing up).
  const sweep = sweepAngle();
  const heading = state.pose.headingDeg;
  const bx = state.pose.x;
  const by = state.pose.y;
  const now = performance.now();
  // A genuine telemetry frame just arrived: mark the picture fresh so radarClock
  // tracks real time. When frames stop, this stops advancing and the display
  // ageing clock freezes, keeping the last radar picture visible.
  lastRadarFreshAt = now;
  // בזמן ייצוב (סחרור + חלון-ההתייצבות) מקפיאים את הצבירה: לא מוסיפים/מעדכנים
  // נקודות, כדי שהתמונה האחרונה תישאר כפי שהייתה (מצוירת דהויה וקפואה — ראה
  // drawSensorArcs) ולא תסתובב סביב הסירה. עדיין מקדמים את מצב הגַּאפ-פיל כדי
  // שלא ימלא פער-ענק ברגע שהצבירה חוזרת.
  updateRadarSpinState();
  // סופרים פריימי-טלמטריה טריים שהגיעו כשלא מסתחררים — כך radarPictureStable
  // ידע שהצטברו מספיק נתונים לבנות תמונה מחדש אחרי סיבוב (לא רק שעבר זמן-קיר).
  if (!radarSpinning && radarFreshFramesSinceSpin < RADAR_SETTLE_MIN_FRAMES) {
    radarFreshFramesSinceSpin += 1;
  }
  if (!radarPictureStable()) {
    prevSweepDeg = sweep;
    prevSweepT = now;
    for (const beam of SENSOR_BEAMS) prevBeamVal[beam.key] = state.telemetry[beam.key];
    return;
  }
  // --- Angular gap-fill (real HW only, DISPLAY-only) ----------------------
  // The servo sweeps a full arc ~1×/s but RF delivers only ~1-3 packets/s, so
  // most swept angles never arrive and the picture strobes. When a new packet
  // lands, paint the angular gap since the previous packet with a linear
  // interpolation between the two readings — one packet fills a whole arc. These
  // points are marked filled:true: they render as blips (and give the noise
  // filter neighbours so real dots stop flickering) but are EXCLUDED from wall
  // fitting and from nav — the navigator never acts on an interpolated guess.
  if (!state.mockEnabled && prevSweepDeg !== null) {
    const dt = now - prevSweepT;
    const span = sweep - prevSweepDeg;
    const aspan = Math.abs(span);
    if (
      dt > 0 &&
      dt <= RADAR_GAPFILL_MAX_MS &&
      aspan > RADAR_GAPFILL_STEP_DEG &&
      aspan <= RADAR_GAPFILL_MAX_SPAN_DEG
    ) {
      const steps = Math.floor(aspan / RADAR_GAPFILL_STEP_DEG);
      const dir = span > 0 ? 1 : -1;
      for (let i = 1; i < steps; i += 1) {
        const s = prevSweepDeg + dir * i * RADAR_GAPFILL_STEP_DEG;
        const frac = (s - prevSweepDeg) / span; // 0..1
        for (const beam of SENSOR_BEAMS) {
          if (MASK_FRONT_DISPLAY && beam.masked) continue; // חיישן פגום — לא ממלא פערים בתצוגה
          const pv = prevBeamVal[beam.key];
          const nv = state.telemetry[beam.key];
          if (pv == null || nv == null || pv >= 999 || nv >= 999) continue;
          const absSlot = normalizeDeg(heading + beam.dir + s);
          const ex = radarMemory.get(absSlot);
          if (ex && !ex.filled) continue; // never clobber a real measurement
          const val = pv + (nv - pv) * frac;
          const rad = degToRad(absSlot);
          radarMemory.set(absSlot, {
            value: val,
            wx: bx + Math.sin(rad) * val,
            wy: by + Math.cos(rad) * val,
            t: now,
            filled: true,
          });
        }
      }
    }
  }
  for (const beam of SENSOR_BEAMS) {
    if (MASK_FRONT_DISPLAY && beam.masked) continue; // חיישן פגום — לא מצויר בתמונה
    const absSlot = normalizeDeg(heading + beam.dir + sweep);
    const newVal = state.telemetry[beam.key];
    if (newVal == null) continue;
    // Always refresh the timestamp when this slot is actually measured, so a
    // continuously confirmed obstacle stays alive; keep the smoothed value to
    // avoid flicker from minor (<5 cm) sensor noise.
    const existing = radarMemory.get(absSlot);
    const value =
      existing && Math.abs(newVal - existing.value) <= 5 ? existing.value : newVal;
    // Freeze the obstacle at the FIXED world point where it was actually
    // measured. Storing only (bearing, range) made every consumer re-project the
    // stale range from the boat's CURRENT pose, so a reading dragged along with
    // the boat (staying at a constant range from the bow) until the servo next
    // refreshed this bearing — up to RADAR_TTL_MS later. Anchoring it in world
    // space makes it stay put, exactly like the accumulated map cells.
    const rad = degToRad(absSlot);
    const wx = bx + Math.sin(rad) * value;
    const wy = by + Math.cos(rad) * value;
    radarMemory.set(absSlot, { value, wx, wy, t: now });
  }
  // Remember this packet's servo angle + per-sensor ranges for the next gap-fill.
  prevSweepDeg = sweep;
  prevSweepT = now;
  for (const beam of SENSOR_BEAMS) prevBeamVal[beam.key] = state.telemetry[beam.key];
}

// Remove slots whose last real measurement is older than the DISPLAY TTL. Uses
// the frozen display clock so a comms dropout doesn't prune the last picture
// away, and the longer RADAR_DISPLAY_TTL_MS window keeps each bearing on screen
// across a full slow real-HW sweep so the picture stays continuous (nav-facing
// consumers still re-filter these entries at the shorter RADAR_TTL_MS).
function pruneRadarMemory() {
  const now = radarClock();
  for (const [slot, entry] of radarMemory) {
    if (now - entry.t > RADAR_DISPLAY_TTL_MS) {
      radarMemory.delete(slot);
    }
  }
}

function drawSensorArcs(cx, cy, maxR) {
  pruneRadarMemory();
  // radarMemory holds each detection at the world point where it was measured
  // (entry.wx/wy). In MOCK (accurate pose) the dots are drawn WORLD-ANCHORED —
  // reprojected from the current pose like the world-frame walls + the
  // accumulated map — so the picture flows continuously as the boat moves and the
  // dots sit exactly where the boat really is. On REAL HW the dead-reckoned pose
  // is unreliable, so dots stay boat-fixed (bearing, range) to match the
  // boat-fixed walls there. Rotation is reflected via (bearing - heading) so the
  // bow stays pointing up.
  // בזמן ייצוב (סחרור) מציירים סביב ה-heading שנתפס בתחילת הסחרור, כך שהתמונה
  // האחרונה נשארת קפואה ולא מסתובבת סביב הסירה, ובעוצמת-צבע דהויה (fade).
  const stable = radarPictureStable();
  const heading = stable ? state.pose.headingDeg : radarFreezeHeading;
  const fade = stable ? 1 : 0.35;
  // Match updateRadarWalls: on real HW the walls are stored in the boat-fixed
  // frame (position zeroed), so reproject them with the same zeroed position or
  // they'd slide off the boat-fixed dots. Simulator keeps the true pose.
  const bx = state.mockEnabled ? state.pose.x : 0;
  const by = state.mockEnabled ? state.pose.y : 0;

  // Collect the currently valid detections as points at their true
  // (bearing, distance) location, so a straight wall lands on a straight line
  // and scattered bodies stay separate dots.
  const points = [];
  for (const [absSlot, entry] of radarMemory) {
    if (!entry.value || entry.value >= 999) continue;
    const dist = entry.value;
    if (dist > sim.maxRange) continue;
    let relSlot;
    let pixelDist;
    if (state.mockEnabled && entry.wx != null) {
      // World-anchored: reproject the FROZEN world point from the current pose, so
      // the dot stays put in the world and the picture flows instead of dragging.
      const dx = entry.wx - bx;
      const dy = entry.wy - by;
      pixelDist = (Math.min(Math.hypot(dx, dy), sim.maxRange) / sim.maxRange) * maxR;
      relSlot = normalizeDeg((Math.atan2(dx, dy) * 180) / Math.PI - heading - SWEEP_CENTER_DEG);
    } else {
      // Real HW: boat-fixed (bearing, range), matching the boat-fixed walls.
      // Center on the sweep midpoint so the bow points straight up.
      pixelDist = (dist / sim.maxRange) * maxR;
      relSlot = normalizeDeg(absSlot - heading - SWEEP_CENTER_DEG);
    }
    const rad = degToRad(relSlot) - Math.PI / 2;
    points.push({
      slot: absSlot,
      dist,
      x: cx + Math.cos(rad) * pixelDist,
      y: cy + Math.sin(rad) * pixelDist,
    });
  }

  // Noise rejection: a genuine surface reflects several returns clustered at a
  // similar bearing AND a similar range, whereas sensor noise shows up as a lone
  // blip with nothing beside it (e.g. one 25 cm return next to three 50 cm
  // returns is spray/echo, not an object). Keep a point only if it has at least
  // one neighbour within NOISE_ANGLE_DEG and NOISE_DIST_CM; drop the isolated
  // ones so they never appear on the picture.
  const NOISE_ANGLE_DEG = 30; // ~two 15° sweep slots
  const NOISE_DIST_CM = 20;   // similar range (< the 25 cm gap in the example)
  const signal = points.filter((p) =>
    points.some(
      (q) =>
        q !== p &&
        absAngleDiffDeg(p.slot, q.slot) <= NOISE_ANGLE_DEG &&
        Math.abs(p.dist - q.dist) <= NOISE_DIST_CM
    )
  );

  // Draw the per-side fitted wall as ONE solid, gap-free segment. This replaces
  // the old "connect adjacent dots" logic, which broke whenever the slow real-HW
  // sweep skipped a 15° slot (leaving >20° gaps that never joined) — the very
  // thing that made a wall look like scattered, blinking dots. The segment is
  // frozen in world space and reprojected to the boat's current pose, and it
  // survives WALL_TTL_MS so it stays steady between sweeps.
  const wallToPixel = (wx, wy) => {
    const dx = wx - bx;
    const dy = wy - by;
    const dist = Math.hypot(dx, dy);
    const absBearing = normalizeDeg((Math.atan2(dx, dy) * 180) / Math.PI);
    const relSlot = normalizeDeg(absBearing - heading - SWEEP_CENTER_DEG);
    const rad = degToRad(relSlot) - Math.PI / 2;
    const pixelDist = (Math.min(dist, sim.maxRange) / sim.maxRange) * maxR;
    return { x: cx + Math.cos(rad) * pixelDist, y: cy + Math.sin(rad) * pixelDist };
  };
  radarCtx.strokeStyle = `rgba(44, 255, 197, ${0.85 * fade})`;
  radarCtx.lineWidth = 3;
  radarCtx.lineCap = "round";
  const now = radarClock();
  // Draw each fitted wall at its OBSERVED extent. We deliberately do NOT extend
  // free ends out to the range edge: the real world is made of BOUNDED segments
  // (baffles 1 m long; the pool boundary reaches the range on its own where the
  // samples actually reach it), and artificially running every free end to the
  // range circle turned the picture into rays from the boat instead of the
  // rectangular world. "No floating wall" is enforced by REJECTING isolated noise
  // segments in updateRadarWalls, not by extending real ones. Interior coverage
  // gaps >40 cm are drawn as BREAKS via w.runs (fractions along w.x1..w.y2), so a
  // real passage never gets a line stretched across it.
  for (const [, w] of radarWalls) {
    if (now - w.t > WALL_TTL_MS) continue;
    const runs = w.runs && w.runs.length ? w.runs : [[0, 1]];
    for (const [f0, f1] of runs) {
      const a = wallToPixel(w.x1 + (w.x2 - w.x1) * f0, w.y1 + (w.y2 - w.y1) * f0);
      const b = wallToPixel(w.x1 + (w.x2 - w.x1) * f1, w.y1 + (w.y2 - w.y1) * f1);
      radarCtx.beginPath();
      radarCtx.moveTo(a.x, a.y);
      radarCtx.lineTo(b.x, b.y);
      radarCtx.stroke();
    }
  }

  // Draw each raw detection as a faint blip on top, so the underlying returns
  // are still visible behind the smoothed wall line.
  radarCtx.fillStyle = `rgba(44, 255, 197, ${0.6 * fade})`;
  for (const p of signal) {
    radarCtx.beginPath();
    radarCtx.arc(p.x, p.y, 2, 0, Math.PI * 2);
    radarCtx.fill();
  }
}

function drawSensorBeams(cx, cy, maxR) {
  const fovHalf = 7.5;
  const sweep = sweepAngle();
  radarCtx.save();
  radarCtx.lineWidth = 1.5;
  for (const beam of SENSOR_BEAMS) {
    const startRad = degToRad(beam.dir + sweep - SWEEP_CENTER_DEG - fovHalf);
    const endRad = degToRad(beam.dir + sweep - SWEEP_CENTER_DEG + fovHalf);
    const centerRad = degToRad(beam.dir + sweep - SWEEP_CENTER_DEG);

    // Soft FOV fill
    radarCtx.fillStyle = beam.color.replace(", 1)", ", 0.08)");
    radarCtx.beginPath();
    radarCtx.moveTo(cx, cy);
    radarCtx.arc(cx, cy, maxR, startRad - Math.PI / 2, endRad - Math.PI / 2);
    radarCtx.closePath();
    radarCtx.fill();

    // FOV edges + center
    radarCtx.strokeStyle = beam.color.replace(", 1)", ", 0.6)");
    for (const rad of [startRad, endRad, centerRad]) {
      radarCtx.beginPath();
      radarCtx.moveTo(cx, cy);
      radarCtx.lineTo(cx + Math.sin(rad) * maxR, cy - Math.cos(rad) * maxR);
      radarCtx.stroke();
    }
  }
  radarCtx.restore();
}

function setServerMessage(message, isError) {
  serverMessage.textContent = message;
  serverMessage.style.color = isError ? "#b00020" : "";
}

async function getJson(url) {
  if (SERVERLESS) return LocalBridge.request("GET", url);
  const response = await fetch(url, { cache: "no-store" });
  return parseJsonResponse(response);
}

async function postJson(url, body) {
  if (SERVERLESS) return LocalBridge.request("POST", url, body);
  const response = await fetch(url, {
    method: "POST",
    headers: {
      "Content-Type": "application/json",
    },
    body: JSON.stringify(body),
  });
  return parseJsonResponse(response);
}

async function parseJsonResponse(response) {
  const payload = await response.json().catch(() => ({}));
  if (!response.ok) {
    throw new Error(payload.error || `HTTP ${response.status}`);
  }
  return payload;
}

function clamp(value, min, max) {
  return Math.max(min, Math.min(max, value));
}

function degToRad(deg) {
  return (deg * Math.PI) / 180;
}

function normalizeDeg(deg) {
  let out = deg % 360;
  if (out < 0) {
    out += 360;
  }
  return out;
}

function absAngleDiffDeg(a, b) {
  let d = normalizeDeg(a) - normalizeDeg(b);
  d = ((d + 540) % 360) - 180;
  return Math.abs(d);
}

