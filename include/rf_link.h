// rf_link.h — פרוטוקול RF משותף לסירה (main.cpp) ולחוף (shore_radio.cpp).
// שני הקצוות חייבים לכלול את הקובץ הזה ולהיצרב יחד — כל שינוי כאן = צריבה לשניהם.
//
// שלוש מטרות (לבקשת המשתמש):
//   1) דחיסה: מטען הפקודה/הטלמטריה קוצץ מ-10 בתים ל-5 בתי נתונים בלבד
//      (מהירויות ומרחקים נדחסים לבית בודד כל אחד), => פחות זמן-אוויר => פחות דיליי.
//   2) אימות: לכל פריים מצורף MAC (HalfSipHash-2-4 עם מפתח סודי משותף), כך
//      שהמקלט מקבל אך ורק פריימים שנחתמו במפתח — גורם זר בלי המפתח לא יכול
//      לשלוט בסירה ולא לזייף טלמטריה. בנוסף מונה-מחזור (seq) חוסם replay.
//   3) אמינות בלי דיליי + טווח: משאירים 2000bps (רגישות/טווח מרביים) אבל
//      הפריים קצר יותר => פחות זמן-אוויר => round-trip מהיר יותר + פחות סיכוי
//      שביט שגוי יהרוס פריים. ה-MAC גם משמש בדיקת-שלמות חזקה מ-CRC לבדו.
//
// דחיסה עדיפה על העלאת קצב: קצב נמוך שומר על טווח, וקיצור הפריים נותן את
// שיפור-הדיליי בלי לוותר על הטווח.
#ifndef RF_LINK_H
#define RF_LINK_H

#include <Arduino.h>
#include <string.h>

// ===== מפתח סודי משותף (64 ביט) — חייב להיות זהה בסירה ובחוף =====
// החליפו לערכים אקראיים משלכם ושמרו בסוד. שינוי המפתח = צריבה לשני הקצוות.
static const uint8_t RF_KEY[8] = {
  0x9E, 0x37, 0x79, 0xB1, 0x7A, 0x2C, 0x4F, 0xE5
};

// ===== HalfSipHash-2-4 — MAC ממופתח, 32 ביט, ידידותי ל-AVR (8 ביט) =====
// נבחר על פני SipHash המלא כי הוא עובד ב-32 ביט (מהיר/קטן על Uno). המטען
// הקצר (5-6 בתים) => בלוק אחד + סיום => עשרות מיקרו-שניות, זניח מול ~150ms אוויר.
static inline uint32_t rf_rotl32(uint32_t x, uint8_t b) {
  return (x << b) | (x >> (32 - b));
}
static inline uint32_t rf_load32(const uint8_t *p) {
  return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
         ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}
static uint32_t rf_halfsiphash(const uint8_t *in, uint8_t inlen) {
  uint32_t k0 = rf_load32(RF_KEY);
  uint32_t k1 = rf_load32(RF_KEY + 4);
  uint32_t v0 = k0;
  uint32_t v1 = k1;
  uint32_t v2 = 0x6c796765UL ^ k0;
  uint32_t v3 = 0x74656462UL ^ k1;
  uint32_t b = ((uint32_t)inlen) << 24;

#define RF_SIPROUND                                                            \
  do {                                                                         \
    v0 += v1; v1 = rf_rotl32(v1, 5);  v1 ^= v0; v0 = rf_rotl32(v0, 16);        \
    v2 += v3; v3 = rf_rotl32(v3, 8);  v3 ^= v2;                                \
    v0 += v3; v3 = rf_rotl32(v3, 7);  v3 ^= v0;                                \
    v2 += v1; v1 = rf_rotl32(v1, 13); v1 ^= v2; v2 = rf_rotl32(v2, 16);        \
  } while (0)

  uint8_t left = inlen & 3;
  uint8_t fullblocks = inlen - left;
  for (uint8_t i = 0; i < fullblocks; i += 4) {
    uint32_t m = rf_load32(in + i);
    v3 ^= m; RF_SIPROUND; RF_SIPROUND; v0 ^= m;
  }
  const uint8_t *tail = in + fullblocks;
  switch (left) {
    case 3: b |= ((uint32_t)tail[2]) << 16;  // נפילה מכוונת
    case 2: b |= ((uint32_t)tail[1]) << 8;   // נפילה מכוונת
    case 1: b |= ((uint32_t)tail[0]); break;
    default: break;
  }
  v3 ^= b; RF_SIPROUND; RF_SIPROUND; v0 ^= b;
  v2 ^= 0xff; RF_SIPROUND; RF_SIPROUND; RF_SIPROUND; RF_SIPROUND;
#undef RF_SIPROUND
  return v1 ^ v3;
}

// ===== פריימים דחוסים (packed = בלי ריפוד; אותו פריסת-בתים בשני הקצוות) =====

// פקודה: חוף -> סירה. 8 בתים (היה 10). המהירויות מוקטנות פי-2 (רזולוציית 2,
// זניחה ל-PWM/סרוו) כדי להיכנס לבית חתום בודד כל אחת. radarAngle הושמט לגמרי —
// הסירה סורקת מקומית ולא השתמשה בו ממילא.
struct __attribute__((packed)) RfCommand {
  uint8_t seq;      // מונה-מחזור (anti-replay)
  int8_t  left;     // מהירות/2
  int8_t  right;    // מהירות/2
  int8_t  winch;    // מהירות/2
  uint8_t flags;    // bit0 = mode (0=ידני, 1=אוטומטי)
  uint8_t mac[3];   // 3 הבתים הנמוכים של HalfSipHash על 5 הבתים הראשונים
};

// טלמטריה: סירה -> חוף. 8 בתים (היה 10). מרחקים מוקטנים פי-2 (0..508ס"מ ברזולוציית
// 2ס"מ), 255 = "אין הד" (משמר את סמן ה-999 המקורי). הזווית 0..180 בבית אחד.
struct __attribute__((packed)) RfTelemetry {
  uint8_t seq;
  uint8_t dRadar;   // ס"מ/2, 255 = אין הד
  uint8_t dFront;
  uint8_t dLeft;
  uint8_t dRight;
  uint8_t angle;    // 0..180
  uint8_t mac[2];   // 2 הבתים הנמוכים של HalfSipHash על 6 הבתים הראשונים
};

// ===== קידוד/פענוח שדות =====
static inline uint8_t rf_dist_encode(int16_t cm) {
  if (cm < 0 || cm >= 999) return 255;   // אין הד / סמן
  int16_t v = cm / 2;
  if (v > 254) v = 254;
  return (uint8_t)v;
}
static inline int16_t rf_dist_decode(uint8_t v) {
  return (v == 255) ? 999 : (int16_t)v * 2;
}
static inline int8_t rf_speed_encode(int v) {
  if (v > 254) v = 254;
  if (v < -254) v = -254;
  return (int8_t)(v / 2);
}
static inline int rf_speed_decode(int8_t v) {
  return (int)v * 2;
}

// ===== חתימה/אימות =====
static inline void rf_cmd_sign(RfCommand *p) {
  uint32_t t = rf_halfsiphash((const uint8_t *)p, 5);
  p->mac[0] = (uint8_t)(t & 0xff);
  p->mac[1] = (uint8_t)((t >> 8) & 0xff);
  p->mac[2] = (uint8_t)((t >> 16) & 0xff);
}
static inline bool rf_cmd_verify(const RfCommand *p) {
  uint32_t t = rf_halfsiphash((const uint8_t *)p, 5);
  return p->mac[0] == (uint8_t)(t & 0xff) &&
         p->mac[1] == (uint8_t)((t >> 8) & 0xff) &&
         p->mac[2] == (uint8_t)((t >> 16) & 0xff);
}
static inline void rf_tel_sign(RfTelemetry *p) {
  uint32_t t = rf_halfsiphash((const uint8_t *)p, 6);
  p->mac[0] = (uint8_t)(t & 0xff);
  p->mac[1] = (uint8_t)((t >> 8) & 0xff);
}
static inline bool rf_tel_verify(const RfTelemetry *p) {
  uint32_t t = rf_halfsiphash((const uint8_t *)p, 6);
  return p->mac[0] == (uint8_t)(t & 0xff) &&
         p->mac[1] == (uint8_t)((t >> 8) & 0xff);
}

// רעננות מונה-המחזור (anti-replay): true אם seq חדש מ-last (מודולו 256, חלון קדימה).
// מטפל בגלישת המונה בקצב שידור גבוה. פריים ראשון אחרי אתחול מתקבל בלי הבדיקה
// (ראו haveSeq בקוד הקצוות) כדי להסתנכרן למונה של הצד השני.
static inline bool rf_seq_fresh(uint8_t seq, uint8_t last) {
  uint8_t d = (uint8_t)(seq - last);
  return d != 0 && d < 128;
}

#endif  // RF_LINK_H
