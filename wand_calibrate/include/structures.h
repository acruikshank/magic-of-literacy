#define NUM_STREAKS 50

typedef struct WandUpdate {
  uint8_t theta;
  uint8_t amplitude;
  bool hasStreak;
  uint8_t streakColor;
} WandUpdate;
