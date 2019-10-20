#define STATE_INIT_CHANNEL 0
#define STATE_COLLECT 1
#define STATE_READY 2
#define STATE_POISED 3
#define STATE_TRIGGERED 4

typedef struct WandUpdate {
  uint8_t channel;
  uint8_t theta;
  uint8_t amplitude;
  bool hasStreak;
  uint8_t streakColor;
  uint8_t state;
} WandUpdate;
