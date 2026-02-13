#include <Pixy2.h>
#include <PIDLoop.h>
#include <ZumoMotors.h>

#define ZUMO_FAST   390
#define ZUMO_SLOW   290
#define TURN_SPIN   400
#define SEARCH_SPIN 220
#define X_CENTER    (pixy.frameWidth/2)

Pixy2 pixy;
ZumoMotors motors;
PIDLoop headingLoop(7500, 0, 4000, false);

enum State { FOLLOW, TURN_RIGHT };
State state = FOLLOW;

uint32_t lastJunctionMs = 0;
uint32_t turnStartMs = 0;

const uint32_t COOLDOWN_MS = 1000;
const uint32_t TURN_TIMEOUT_MS = 900;

int32_t lastError = 0;
bool hasLastError = false;

static inline void swap16(int16_t &a, int16_t &b){ int16_t t=a; a=b; b=t; }

static inline int16_t farXNormalized()
{
  int16_t x0=pixy.line.vectors->m_x0, y0=pixy.line.vectors->m_y0;
  int16_t x1=pixy.line.vectors->m_x1, y1=pixy.line.vectors->m_y1;
  if (y0 < y1) { swap16(x0,x1); swap16(y0,y1); }
  return x1;
}

static inline void setSpeeds(int16_t l, int16_t r)
{
  if (l > 400) l = 400; if (l < -400) l = -400;
  if (r > 400) r = 400; if (r < -400) r = -400;
  motors.setLeftSpeed(l);
  motors.setRightSpeed(r);
}

static inline void searchSpin()
{
  // if we remember the line was left, spin left; else spin right
  if (hasLastError && lastError < 0) setSpeeds(-TURN_SPIN, TURN_SPIN);
  else setSpeeds(SEARCH_SPIN, -SEARCH_SPIN);
}

void setup()
{
  motors.setLeftSpeed(0);
  motors.setRightSpeed(0);

  pixy.init();
  pixy.setLamp(1, 1);
  pixy.changeProg("line");
  pixy.setServos(455, 1000);
}

void loop()
{
  int8_t res = pixy.line.getMainFeatures();
  uint32_t now = millis();

  // Nothing detected -> don't drive forward, just search
  if (res <= 0) { searchSpin(); return; }

  // --- Forced right turn state (T junction) ---
  if (state == TURN_RIGHT)
  {
    setSpeeds(TURN_SPIN, -TURN_SPIN);

    if (res & LINE_VECTOR)
    {
      int32_t err = (int32_t)farXNormalized() - (int32_t)X_CENTER;
      lastError = err; hasLastError = true;

      // reacquire and exit once roughly centered
      if (abs((int)err) < 20)
      {
        state = FOLLOW;
        lastJunctionMs = now;
        return;
      }
    }

    // timeout: stop forcing, resume normal behavior
    if (now - turnStartMs > TURN_TIMEOUT_MS)
      state = FOLLOW;

    return;
  }

  // --- Junction detection (only trigger on T: branches >= 3) ---
  if ((res & LINE_INTERSECTION) && (now - lastJunctionMs > COOLDOWN_MS))
  {
    int branches = pixy.line.intersections->m_n;
    if (branches >= 3)
    {
      state = TURN_RIGHT;
      turnStartMs = now;
      return;
    }
  }

  // --- Normal PID follow ---
  if (res & LINE_VECTOR)
  {
    int32_t err = (int32_t)farXNormalized() - (int32_t)X_CENTER;
    lastError = err; hasLastError = true;

    headingLoop.update(err);

    int base = (pixy.line.vectors->m_flags & LINE_FLAG_INTERSECTION_PRESENT) ? ZUMO_SLOW : ZUMO_FAST;
    int16_t l = base + headingLoop.m_command;
    int16_t r = base - headingLoop.m_command;

    setSpeeds(l, r);
  }
  else
  {
    // no vector but "something" detected -> search instead of moving forward
    searchSpin();
  }
}

