#ifndef CMD_UTILS_HPP
#define CMD_UTILS_HPP

float map_float(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float raw2deg(float raw)
{
  return map_float(raw, 0, 1023, 0, 300);
}

float deg2raw(float deg)
{
  return map_float(deg, 0, 300, 0, 1023);
}

#endif // CMD_UTILS_HPP