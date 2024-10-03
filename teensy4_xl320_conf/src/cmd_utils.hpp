#ifndef CMD_UTILS_HPP
#define CMD_UTILS_HPP

#define RAW_MAX 4095
#define RAW_MIN 0
#define DEG_MAX 359
#define DEG_MIN 0

float map_float(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float raw2deg(int32_t raw)
{
  // return map_float(raw, 0, 1023, 0, 300);
  return map_float(raw, RAW_MIN, RAW_MAX, DEG_MIN, DEG_MAX);
}

float deg2raw(float deg)
{
  // return map_float(deg, 0, 300, 0, 1023);
  return map_float(deg, DEG_MIN, DEG_MAX, RAW_MIN, RAW_MAX);
}

#endif // CMD_UTILS_HPP