#include "oled.h"

static uint8_t OLED_AppendUnsigned(char *buffer, uint32_t value)
{
  char reversed[10];
  uint8_t length = 0U;
  uint8_t digits = 0U;

  if (value == 0U)
  {
    buffer[length++] = '0';
    return length;
  }

  while ((value > 0U) && (digits < sizeof(reversed)))
  {
    reversed[digits++] = (char)('0' + (value % 10U));
    value /= 10U;
  }

  while (digits > 0U)
  {
    buffer[length++] = reversed[--digits];
  }

  return length;
}

void OLED_PrintNumber(uint8_t x, uint8_t y, int num, const ASCIIFont *font, OLED_ColorMode color)
{
  char buffer[12];
  uint8_t length = 0U;
  unsigned int magnitude;

  if (num < 0)
  {
    buffer[length++] = '-';
    magnitude = (unsigned int)(-(num + 1)) + 1U;
  }
  else
  {
    magnitude = (unsigned int)num;
  }

  length += OLED_AppendUnsigned(buffer + length, magnitude);
  buffer[length] = '\0';
  OLED_PrintASCIIString(x, y, buffer, font, color);
}

void OLED_PrintFloat(uint8_t x, uint8_t y, float num, uint8_t decimals, const ASCIIFont *font, OLED_ColorMode color)
{
  char buffer[20];
  uint8_t length = 0U;
  uint32_t scale = 1U;
  uint32_t scaled_value;
  uint32_t integer_part;
  uint32_t fractional_part;
  uint8_t i;

  if (decimals > 4U)
  {
    decimals = 4U;
  }

  if (num < 0.0f)
  {
    buffer[length++] = '-';
    num = -num;
  }

  for (i = 0U; i < decimals; i++)
  {
    scale *= 10U;
  }

  scaled_value = (uint32_t)(num * (float)scale + 0.5f);
  integer_part = scaled_value / scale;
  fractional_part = scaled_value % scale;

  length += OLED_AppendUnsigned(buffer + length, integer_part);

  if (decimals > 0U)
  {
    buffer[length++] = '.';

    for (i = decimals; i > 0U; i--)
    {
      uint32_t divisor = 1U;
      uint8_t j;

      for (j = 1U; j < i; j++)
      {
        divisor *= 10U;
      }

      buffer[length++] = (char)('0' + ((fractional_part / divisor) % 10U));
    }
  }

  buffer[length] = '\0';
  OLED_PrintASCIIString(x, y, buffer, font, color);
}
