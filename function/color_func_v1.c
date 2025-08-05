#ifndef COLOR_FUNC_H
#define COLOR_FUNC_H
/*
 * This file is part of evSim
 *
 * Copyright (C) 2025 pabogus <https://github.com/pabogus>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License as published
 * by the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Affero General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public License
 * along with this program. If not, see <https://www.gnu.org/licenses/>.
 */

/*
 * color_func_v1.c
 *
 * Provides functions to transform colors between color spaces
 * 
 * History
 * 15.01.2024 Initial version. Btw. today it snowed some centimeters.
 * 
 */

#include "config.h"
#include "color_func.h"



/*
//--------------------------------------------------------------------
// configure color functions
//--------------------------------------------------------------------
*/
//#define USE_FAST_HSV2RGB_FUNCTION
//#define LED_DIM_NR_SHIFTS; /* 1=1/2 2=1/4 3=1/8


#ifndef USE_FAST_HSV2RGB_FUNCTION
/*
 * 20.03.2016 Function from http://www.mikrocontroller.net/attachment/18560/main.c
 *  Author: Benedikt K. (benedikt) (Moderator)
 *  Date:   01.11.2006 12:52
 * 20.03.2016 RW Rückgabewert statt statischer Variablen
H:          der Farbton als Farbwinkel H auf dem Farbkreis (z. B. 0° = Rot, 120° = Grün, 240° = Blau)
S:          die Sättigung S in Prozent (z. B. 0% = keine Farbe, 50% = ungesättigte Farbe, 100% = gesättigte, reine Farbe)
V:          der Grauwert V als Prozentwert angegeben (z. B. 0% = keine Helligkeit, 100% = volle Helligkeit)

Skalierung der HSV Werte:
H:			0-255, 0=rot, 42=gelb, 85=grün, 128=türkis, 171=blau, 214=violett
S:			0-255, 0=weißtöne, 255=volle Farben
V:			0-255, 0=aus, 255=maximale Helligkeit
*/
// a color with 3 components: h, s and v
ColorRGB_t hsv_to_rgb(unsigned char h, unsigned char s, unsigned char v)
{
  unsigned char i, f;
  unsigned int p, q, t;
  ColorRGB_t color;
  if (s == 0)
  {
    color.r = color.g = color.b = v;
  }
  else
  {
    i = h / 43;
    f = h % 43;
    p = (v * (255 - s))/256;
    q = (v * ((10710 - (s * f))/42))/256;
    t = (v * ((10710 - (s * (42 - f)))/42))/256;
    switch (i)
    {
      case 0: color.r = v; color.g = t; color.b = p; break;
      case 1: color.r = q; color.g = v; color.b = p; break;
      case 2: color.r = p; color.g = v; color.b = t; break;
      case 3: color.r = p; color.g = q; color.b = v; break;
      case 4: color.r = t; color.g = p; color.b = v; break;
      case 5: color.r = v; color.g = p; color.b = q; break;
    }
  }
  return color;
}

#else /* USE_FAST_HSV2RGB_FUNCTION */
/*
 * https://de.wikipedia.org/wiki/HSV-Farbraum#Transformation_von_RGB_und_HSV
 */
ColorRGB_t hsv_to_rgb(uint8_t h, uint8_t s, uint8_t v)
{
  uint8_t i, f, fr, diff;
  uint16_t p, q, t;
  ColorRGB_t color;

  /* h is devided by 32 giving a usable range from 0-192
   * as the input is from 0-256 h needs to be scaled like h*(192/256)=h*(3/4)
   * -> h=h*3/4 = (4h-1h)/4 = h-h/4
   */
  uint8_t scale = h / 4;
  h = h - scale;

#ifdef LED_DIM_NR_SHIFTS
  v = v >> LED_DIM_NR_SHIFTS;
#endif

  i = h / 32;
  f = h - i*32;
  fr = 32 - f;

  p = v * (255 - s);
  p = p / 256;
  diff = v - p;

  q = fr * diff / 32;
  q = q + p;

  t = f * diff / 32;
  t = t + p;

  switch (i)
  {
    case 0:
    case 6: color.r = v; color.g = t; color.b = p; break;
    case 1: color.r = q; color.g = v; color.b = p; break;
    case 2: color.r = p; color.g = v; color.b = t; break;
    case 3: color.r = p; color.g = q; color.b = v; break;
    case 4: color.r = t; color.g = p; color.b = v; break;
    case 5: color.r = v; color.g = p; color.b = q; break;
  }
  return color;
}
#endif /* USE_FAST_HSV2RGB_FUNCTION */

#endif //COLOR_FUNC_H
