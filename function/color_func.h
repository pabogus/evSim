#ifndef _COLOR_FUNC_H_
#define _COLOR_FUNC_H_
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
 * color_func.h
 *
 * Definitions of structures holding color information
 * 
 * History
 * 15.01.2024 Initial version
 * 
 */

typedef struct
{
  uint8_t g;
  uint8_t r;
  uint8_t b;
} ColorRGB_t;

// a color with 3 components: h, s and v
typedef struct
{
  unsigned char h;
  unsigned char s;
  unsigned char v;
} ColorHSV_t;

ColorRGB_t hsv_to_rgb(uint8_t h, uint8_t s, uint8_t v);

#endif //_COLOR_FUNC_H_
