#ifndef _BITMAP_FRAME_H
#define _BITMAP_FRAME_H
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
 * bitmap_frame.h
 *
 * Structures to define a bitmap. 
 * Can e.g. be used to have bitmaps in the code and copy them
 * to a display. 
 */

#define BMP_TYPE_PICTURE   1
#define BMP_TYPE_ANIMATION 2

/// Data for a bitmap or animation
typedef struct {
  uint8_t type;
  uint8_t nrFrames;
  uint8_t nrPixlX;
  uint8_t nrPixlY;
  const void* data;
} framesDefinition;

#endif //_BITMAP_FRAME_H
