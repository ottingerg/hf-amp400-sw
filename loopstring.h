
/*
* loopstring.h - AD7415 library for Arduino
* Copyright (c) 2015 Otelo eGen - Georg Ottinger (georg.ottinger@oteloegen.at)
* All rights reserved.
*
* This library is free software; you can redistribute it and/or
* modify it under the terms of the GNU Lesser General Public
* License as published by the Free Software Foundation; either
* version 2.1 of the License, or (at your option) any later version.
*
* This library is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
* Lesser General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public
* License along with this library; if not, write to the Free Software
* Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#ifndef LOOPSTRING_H_INCLUDED
#define LOOPSTRRING_H_INCLUDED

#include "Arduino.h"

#define LOOPSTRING_MAXLEN 50

class LOOPSTRING
{
public:
  
  void add_character( char c );
  bool string_valid(void);
  char *get_string();
  char *get_string(bool clearflag);
  void set_delimiter(char d);
  void clear_valid(void);

private:

  char lstring[LOOPSTRING_MAXLEN];
  bool is_valid = false;
  int cnt = 0;
  char delimiter = 0x0A;
    
};

#endif        // LOOPSTRING_H_INCLUDED

