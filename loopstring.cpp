
/*
* loopstring.cpp - AD7415 library for Arduino
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

#include "loopstring.h"

void LOOPSTRING::add_character( char c )
{
#ifdef DEBUG
  Serial.println(c,HEX);
#endif
  if(cnt < LOOPSTRING_MAXLEN-1)
  {
    if(c == delimiter)
    {
      is_valid = true;
      lstring[cnt++] = 0;
    } else {
      lstring[cnt++] = c;
    }
  }
}
  
char *LOOPSTRING::get_string()
{
  return lstring;
}



char *LOOPSTRING::get_string(bool clearflag)
{
  if(clearflag) is_valid = false;
  return get_string();
}

void LOOPSTRING::set_delimiter(char d)
{
  delimiter = d;
}

bool LOOPSTRING::string_valid(void)
{
  return is_valid;
}
