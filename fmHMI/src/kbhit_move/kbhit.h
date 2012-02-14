/****************************************************************************
# kbhit linux implementation
# Copyright (c) 2004-2011 Kjeld Jensen <kjeld@cetus.dk>
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
#
*****************************************************************************
# File: kbhit.h
# Purpose: Implement the kbhit_test() function which returns true if a char is
# waiting to be read by eg. kbhit_getchar().
# Project: FiRECom
# Author: Kjeld Jensen <kjeld@cetus.dk>
# Created:  2004-10-27 Kjeld Jensen, Source written
# Modified: 2011-02-07 Kjeld Jensen, Released under MIT license
# Modified: 2011-08-15 Kjeld Jensen, Updated documentation
****************************************************************************/
#ifndef KBHIT_H_
#define KBHIT_H_
/* defines */

/* keys */
#define KEY_ESCAPE          	27
#define KEY_SECOND			  	91

#define KEY_ARROW_UP        	65
#define KEY_ARROW_DOWN			66
#define KEY_ARROW_RIGHT			67
#define KEY_ARROW_LEFT			68

/***************************************************************************/
/* function prototypes */

void kbhit_init(void); /* call once to initialize */
int kbhit_test(void); /* returns true if a char is available */
char kbhit_getchar(void); /* return a char */
void kbhit_done(void); /* call once to reset */

/***************************************************************************/

#endif
