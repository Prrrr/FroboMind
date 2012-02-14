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
# File: kbhit.c
# Purpose: Implement the kbhit() function which returns true if a char is
# waiting to be read by eg. getchar().
# Project: FiRECom
# Author: Kjeld Jensen <kjeld@cetus.dk>
# Created:  2004-10-27 Kjeld Jensen, Source written
# Modified: 2011-02-07 Kjeld Jensen, Released under MIT license
# Modified: 2011-08-16 Kjeld Jensen, added call to system("stty echo") in kbhit_quit()
****************************************************************************/
/* system includes */

#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <termios.h>
#include "kbhit.h"

/***************************************************************************/
/* static variables */

static struct termios orig, new;
static int peek = -1;

/***************************************************************************/
void kbhit_init(void)
{
	tcgetattr(0, &orig);
	new = orig;
	new.c_lflag &= ~ICANON;
	new.c_lflag &= ~ECHO;
	new.c_lflag &= ~ISIG;
	new.c_cc[VMIN] = 1;
	new.c_cc[VTIME] = 0;
	tcsetattr(0, TCSANOW, &new);
}
/***************************************************************************/
int kbhit_test(void)
{
	int status = 0;
	char ch;
	int nread;

	if(peek == -1)
	{
		new.c_cc[VMIN]=0;
		tcsetattr(0, TCSANOW, &new);
		nread = read(0,&ch,1);
		new.c_cc[VMIN]=1;
		tcsetattr(0, TCSANOW, &new);

		if(nread == 1)
		{
			peek = ch;
			status = 1;
		}
	}
	else
		status = 1;

	return status;
}
/***************************************************************************/
char kbhit_getchar(void)
{
	char ch;

	if(peek != -1)
	{
		ch = peek;
		peek = -1;
	}
	else
		read(0,&ch,1);

	return ch;
}
/***************************************************************************/
void kbhit_done(void)
{
	tcsetattr(0,TCSANOW, &orig);
	system("stty echo"); /* enable echo of input from the keyboard */
}
/***************************************************************************/
