/*
 * Copyright (C) 2010 MediaTek, Inc.
 *
 * Author: Terry Chang <terry.chang@mediatek.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef _RTC_MT_H_
#define _RTC_MT_H_

#define RTC_YES		1
#define RTC_NO		0

/*
 * Reset to default date if RTC time is over 2038/1/19 3:14:7
 * Year (YEA)        : 1970 ~ 2037
 * Month (MTH)       : 1 ~ 12
 * Day of Month (DOM): 1 ~ 31
 */
#define RTC_OVER_TIME_RESET	RTC_YES
#define RTC_DEFAULT_YEA		2016
#define RTC_DEFAULT_MTH		1
#define RTC_DEFAULT_DOM		1

/*
Cap selection
0 : 0.1uF only
1 : 0.1uF + 1uF + 1.5Kohm
2 : 0.1uF + 2.2uF + 1.5Kohm
3 : 0.1uF + 4.7uF + 1.5Kohm
4 : 0.1uF + 10uF + 1.5Kohm
5 : 0.1uF + 22uF + 1.5Kohm
6 : 0.1uF + super cap(>>22uF) + 1.5Kohm
7 : 0.1uF + little Li battery + 1.5Kohm
*/
#define RTC_CAP_SEL		6

#endif
