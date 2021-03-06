#ifndef _CUST_BATTERY_METER_TABLE_H
#define _CUST_BATTERY_METER_TABLE_H

#include <mach/mt_typedefs.h>

// ============================================================
// define
// ============================================================
#define BAT_NTC_10 1
#define BAT_NTC_47 0

#if (BAT_NTC_10 == 1)
#define RBAT_PULL_UP_R             16900	
#endif

#if (BAT_NTC_47 == 1)
#define RBAT_PULL_UP_R             61900	
#endif

#define RBAT_PULL_UP_VOLT          1800



// ============================================================
// ENUM
// ============================================================

// ============================================================
// structure
// ============================================================

// ============================================================
// typedef
// ============================================================
typedef struct _BATTERY_PROFILE_STRUC
{
    kal_int32 percentage;
    kal_int32 voltage;
} BATTERY_PROFILE_STRUC, *BATTERY_PROFILE_STRUC_P;

typedef struct _R_PROFILE_STRUC
{
    kal_int32 resistance; // Ohm
    kal_int32 voltage;
} R_PROFILE_STRUC, *R_PROFILE_STRUC_P;

typedef enum
{
    T1_0C,
    T2_25C,
    T3_50C
} PROFILE_TEMPERATURE;

// ============================================================
// External Variables
// ============================================================

// ============================================================
// External function
// ============================================================

// ============================================================
// <DOD, Battery_Voltage> Table
// ============================================================
#if (BAT_NTC_10 == 1)
    BATT_TEMPERATURE Batt_Temperature_Table[] = {
        {-20,68237},
        {-15,53650},
        {-10,42506},
        { -5,33892},
        {  0,27219},
        {  5,22021},
        { 10,17926},
        { 15,14674},
        { 20,12081},
        { 25,10000},
        { 30,8315},
        { 35,6948},
        { 40,5834},
        { 45,4917},
        { 50,4161},
        { 55,3535},
        { 60,3014}
    };
#endif

#if (BAT_NTC_47 == 1)
    BATT_TEMPERATURE Batt_Temperature_Table[] = {
        {-20,483954},
        {-15,360850},
        {-10,271697},
        { -5,206463},
        {  0,158214},
        {  5,122259},
        { 10,95227},
        { 15,74730},
        { 20,59065},
        { 25,47000},
        { 30,37643},
        { 35,30334},
        { 40,24591},
        { 45,20048},
        { 50,16433},
        { 55,13539},
        { 60,11210}        
    };
#endif

// T0 -10C
BATTERY_PROFILE_STRUC battery_profile_t0[] =
{
{0,	4325},
{1,	4292},
{3,	4268},
{4,	4247},
{6,	4228},
{7,	4211},
{9,	4194},
{10,	4178},
{12,	4162},
{13,	4146},
{15,	4131},
{16,	4116},
{17,	4104},
{19,	4091},
{20,	4077},
{22,	4058},
{23,	4032},
{25,	4006},
{26,	3985},
{28,	3968},
{29,	3955},
{31,	3943},
{32,	3933},
{33,	3924},
{35,	3915},
{36,	3906},
{38,	3896},
{39,	3887},
{41,	3878},
{42,	3869},
{44,	3860},
{45,	3852},
{47,	3844},
{48,	3837},
{49,	3830},
{51,	3824},
{52,	3818},
{54,	3812},
{55,	3808},
{57,	3803},
{58,	3799},
{60,	3796},
{61,	3793},
{63,	3790},
{64,	3787},
{65,	3784},
{67,	3781},
{68,	3778},
{70,	3775},
{71,	3771},
{73,	3767},
{74,	3763},
{76,	3759},
{77,	3754},
{78,	3749},
{80,	3742},
{81,	3735},
{83,	3728},
{84,	3720},
{86,	3714},
{87,	3708},
{89,	3702},
{90,	3695},
{92,	3685},
{93,	3664},
{94,	3627},
{96,	3567},
{99,	3540},
{100,3500}       
};      
        
// T1 0C 
BATTERY_PROFILE_STRUC battery_profile_t1[] =
{
{0,	4330},
{1,	4304},
{3,	4283},
{4,	4264},
{6,	4247},
{7,	4230},
{9,	4214},
{10,	4199},
{12,	4183},
{13,	4168},
{15,	4153},
{16,	4137},
{18,	4122},
{19,	4107},
{21,	4095},
{22,	4086},
{24,	4075},
{25,	4057},
{27,	4030},
{28,	4003},
{30,	3984},
{31,	3969},
{33,	3957},
{34,	3947},
{36,	3938},
{37,	3928},
{38,	3917},
{40,	3905},
{41,	3894},
{43,	3882},
{44,	3872},
{46,	3863},
{47,	3854},
{49,	3846},
{50,	3838},
{52,	3832},
{53,	3825},
{55,	3820},
{56,	3814},
{58,	3809},
{59,	3804},
{61,	3799},
{62,	3795},
{64,	3792},
{65,	3789},
{67,	3786},
{68,	3784},
{70,	3781},
{71,	3778},
{72,	3775},
{74,	3772},
{75,	3767},
{77,	3763},
{78,	3758},
{80,	3753},
{81,	3745},
{83,	3738},
{84,	3731},
{86,	3722},
{87,	3712},
{89,	3703},
{90,	3699},
{92,	3695},
{93,	3690},
{95,	3679},
{96,	3639},
{98,	3569},
{99,	3463},
{100,3400}

};           

// T2 25C
BATTERY_PROFILE_STRUC battery_profile_t2[] =
{
{0,	4331},
{1,	4311},
{3,	4292},
{4,	4276},
{6,	4258},
{7,	4242},
{9,	4226},
{10,	4210},
{12,	4194},
{13,	4179},
{15,	4163},
{16,	4147},
{18,	4132},
{19,	4117},
{21,	4102},
{22,	4088},
{24,	4077},
{25,	4070},
{27,	4055},
{28,	4027},
{30,	4006},
{31,	3991},
{33,	3980},
{34,	3973},
{36,	3966},
{37,	3957},
{38,	3945},
{40,	3930},
{41,	3914},
{43,	3898},
{44,	3884},
{46,	3873},
{47,	3865},
{49,	3858},
{50,	3849},
{52,	3841},
{53,	3836},
{55,	3830},
{56,	3823},
{58,	3818},
{59,	3813},
{61,	3808},
{62,	3804},
{64,	3800},
{65,	3796},
{67,	3791},
{68,	3787},
{70,	3782},
{71,	3780},
{73,	3777},
{74,	3776},
{75,	3775},
{77,	3773},
{78,	3772},
{80,	3770},
{81,	3769},
{83,	3767},
{84,	3765},
{86,	3760},
{87,	3759},
{89,	3756},
{90,	3754},
{92,	3745},
{93,	3740},
{95,	3720},
{96,	3680},
{98,	3586},
{99,	3491},
{100,3400}
       
};     

// T3 50C
BATTERY_PROFILE_STRUC battery_profile_t3[] =
{
{0, 	4313},
{1, 	4294},
{3, 	4277},
{4, 	4260},
{6, 	4242},
{7, 	4226},
{9, 	4210},
{10, 	4194},
{12,	4179},
{13, 	4162},
{15, 	4147},
{16, 	4132},
{18, 	4117},
{19, 	4102},
{21, 	4087},
{22, 	4074},
{24, 	4060},
{25, 	4050},
{27, 	4036},
{28, 	4014},
{29, 	3998},
{31, 	3989},
{32, 	3981},
{34, 	3970},
{35, 	3959},
{37, 	3950},
{38, 	3936},
{40, 	3926},
{41, 	3911},
{43, 	3891},
{44, 	3877},
{46, 	3867},
{47, 	3857},
{49, 	3848},
{50, 	3841},
{52, 	3834},
{53, 	3828},
{54, 	3821},
{56, 	3815},
{57, 	3810},
{59, 	3804},
{60, 	3798},
{62, 	3794},
{63, 	3790},
{65, 	3786},
{66, 	3782},
{68, 	3779},
{69, 	3775},
{71, 	3767},
{72, 	3756},
{74, 	3750},
{75, 	3744},
{77, 	3737},
{78, 	3731},
{80, 	3728},
{81, 	3723},
{82, 	3717},
{84, 	3710},
{85, 	3702},
{87, 	3693},
{88, 	3682},
{90, 	3678},
{91, 	3676},
{93, 	3674},
{94, 	3670},
{96, 	3648},
{97, 	3591},
{99, 	3511},
{100,3500}
     
};           

// battery profile for actual temperature. The size should be the same as T1, T2 and T3
BATTERY_PROFILE_STRUC battery_profile_temperature[] =
{
  {0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },  
	{0  , 0 }, 
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },  
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 }
};    

// ============================================================
// <Rbat, Battery_Voltage> Table
// ============================================================
// T0 -10C
R_PROFILE_STRUC r_profile_t0[] =
{
{193,	4325},
{1796,	4292},
{1808,	4268},
{1796,	4247},
{1780,	4228},
{1763,	4211},
{1745,	4194},
{1728,	4178},
{1711,	4162},
{1694,	4146},
{1677,	4131},
{1663,	4116},
{1652,	4104},
{1644,	4091},
{1633,	4077},
{1616,	4058},
{1586,	4032},
{1558,	4006},
{1540,	3985},
{1531,	3968},
{1523,	3955},
{1519,	3943},
{1516,	3933},
{1513,	3924},
{1508,	3915},
{1504,	3906},
{1498,	3896},
{1492,	3887},
{1488,	3878},
{1484,	3869},
{1480,	3860},
{1478,	3852},
{1480,	3844},
{1480,	3837},
{1482,	3830},
{1482,	3824},
{1484,	3818},
{1487,	3812},
{1491,	3808},
{1495,	3803},
{1500,	3799},
{1507,	3796},
{1512,	3793},
{1522,	3790},
{1529,	3787},
{1536,	3784},
{1546,	3781},
{1556,	3778},
{1568,	3775},
{1581,	3771},
{1594,	3767},
{1611,	3763},
{1626,	3759},
{1646,	3754},
{1668,	3749},
{1693,	3742},
{1718,	3735},
{1751,	3728},
{1784,	3720},
{1827,	3714},
{1877,	3708},
{1934,	3702},
{2004,	3695},
{2084,	3685},
{2176,	3664},
{2269,	3627},
{2386,	3567},
{2486,	3540},
{2586,	3500}
};      

// T1 0C
R_PROFILE_STRUC r_profile_t1[] =
{
{180,	4330},
{941,	4304},
{953,	4283},
{953,	4264},
{953,	4247},
{950,	4230},
{946,	4214},
{943,	4199},
{939,	4183},
{936,	4168},
{932,	4153},
{928,	4137},
{924,	4122},
{918,	4107},
{918,	4095},
{922,	4086},
{926,	4075},
{920,	4057},
{897,	4030},
{880,	4003},
{873,	3984},
{868,	3969},
{866,	3957},
{866,	3947},
{864,	3938},
{859,	3928},
{852,	3917},
{843,	3905},
{836,	3894},
{831,	3882},
{827,	3872},
{827,	3863},
{829,	3854},
{828,	3846},
{830,	3838},
{832,	3832},
{836,	3825},
{838,	3820},
{843,	3814},
{847,	3809},
{851,	3804},
{857,	3799},
{860,	3795},
{866,	3792},
{872,	3789},
{877,	3786},
{884,	3784},
{890,	3781},
{897,	3778},
{904,	3775},
{912,	3772},
{920,	3767},
{931,	3763},
{943,	3758},
{957,	3753},
{970,	3745},
{986,	3738},
{1006,	3731},
{1029,	3722},
{1054,	3712},
{1084,	3703},
{1122,	3699},
{1172,	3695},
{1234,	3690},
{1316,	3679},
{1388,	3639},
{1498,	3569},
{1667,	3463},
{1767,	3400}
};     

// T2 25C
R_PROFILE_STRUC r_profile_t2[] =
{
{217, 	4331},
{217, 	4311},
{222, 	4292},
{222, 	4276},
{220, 	4258},
{219, 	4242},
{217, 	4226},
{218, 	4210},
{221, 	4194},
{220, 	4179},
{219, 	4163},
{219, 	4147},
{223, 	4132},
{228, 	4117},
{228, 	4102},
{230, 	4088},
{232, 	4077},
{249, 	4070},
{243, 	4055},
{227, 	4027},
{229, 	4006},
{232, 	3991},
{241, 	3980},
{243, 	3973},
{250, 	3966},
{247, 	3957},
{256, 	3945},
{244, 	3930},
{235, 	3914},
{228, 	3898},
{214, 	3884},
{206, 	3873},
{205, 	3865},
{206, 	3858},
{208, 	3849},
{210, 	3841},
{216, 	3836},
{216, 	3830},
{214, 	3823},
{219, 	3818},
{219, 	3813},
{223, 	3808},
{226, 	3804},
{229, 	3800},
{234, 	3796},
{232, 	3791},
{236, 	3787},
{233, 	3782},
{233, 	3777},
{230, 	3771},
{230, 	3766},
{229, 	3761},
{229, 	3755},
{230, 	3750},
{234, 	3745},
{237, 	3740},
{237, 	3733},
{241, 	3725},
{244, 	3719},
{244, 	3709},
{246, 	3699},
{250, 	3695},
{260, 	3693},
{265, 	3690},
{276, 	3684},
{272, 	3649},
{285, 	3586},
{314, 	3491},
{304,	3400}
}; 

// T3 50C
R_PROFILE_STRUC r_profile_t3[] =
{
{139, 	4313},
{139, 	4294},
{140, 	4277},
{139, 	4260},
{138, 	4242},
{137, 	4226},
{136, 	4210},
{138, 	4194},
{141, 	4179},
{136, 	4162},
{142, 	4147},
{140, 	4132},
{143, 	4117},
{144, 	4102},
{144, 	4087},
{146, 	4074},
{144, 	4060},
{156, 	4050},
{153, 	4036},
{144, 	4014},
{151, 	3998},
{153, 	3989},
{156, 	3981},
{154, 	3970},
{160, 	3959},
{165, 	3950},
{160, 	3936},
{166, 	3926},
{161, 	3911},
{145, 	3891},
{140, 	3877},
{140, 	3867},
{139, 	3857},
{139, 	3848},
{140, 	3841},
{137, 	3834},
{138, 	3828},
{141, 	3821},
{137, 	3815},
{141, 	3810},
{141, 	3804},
{140, 	3798},
{143, 	3794},
{147, 	3790},
{147, 	3786},
{150, 	3782},
{156, 	3779},
{156, 	3775},
{149, 	3767},
{142, 	3756},
{143, 	3750},
{144, 	3744},
{144, 	3737},
{142, 	3731},
{146, 	3728},
{146, 	3723},
{150, 	3717},
{149, 	3710},
{151, 	3702},
{147, 	3693},
{145, 	3682},
{146, 	3678},
{150, 	3676},
{153,	3674},
{159, 	3670},
{158, 	3648},
{160, 	3591},
{179, 	3511},
{170,	3500}
}; 

// r-table profile for actual temperature. The size should be the same as T1, T2 and T3
R_PROFILE_STRUC r_profile_temperature[] =
{
  {0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },  
	{0  , 0 }, 
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },  
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 }
};    

// ============================================================
// function prototype
// ============================================================
int fgauge_get_saddles(void);
BATTERY_PROFILE_STRUC_P fgauge_get_profile(kal_uint32 temperature);

int fgauge_get_saddles_r_table(void);
R_PROFILE_STRUC_P fgauge_get_profile_r_table(kal_uint32 temperature);

#endif	//#ifndef _CUST_BATTERY_METER_TABLE_H

