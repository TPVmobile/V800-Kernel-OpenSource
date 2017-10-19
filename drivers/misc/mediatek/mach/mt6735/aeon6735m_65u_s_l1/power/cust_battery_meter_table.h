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
        { 60,3014},
        { 65,2586}
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
	{0   , 4098},         
	{2   , 4069},         
	{3   , 4053},         
	{5   , 4040},         
	{7   , 4023},         
	{8   , 3997},         
	{10  , 3961},         
	{12  , 3946},         
	{13  , 3938},         
	{15  , 3932},         
	{17  , 3926},         
	{19  , 3918},         
	{20  , 3910},         
	{22  , 3901},         
	{23  , 3894},         
	{25  , 3885},         
	{27  , 3874},         
	{29  , 3866},         
	{30  , 3856},         
	{32  , 3846},         
	{34  , 3838},         
	{35  , 3830},         
	{37  , 3823},         
	{39  , 3817},         
	{40  , 3814},         
	{42  , 3808},         
	{44  , 3806},         
	{45  , 3803},         
	{47  , 3801},         
	{49  , 3798},         
	{50  , 3795},         
	{52  , 3796},         
	{54  , 3795},         
	{55  , 3792},         
	{57  , 3792},         
	{59  , 3790},         
	{60  , 3789},         
	{62  , 3787},         
	{64  , 3785},         
	{65  , 3783},         
	{67  , 3781},         
	{69  , 3776},         
	{70  , 3772},         
	{72  , 3767},         
	{74  , 3763},         
	{76  , 3758},         
	{77  , 3751},         
	{79  , 3742},         
	{81  , 3734},         
	{82  , 3725},         
	{84  , 3719},         
	{86  , 3715},         
	{87  , 3712},         
	{89  , 3707},         
	{91  , 3702},         
	{92  , 3696},         
	{94  , 3678},         
	{96  , 3647},         
	{97  , 3612},         
	{98  , 3575},         
	{98  , 3537},          
  {99  , 3502},
  {99  , 3472},
  {100 , 3443},
  {100 , 3419},
  {100 , 3395},
  {100 , 3373},
  {100 , 3357},
  {100 , 3341},
  {100 , 3328},
	{100 , 3317}, 
	{100 , 3307},
	{100 , 3300},
	{100 , 3293},
	{100 , 3288},
	{100 , 3283}, 
	{100 , 3275},
	{100 , 3271},
	{100 , 3267},
	{100 , 3260},
	{100 , 3256},
	{100 , 3251},
	{100 , 3243},
	{100 , 3239},
	{100 , 3233},
	{100 , 3225},
	{100 , 3218},
	{100 , 3214},
	{100 , 3209},
	{100 , 3202},
	{100 , 3196},
	{100 , 3185},
	{100 , 3171},
	{100 , 3157},
	{100 , 3142},
	{100 , 3125},
	{100 , 3114},
	{100 , 3095},
	{100 , 3095},
	{100 , 3270}	       
};      
        
// T1 0C 
BATTERY_PROFILE_STRUC battery_profile_t1[] =
{
	{0   ,4048},         
	{2   ,4008},         
	{3   ,3989},         
	{5   ,3977},         
	{6   ,3966},         
	{8   ,3960},         
	{9   ,3956},         
	{11  ,3951},         
	{13  ,3948},         
	{14  ,3941},         
	{16  ,3935},         
	{17  ,3928},         
	{19  ,3922},         
	{20  ,3914},         
	{22  ,3906},         
	{24  ,3898},         
	{25  ,3892},         
	{27  ,3882},         
	{28  ,3872},         
	{30  ,3860},         
	{31  ,3849},         
	{33  ,3839},         
	{35  ,3831},         
	{36  ,3824},         
	{38  ,3818},         
	{39  ,3815},         
	{41  ,3808},         
	{42  ,3805},         
	{44  ,3803},         
	{46  ,3798},         
	{47  ,3796},         
	{49  ,3793},         
	{50  ,3792},         
	{52  ,3790},         
	{53  ,3790},         
	{55  ,3788},         
	{57  ,3788},         
	{58  ,3787},         
	{60  ,3787},         
	{61  ,3785},         
	{63  ,3785},         
	{64  ,3784},         
	{66  ,3782},         
	{67  ,3779},         
	{69  ,3777},         
	{71  ,3774},         
	{72  ,3769},         
	{74  ,3766},         
	{75  ,3762},         
	{77  ,3756},         
	{78  ,3748},         
	{80  ,3742},         
	{82  ,3734},         
	{83  ,3724},         
	{85  ,3714},         
	{86  ,3708},         
	{88  ,3703},         
	{89  , 3701},         
  {91  ,3699},
	{93  ,3696},         
	{94  ,3689},          
  {96  ,3662},
  {97  ,3601},
  {99  ,3533},
  {99  ,3475},
  {100 ,3418},
  {100 ,3363},
  {100 ,3315},
  {100 ,3270},
  {100 ,3238},
	{100 ,3208}, 
	{100 ,3191},
	{100 ,3172},
	{100 ,3159},
	{100 ,3150},
	{100 ,3137},
	{100 ,3137},
	{100 ,3137},
	{100 ,3137},
	{100 ,3137},
	{100 ,3137},
	{100 ,3137},
	{100 ,3137},
	{100 ,3137},
	{100 ,3137},
	{100 ,3137},
	{100 ,3137},
	{100 ,3137},
	{100 ,3137},
	{100 ,3137},
	{100 ,3137},
	{100 ,3137},
	{100 ,3137},
	{100 ,3137},
	{100 ,3137},
	{100 ,3137},
	{100 ,3137},
	{100 ,3137},
	{100 ,3137}, 
	{100 ,3137}	       
};           

// T2 25C
BATTERY_PROFILE_STRUC battery_profile_t2[] =
{
	{0   , 4165},         
	{1   , 4149},         
	{3   , 4136},         
	{4   , 4121},         
	{5   , 4110},         
	{7   , 4098},         
	{8   , 4086},         
	{9   , 4081},         
	{11  , 4077},         
	{12  , 4067},         
	{13  , 4047},         
	{15  , 4025},         
	{16  , 4006},         
	{17  , 3993},         
	{19  , 3983},         
	{20  , 3975},         
	{21  , 3971},         
	{23  , 3968},         
	{24  , 3964},         
	{25  , 3958},         
	{27  , 3949},         
	{28  , 3943},         
	{29  , 3934},         
	{31  , 3928},         
	{32  , 3920},         
	{34  , 3913},         
	{35  , 3906},         
	{36  , 3898},         
	{38  , 3890},         
	{39  , 3878},         
	{40  , 3865},         
	{42  , 3853},         
	{43  , 3843},         
	{44  , 3836},         
	{46  , 3829},         
	{47  , 3824},         
	{48  , 3820},         
	{50  , 3814},         
	{51  , 3812},         
	{52  , 3807},         
	{54  , 3803},         
	{55  , 3801},         
	{56  , 3796},         
	{58  , 3794},         
	{59  , 3791},         
	{60  , 3789},         
	{62  , 3786},         
	{63  , 3784},         
	{64  , 3782},         
	{66  , 3781},         
	{67  , 3779},         
	{68  , 3779},         
	{70  , 3777},         
	{71  , 3775},         
	{72  , 3772},         
	{74  , 3769},         
	{75  , 3765},         
	{76  , 3761},         
	{78  , 3757},         
	{79  , 3752},         
	{80  , 3747},          
  {82  , 3741},
  {83  , 3733},
  {84  , 3724},
  {86  , 3717},
  {87  , 3706},
  {88  , 3697},
  {90  , 3695},
  {91  , 3694},
  {92  , 3692},
	{94  , 3690}, 
	{95  , 3684},
	{97  , 3651},
	{98  , 3587},
	{99  , 3498},
	{100 , 3347},
	{100 , 3207},
	{100 , 3164},
	{100 , 3128},
	{100 , 3087},
	{100 , 3063},
	{100 , 3041},
	{100 , 3029},
	{100 , 3026},
	{100 , 3023},
	{100 , 3005},
	{100 , 2998},
	{100 , 2992},
	{100 , 2981},
	{100 , 2973},
	{100 , 2974},
	{100 , 2975},
	{100 , 2960},
	{100 , 2950},
	{100 , 2949},
	{100 , 2947},
	{100 , 2944},
	{100 , 2939},
	{100 , 2936},
	{100 , 2931}	       
};     

// T3 50C
BATTERY_PROFILE_STRUC battery_profile_t3[] =
{
	{0   , 4181},         
	{1   , 4167},         
	{3   , 4152},         
	{4   , 4139},         
	{5   , 4127},         
	{7   , 4114},         
	{8   , 4103},         
	{9   , 4090},         
	{11  , 4078},         
	{12  , 4067},         
	{13  , 4056},         
	{14  , 4049},         
	{16  , 4036},         
	{17  , 4022},         
	{18  , 4010},         
	{20  , 4001},         
	{21  , 3995},         
	{22  , 3986},         
	{24  , 3977},         
	{25  , 3969},         
	{26  , 3959},         
	{28  , 3952},         
	{29  , 3943},         
	{30  , 3935},         
	{31  , 3929},         
	{33  , 3920},         
	{34  , 3913},         
	{35  , 3906},         
	{37  , 3899},         
	{38  , 3893},         
	{39  , 3887},         
	{41  , 3879},         
	{42  , 3867},         
	{43  , 3851},         
	{45  , 3840},         
	{46  , 3833},         
	{47  , 3827},         
	{48  , 3820},         
	{50  , 3816},         
	{51  , 3812},         
	{52  , 3808},         
	{54  , 3803},         
	{55  , 3800},         
	{56  , 3797},         
	{58  , 3794},         
	{59  , 3791},         
	{60  , 3787},         
	{62  , 3785},         
	{63  , 3782},         
	{64  , 3779},         
	{66  , 3778},         
	{67  , 3776},         
	{68  , 3775},         
	{69  , 3772},         
	{71  , 3767},         
	{72  , 3759},         
	{73  , 3753},         
	{75  , 3751},         
	{76  , 3746},         
	{77  , 3742},         
	{79  , 3737},          
  {80  , 3732},
  {81  , 3729},
  {83  , 3724},
  {84  , 3715},
  {85  , 3708},
  {86  , 3699},
  {88  , 3689},
  {89  , 3681},
  {90  , 3680},
	{92  , 3680}, 
	{93  , 3678},
	{94  , 3676},
	{96  , 3664},
	{97  , 3619},
	{98  , 3553},
	{100 , 3454},
	{100 , 3279},
	{100 , 3141},
	{100 , 3081},
	{100 , 3038},
	{100 , 3012},
	{100 , 2982},
	{100 , 2976},
	{100 , 2956},
	{100 , 2947},
	{100 , 2942},
	{100 , 2936},
	{100 , 2939},
	{100 , 2926},
	{100 , 2925},
	{100 , 2922},
	{100 , 2918},
	{100 , 2910},
	{100 , 2904},
	{100 , 2897},
	{100 , 2891},
	{100 , 2881},
	{100 , 2873},
	{100 , 2876}	       
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
	{865  ,4098},         
	{865  ,4069},         
	{893  ,4053},         
	{915  ,4040},         
	{955  ,4023},         
	{1023 ,3997},         
	{1200 ,3961},         
	{1338 ,3946},         
	{1375 ,3938},         
	{1388 ,3932},         
	{1408 ,3926},         
	{1420 ,3918},         
	{1428 ,3910},         
	{1418 ,3901},         
	{1428 ,3894},         
	{1423 ,3885},         
	{1418 ,3874},         
	{1425 ,3866},         
	{1428 ,3856},         
	{1428 ,3846},         
	{1425 ,3838},         
	{1423 ,3830},         
	{1420 ,3823},         
	{1415 ,3817},         
	{1425 ,3814},         
	{1425 ,3808},         
	{1450 ,3806},         
	{1468 ,3803},         
	{1465 ,3801},         
	{1483 ,3798},         
	{1488 ,3795},         
	{1510 ,3796},         
	{1515 ,3795},         
	{1533 ,3792},         
	{1535 ,3792},         
	{1548 ,3790},         
	{1543 ,3789},         
	{1563 ,3787},         
	{1588 ,3785},         
	{1610 ,3783},         
	{1625 ,3781},         
	{1640 ,3776},         
	{1653 ,3772},         
	{1660 ,3767},         
	{1680 ,3763},         
	{1690 ,3758},         
	{1710 ,3751},         
	{1733 ,3742},         
	{1745 ,3734},         
	{1765 ,3725},         
	{1788 ,3719},         
	{1813 ,3715},         
	{1853 ,3712},         
	{1905 ,3707},         
	{1965 ,3702},         
	{2010 ,3696},         
	{2080 ,3678},         
	{2123 ,3647},         
	{2035 ,3612},         
	{1943 ,3575},         
	{1853 ,3537},          
  {1770 ,3502},
  {1685 ,3472},
  {1623 ,3443},
  {1550 ,3419},
  {1493 ,3395},
  {1448 ,3373},
  {1395 ,3357},
  {1368 ,3341},
  {1338 ,3328},
	{1303 ,3317}, 
	{1298 ,3307},
	{1263 ,3300},
	{1253 ,3293},
	{1260 ,3288},
	{1225 ,3283},
	{1240 ,3275},
	{1198 ,3271},
	{1215 ,3267},
	{1198 ,3260},
	{1200 ,3256},
	{1218 ,3251},
	{1228 ,3243},
	{1138 ,3239},
	{1230 ,3233},
	{1243 ,3225},
	{1155 ,3218},
	{1165 ,3214},
	{1045 ,3209},
	{1170 ,3202},
	{1183 ,3196},
	{1340 ,3185},
	{1368 ,3171},
	{1423 ,3157},
	{1455 ,3142},
	{1533 ,3125},
	{1365 ,3114},
	{1653 ,3095},
	{1653 ,3095}, 
	{1653 ,3095}	       
};      

// T1 0C
R_PROFILE_STRUC r_profile_t1[] =
{
	{633  , 4048},         
	{633  , 4008},         
	{678  , 3989},         
	{685  , 3977},         
	{700  , 3966},         
	{713  , 3960},         
	{728  , 3956},         
	{748  , 3951},         
	{753  , 3948},         
	{763  , 3941},         
	{763  , 3935},         
	{768  , 3928},         
	{783  , 3922},         
	{775  , 3914},         
	{780  , 3906},         
	{790  , 3898},         
	{790  , 3892},         
	{793  , 3882},         
	{798  , 3872},         
	{778  , 3860},         
	{778  , 3849},         
	{770  , 3839},         
	{778  , 3831},         
	{770  , 3824},         
	{785  , 3818},         
	{795  , 3815},         
	{785  , 3808},         
	{805  , 3805},         
	{810  , 3803},         
	{815  , 3798},         
	{818  , 3796},         
	{835  , 3793},         
	{838  , 3792},         
	{840  , 3790},         
	{865  , 3790},         
	{863  , 3788},         
	{880  , 3788},         
	{893  , 3787},         
	{908  , 3787},         
	{928  , 3785},         
	{933  , 3785},         
	{960  , 3784},         
	{965  , 3782},         
	{990  , 3779},         
	{1003 , 3777},         
	{1033 , 3774},         
	{1045 , 3769},         
	{1070 , 3766},         
	{1098 , 3762},         
	{1113 , 3756},         
	{1145 , 3748},         
	{1185 , 3742},         
	{1208 , 3734},         
	{1248 , 3724},         
	{1295 , 3714},         
	{1333 , 3708},         
	{1405 , 3703},         
	{1465 , 3701},         
	{1560 , 3699},         
	{1643 , 3696},         
	{1745 , 3689},          
  {1815 , 3662},
  {1863 , 3601},
  {1840 , 3533},
  {1688 , 3475},
  {1560 , 3418},
  {1418 , 3363},
  {1313 , 3315},
  {1200 , 3270},
  {1100 , 3238},
	{1060 , 3208}, 
	{980  , 3191},
	{1000 , 3172},
	{955  , 3159},
	{878  , 3150},
	{960  , 3137},
	{960  , 3137},
	{960  , 3137},
	{960  , 3137},
	{960  , 3137},
	{960  , 3137},
	{960  , 3137},
	{960  , 3137},
	{960  , 3137},
	{960  , 3137},
	{960  , 3137},
	{960  , 3137},
	{960  , 3137},
	{960  , 3137},
	{960  , 3137},
	{960  , 3137},
	{960  , 3137},
	{960  , 3137},
	{960  , 3137},
	{960  , 3137},
	{960  , 3137},
	{960  , 3137},
	{960  , 3137},
	{960  , 3137}, 
	{960  , 3137}	       
};     

// T2 25C
R_PROFILE_STRUC r_profile_t2[] =
{
	{250   , 4165},         
	{250   , 4149},         
	{243   , 4136},         
	{240   , 4121},         
	{250   , 4110},         
	{250   , 4098},         
	{248   , 4086},         
	{258   , 4081},         
	{273   , 4077},         
	{278   , 4067},         
	{263   , 4047},         
	{265   , 4025},         
	{263   , 4006},         
	{268   , 3993},         
	{263   , 3983},         
	{268   , 3975},         
	{283   , 3971},         
	{288   , 3968},         
	{290   , 3964},         
	{295   , 3958},         
	{288   , 3949},         
	{295   , 3943},         
	{295   , 3934},         
	{298   , 3928},         
	{298   , 3920},         
	{295   , 3913},         
	{298   , 3906},         
	{298   , 3898},         
	{293   , 3890},         
	{283   , 3878},         
	{270   , 3865},         
	{255   , 3853},         
	{243   , 3843},         
	{240   , 3836},         
	{240   , 3829},         
	{238   , 3824},         
	{238   , 3820},         
	{235   , 3814},         
	{243   , 3812},         
	{245   , 3807},         
	{245   , 3803},         
	{253   , 3801},         
	{243   , 3796},         
	{248   , 3794},         
	{250   , 3791},         
	{255   , 3789},         
	{253   , 3786},         
	{258   , 3784},         
	{258   , 3782},         
	{260   , 3781},         
	{258   , 3779},         
	{265   , 3779},         
	{268   , 3777},         
	{270   , 3775},         
	{265   , 3772},         
	{265   , 3769},         
	{273   , 3765},         
	{273   , 3761},         
	{270   , 3757},         
	{275   , 3752},         
	{278   , 3747},          
  {278   , 3741},
  {278   , 3733},
  {275   , 3724},
  {285   , 3717},
  {285   , 3706},
  {273   , 3697},
  {285   , 3695},
  {303   , 3694},
  {318   , 3692},
	{340   , 3690}, 
	{365   , 3684},
	{368   , 3651},
	{393   , 3587},
	{458   , 3498},
	{575   , 3347},
	{1070  , 3207},
	{933   , 3164},
	{863   , 3128},
	{830   , 3087},
	{710   , 3063},
	{663   , 3041},
	{640   , 3029},
	{570   , 3026},
	{583   , 3023},
	{655   , 3005},
	{575   , 2998},
	{675   , 2992},
	{630   , 2981},
	{665   , 2973},
	{610   , 2974},
	{528   , 2975},
	{673   , 2960},
	{703   , 2950},
	{590   , 2949},
	{473   , 2947},
	{693   , 2944},
	{725   , 2939},
	{483   , 2936}, 
	{480   , 2931}	       
}; 

// T3 50C
R_PROFILE_STRUC r_profile_t3[] =
{
	{138 , 4181},         
	{138 , 4167},         
	{138 , 4152},         
	{140 , 4139},         
	{140 , 4127},         
	{143 , 4114},         
	{143 , 4103},         
	{143 , 4090},         
	{140 , 4078},         
	{143 , 4067},         
	{145 , 4056},         
	{155 , 4049},         
	{153 , 4036},         
	{155 , 4022},         
	{155 , 4010},         
	{155 , 4001},         
	{160 , 3995},         
	{163 , 3986},         
	{163 , 3977},         
	{170 , 3969},         
	{163 , 3959},         
	{173 , 3952},         
	{173 , 3943},         
	{175 , 3935},         
	{180 , 3929},         
	{178 , 3920},         
	{178 , 3913},         
	{180 , 3906},         
	{180 , 3899},         
	{190 , 3893},         
	{190 , 3887},         
	{190 , 3879},         
	{180 , 3867},         
	{158 , 3851},         
	{145 , 3840},         
	{143 , 3833},         
	{140 , 3827},         
	{138 , 3820},         
	{138 , 3816},         
	{143 , 3812},         
	{145 , 3808},         
	{145 , 3803},         
	{145 , 3800},         
	{150 , 3797},         
	{153 , 3794},         
	{153 , 3791},         
	{158 , 3787},         
	{155 , 3785},         
	{160 , 3782},         
	{160 , 3779},         
	{160 , 3778},         
	{163 , 3776},         
	{168 , 3775},         
	{163 , 3772},         
	{158 , 3767},         
	{148 , 3759},         
	{145 , 3753},         
	{150 , 3751},         
	{148 , 3746},         
	{150 , 3742},         
	{150 , 3737},          
  {148 , 3732},
  {155 , 3729},
  {158 , 3724},
  {150 , 3715},
  {155 , 3708},
  {153 , 3699},
  {150 , 3689},
  {143 , 3681},
  {150 , 3680},
	{160 , 3680}, 
	{168 , 3678},
	{180 , 3676},
	{180 , 3664},
	{170 , 3619},
	{188 , 3553},
	{205 , 3454},
	{300 , 3279},
	{858 , 3141},
	{783 , 3081},
	{653 , 3038},
	{530 , 3012},
	{515 , 2982},
	{458 , 2976},
	{498 , 2956},
	{475 , 2947},
	{440 , 2942},
	{425 , 2936},
	{383 , 2939},
	{415 , 2926},
	{330 , 2925},
	{320 , 2922},
	{325 , 2918},
	{385 , 2910},
	{340 , 2904},
	{353 , 2897},
	{358 , 2891},
	{365 , 2881},
	{385 , 2873}, 
	{320 , 2876}	       
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

