// IBT Processor LUT  BER  2022.03.16
// Lookup Table for converting log detector to linear power responce.  11.94dB range segments
// AD8318 detector => ADS1115 @ +/- 2.048GV.  Uwse 1/2 of ADC range
// 0 to 32767 data I/O range.
// Table below from spreadsheet calcs.  32767 = Full Scale.
#define LogScale 7                  // 7 segments of log curve.
#define LogRange 4681               // = floor ( 32767 / LogScale )

#define TableShift 7          // Shift right count for 255 full scale to PWM
#define TableLen 1025         // dummy added on end for interp process
const unsigned int lookup[TableLen] PROGMEM = {
// ,// AD8318 span => ADS1115  with software offaset function
2101,//
2107,//
2112,//
2118,//
2124,//
2129,//
2135,//
2141,//
2147,//
2152,//
2158,//
2164,//
2170,//
2176,//
2182,//
2187,//
2193,//
2199,//
2205,//
2211,//
2217,//
2223,//
2229,//
2235,//
2241,//
2247,//
2253,//
2259,//
2265,//
2271,//
2277,//
2283,//
2290,//
2296,//
2302,//
2308,//
2314,//
2321,//
2327,//
2333,//
2339,//
2346,//
2352,//
2358,//
2365,//
2371,//
2377,//
2384,//
2390,//
2397,//
2403,//
2409,//
2416,//
2422,//
2429,//
2435,//
2442,//
2449,//
2455,//
2462,//
2468,//
2475,//
2482,//
2488,//
2495,//
2502,//
2508,//
2515,//
2522,//
2529,//
2536,//
2542,//
2549,//
2556,//
2563,//
2570,//
2577,//
2584,//
2591,//
2598,//
2605,//
2612,//
2619,//
2626,//
2633,//
2640,//
2647,//
2654,//
2661,//
2668,//
2675,//
2683,//
2690,//
2697,//
2704,//
2712,//
2719,//
2726,//
2734,//
2741,//
2748,//
2756,//
2763,//
2770,//
2778,//
2785,//
2793,//
2800,//
2808,//
2815,//
2823,//
2831,//
2838,//
2846,//
2854,//
2861,//
2869,//
2877,//
2884,//
2892,//
2900,//
2908,//
2915,//
2923,//
2931,//
2939,//
2947,//
2955,//
2963,//
2971,//
2979,//
2987,//
2995,//
3003,//
3011,//
3019,//
3027,//
3035,//
3043,//
3052,//
3060,//
3068,//
3076,//
3085,//
3093,//
3101,//
3110,//
3118,//
3126,//
3135,//
3143,//
3152,//
3160,//
3169,//
3177,//
3186,//
3194,//
3203,//
3211,//
3220,//
3229,//
3237,//
3246,//
3255,//
3264,//
3272,//
3281,//
3290,//
3299,//
3308,//
3317,//
3325,//
3334,//
3343,//
3352,//
3361,//
3370,//
3380,//
3389,//
3398,//
3407,//
3416,//
3425,//
3434,//
3444,//
3453,//
3462,//
3471,//
3481,//
3490,//
3500,//
3509,//
3518,//
3528,//
3537,//
3547,//
3556,//
3566,//
3576,//
3585,//
3595,//
3604,//
3614,//
3624,//
3634,//
3643,//
3653,//
3663,//
3673,//
3683,//
3693,//
3703,//
3713,//
3723,//
3733,//
3743,//
3753,//
3763,//
3773,//
3783,//
3793,//
3803,//
3814,//
3824,//
3834,//
3844,//
3855,//
3865,//
3876,//
3886,//
3896,//
3907,//
3917,//
3928,//
3938,//
3949,//
3960,//
3970,//
3981,//
3992,//
4002,//
4013,//
4024,//
4035,//
4046,//
4057,//
4067,//
4078,//
4089,//
4100,//
4111,//
4122,//
4133,//
4145,//
4156,//
4167,//
4178,//
4189,//
4201,//
4212,//
4223,//
4235,//
4246,//
4257,//
4269,//
4280,//
4292,//
4303,//
4315,//
4327,//
4338,//
4350,//
4362,//
4373,//
4385,//
4397,//
4409,//
4420,//
4432,//
4444,//
4456,//
4468,//
4480,//
4492,//
4504,//
4516,//
4529,//
4541,//
4553,//
4565,//
4578,//
4590,//
4602,//
4615,//
4627,//
4639,//
4652,//
4664,//
4677,//
4690,//
4702,//
4715,//
4727,//
4740,//
4753,//
4766,//
4778,//
4791,//
4804,//
4817,//
4830,//
4843,//
4856,//
4869,//
4882,//
4895,//
4909,//
4922,//
4935,//
4948,//
4962,//
4975,//
4988,//
5002,//
5015,//
5029,//
5042,//
5056,//
5069,//
5083,//
5097,//
5110,//
5124,//
5138,//
5152,//
5165,//
5179,//
5193,//
5207,//
5221,//
5235,//
5249,//
5263,//
5278,//
5292,//
5306,//
5320,//
5335,//
5349,//
5363,//
5378,//
5392,//
5407,//
5421,//
5436,//
5450,//
5465,//
5480,//
5495,//
5509,//
5524,//
5539,//
5554,//
5569,//
5584,//
5599,//
5614,//
5629,//
5644,//
5659,//
5674,//
5690,//
5705,//
5720,//
5736,//
5751,//
5767,//
5782,//
5798,//
5813,//
5829,//
5845,//
5860,//
5876,//
5892,//
5908,//
5924,//
5940,//
5955,//
5971,//
5988,//
6004,//
6020,//
6036,//
6052,//
6068,//
6085,//
6101,//
6118,//
6134,//
6151,//
6167,//
6184,//
6200,//
6217,//
6234,//
6250,//
6267,//
6284,//
6301,//
6318,//
6335,//
6352,//
6369,//
6386,//
6403,//
6421,//
6438,//
6455,//
6472,//
6490,//
6507,//
6525,//
6542,//
6560,//
6578,//
6595,//
6613,//
6631,//
6649,//
6666,//
6684,//
6702,//
6720,//
6738,//
6757,//
6775,//
6793,//
6811,//
6830,//
6848,//
6866,//
6885,//
6903,//
6922,//
6940,//
6959,//
6978,//
6997,//
7015,//
7034,//
7053,//
7072,//
7091,//
7110,//
7129,//
7149,//
7168,//
7187,//
7206,//
7226,//
7245,//
7265,//
7284,//
7304,//
7323,//
7343,//
7363,//
7383,//
7402,//
7422,//
7442,//
7462,//
7482,//
7503,//
7523,//
7543,//
7563,//
7584,//
7604,//
7624,//
7645,//
7665,//
7686,//
7707,//
7727,//
7748,//
7769,//
7790,//
7811,//
7832,//
7853,//
7874,//
7895,//
7916,//
7938,//
7959,//
7981,//
8002,//
8023,//
8045,//
8067,//
8088,//
8110,//
8132,//
8154,//
8176,//
8198,//
8220,//
8242,//
8264,//
8286,//
8309,//
8331,//
8353,//
8376,//
8398,//
8421,//
8443,//
8466,//
8489,//
8512,//
8535,//
8558,//
8581,//
8604,//
8627,//
8650,//
8673,//
8697,//
8720,//
8743,//
8767,//
8790,//
8814,//
8838,//
8862,//
8885,//
8909,//
8933,//
8957,//
8981,//
9006,//
9030,//
9054,//
9078,//
9103,//
9127,//
9152,//
9176,//
9201,//
9226,//
9251,//
9275,//
9300,//
9325,//
9351,//
9376,//
9401,//
9426,//
9451,//
9477,//
9502,//
9528,//
9554,//
9579,//
9605,//
9631,//
9657,//
9683,//
9709,//
9735,//
9761,//
9787,//
9814,//
9840,//
9866,//
9893,//
9920,//
9946,//
9973,//
10000,//
10027,//
10054,//
10081,//
10108,//
10135,//
10162,//
10190,//
10217,//
10244,//
10272,//
10300,//
10327,//
10355,//
10383,//
10411,//
10439,//
10467,//
10495,//
10523,//
10551,//
10580,//
10608,//
10637,//
10665,//
10694,//
10723,//
10752,//
10781,//
10810,//
10839,//
10868,//
10897,//
10926,//
10956,//
10985,//
11015,//
11044,//
11074,//
11104,//
11134,//
11164,//
11194,//
11224,//
11254,//
11284,//
11314,//
11345,//
11375,//
11406,//
11437,//
11467,//
11498,//
11529,//
11560,//
11591,//
11622,//
11654,//
11685,//
11716,//
11748,//
11779,//
11811,//
11843,//
11875,//
11907,//
11939,//
11971,//
12003,//
12035,//
12068,//
12100,//
12133,//
12165,//
12198,//
12231,//
12264,//
12297,//
12330,//
12363,//
12396,//
12429,//
12463,//
12496,//
12530,//
12564,//
12597,//
12631,//
12665,//
12699,//
12733,//
12768,//
12802,//
12836,//
12871,//
12906,//
12940,//
12975,//
13010,//
13045,//
13080,//
13115,//
13150,//
13186,//
13221,//
13257,//
13292,//
13328,//
13364,//
13400,//
13436,//
13472,//
13508,//
13545,//
13581,//
13618,//
13654,//
13691,//
13728,//
13765,//
13802,//
13839,//
13876,//
13913,//
13951,//
13988,//
14026,//
14064,//
14101,//
14139,//
14177,//
14215,//
14254,//
14292,//
14330,//
14369,//
14408,//
14446,//
14485,//
14524,//
14563,//
14602,//
14642,//
14681,//
14720,//
14760,//
14800,//
14839,//
14879,//
14919,//
14959,//
15000,//
15040,//
15080,//
15121,//
15162,//
15202,//
15243,//
15284,//
15325,//
15367,//
15408,//
15449,//
15491,//
15533,//
15574,//
15616,//
15658,//
15700,//
15743,//
15785,//
15827,//
15870,//
15913,//
15955,//
15998,//
16041,//
16084,//
16128,//
16171,//
16214,//
16258,//
16302,//
16346,//
16390,//
16434,//
16478,//
16522,//
16567,//
16611,//
16656,//
16701,//
16745,//
16790,//
16836,//
16881,//
16926,//
16972,//
17017,//
17063,//
17109,//
17155,//
17201,//
17247,//
17294,//
17340,//
17387,//
17434,//
17481,//
17528,//
17575,//
17622,//
17669,//
17717,//
17764,//
17812,//
17860,//
17908,//
17956,//
18005,//
18053,//
18102,//
18150,//
18199,//
18248,//
18297,//
18346,//
18396,//
18445,//
18495,//
18544,//
18594,//
18644,//
18694,//
18745,//
18795,//
18846,//
18896,//
18947,//
18998,//
19049,//
19100,//
19152,//
19203,//
19255,//
19307,//
19358,//
19410,//
19463,//
19515,//
19567,//
19620,//
19673,//
19726,//
19779,//
19832,//
19885,//
19939,//
19992,//
20046,//
20100,//
20154,//
20208,//
20263,//
20317,//
20372,//
20426,//
20481,//
20536,//
20592,//
20647,//
20703,//
20758,//
20814,//
20870,//
20926,//
20982,//
21039,//
21095,//
21152,//
21209,//
21266,//
21323,//
21381,//
21438,//
21496,//
21553,//
21611,//
21670,//
21728,//
21786,//
21845,//
21904,//
21962,//
22021,//
22081,//
22140,//
22200,//
22259,//
22319,//
22379,//
22439,//
22500,//
22560,//
22621,//
22682,//
22743,//
22804,//
22865,//
22927,//
22988,//
23050,//
23112,//
23174,//
23236,//
23299,//
23362,//
23424,//
23487,//
23551,//
23614,//
23677,//
23741,//
23805,//
23869,//
23933,//
23997,//
24062,//
24127,//
24192,//
24257,//
24322,//
24387,//
24453,//
24519,//
24584,//
24651,//
24717,//
24783,//
24850,//
24917,//
24984,//
25051,//
25118,//
25186,//
25254,//
25321,//
25390,//
25458,//
25526,//
25595,//
25664,//
25733,//
25802,//
25871,//
25941,//
26011,//
26081,//
26151,//
26221,//
26291,//
26362,//
26433,//
26504,//
26575,//
26647,//
26718,//
26790,//
26862,//
26935,//
27007,//
27080,//
27152,//
27225,//
27299,//
27372,//
27446,//
27519,//
27593,//
27668,//
27742,//
27817,//
27891,//
27966,//
28042,//
28117,//
28193,//
28268,//
28344,//
28421,//
28497,//
28574,//
28651,//
28728,//
28805,//
28882,//
28960,//
29038,//
29116,//
29194,//
29273,//
29351,//
29430,//
29509,//
29589,//
29668,//
29748,//
29828,//
29908,//
29989,//
30069,//
30150,//
30231,//
30313,//
30394,//
30476,//
30558,//
30640,//
30722,//
30805,//
30888,//
30971,//
31054,//
31138,//
31221,//
31305,//
31389,//
31474,//
31558,//
31643,//
31728,//
31814,//
31899,//
31985,//
32071,//
32157,//
32244,//
32330,//
32417,//
32504,//
32592,//
32679,//
32767,//
32767 // dummy on end
};