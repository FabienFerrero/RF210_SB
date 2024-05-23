// Define this to get the settings for the four test nodes, or leave it undefined to fill in your own settings in the #else block below
#define SB_NB 1

#if SB_NB == 1
String devAddr = "260BE20F";
String nwkkey = "F1479782F58401DD346C1A0128068EE0";
String appskey = "74D90F854110CDAFB75F70AC25F1C3D0";

#elif SB_NB == 2
static String devAddr = "260B3EA6";
static String nwkkey = "42DEA5D05476EF2C58A55EBC0B292C85";
static String appskey = "1699AA035E46FA789FFA2493E4463F6D";

#elif SB_NB == 3
static String devAddr = "260B2666";
static String nwkkey = "213BE0DCC732E4A8189BBCC0413DD0AC";
static String appskey = "B9F696C1AA9EE902907BC0C380702CF1";

#elif SB_NB == 4
static String devAddr = "260B5DDA";
static String nwkkey = "7687AA7D8AFA030A4CCB1BB5633163B5";
static String appskey = "97271FB0D13E85071D1949599178FE6F";


#else
///////////////////////////////////
// Fill in your own settings here
///////////////////////////////////
String devAddr = "260B3500";
String nwkkey = "BD1C68CF9ED71883A1BB2E53CCEA07A6";
String appskey = "525DDB7FDDF6BCFB07791A3C330640D5";

#endif // LACUNA_TEST