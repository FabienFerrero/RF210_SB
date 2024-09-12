// Define this to get the settings for the four test nodes, or leave it undefined to fill in your own settings in the #else block below
#define SB_NB 16

#if SB_NB == 1
String devAddr = "260BE20F";
String nwkkey = "F1479782F58401DD346C1A0128068EE0"; 
String appskey = "9B431A58B90A18D452861898F5807E8F";

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

#elif SB_NB == 5
static String devAddr = "260B8FFD";
static String nwkkey = "98FBACA514667539D238FAC95CB10E22";
static String appskey = "77963498E631D2BBFD89163C339B3BA0";

#elif SB_NB == 6
static String devAddr = "260BEAF5";
static String nwkkey = "C113EC62F68F7DF61AF618327A9D0F4E";
static String appskey = "42D047D45813BC94FACB0E9B1FF8D2B7";

#elif SB_NB == 7
static String devAddr = "260BAFAD";
static String nwkkey = "89E4B4FB5DFD509FA2E4FB4A3524E3FE";
static String appskey = "DD4EFCAD82E16F3F4F343B5030AF241A";

#elif SB_NB == 8
static String devAddr = "260B0E81";
static String nwkkey = "CBB1EB724D6C88AAB0458F72DE23D3F2";
static String appskey = "D29F2AE5874DDF2133F42EFE5D1EBC1D";

#elif SB_NB == 9
static String devAddr = "260BA6F4";
static String nwkkey = "36074AF6A031D17501F65F2BA8422DB6";
static String appskey = "17F8BB7696CFB64F38D3D56A5A61B9D8";

#elif SB_NB == 10
static String devAddr = "260B50AA";
static String nwkkey = "6A3CC8D765B2C7179486F3AE04393F23";
static String appskey = "4837D8FA753DBED13B08368211E20478";

#elif SB_NB == 11
static String devAddr = "260BB748";
static String nwkkey = "8A96C699D81D3C79FBC03FD660DD4A36";
static String appskey = "77B4EA9CDAF67B416E0F71699CBD985C";

#elif SB_NB == 12
static String devAddr = "260B1744";
static String nwkkey = "F8A8C07AECAFF89F5BE376DF733B9131";
static String appskey = "96211B13CB4958A9C0A73455684E4086";

#elif SB_NB == 13
static String devAddr = "260BF7F4";
static String nwkkey = "18CD1E14230358BAB9898A71335DCE2D";
static String appskey = "DADA67DB1EDA14D589C69B5B1EC553E3";

#elif SB_NB == 14
static String devAddr = "260B5653";
static String nwkkey = "7546BB9D0A93C8FB088A5B19CFC3DE65";
static String appskey = "2EA455003A1F17C0036B74D1F384A8F4";

#elif SB_NB == 15
static String devAddr = "260BC6EC";
static String nwkkey = "4C400DFF11B1F189A52B9560A4E49D78";
static String appskey = "1CC6EF22C36FB4DC83642045B3245369";

#elif SB_NB == 16
static String devAddr = "260BA47A";
static String nwkkey = "191BA678F6D3FDD16E2E930D3603BF4E";
static String appskey = "85EBCED9E3F1BC78B453A2FBE85433A6";



#else
///////////////////////////////////
// Fill in your own settings here
////////////////////////////////  ///
String devAddr = "260B3500";
String nwkkey = "BD1C68CF9ED71883A1BB2E53CCEA07A6";
String appskey = "525DDB7FDDF6BCFB07791A3C330640D5";

#endif // LACUNA_TEST