const uint32_t PROGMEM AnimFwdpixelData[] =
{
0x00FF3B5E,
};
const uint32_t PROGMEM AnimFwdcolorTable[] =
{
0x00FF00, 0x000000, 
};

ANIM AnimFwdAnim = { AnimFwdpixelData, AnimFwdcolorTable, 6, 4, 1 };

// 0,1,1,1,1,0,
// 1,0,1,1,0,1,
// 1,1,0,0,1,1,
// 1,1,1,1,1,1,