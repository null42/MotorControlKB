// 复制此内容修改原 ACMSim.h 第 6-10 行

// === 原内容 ===
// #if PC_SIMULATION
//     #define __INVERTER_NONLINEARITY 0   
// #endif

// === 修改为 ===
#if PC_SIMULATION
    // 0=理想  1=Sul1996数学建模  2=实验Sigmoid拟合  
    // 3=查表法(插值)  4=查表法(索引)
    #define __INVERTER_NONLINEARITY 1
#endif
