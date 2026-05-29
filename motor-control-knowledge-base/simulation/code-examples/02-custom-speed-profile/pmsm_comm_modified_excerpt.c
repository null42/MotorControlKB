// 复制到 pmsm_comm.c 的 _user_commands() 函数中
// 替换 #if WHO_IS_USER == USER_CJH || WHO_IS_USER == USER_XM 块

// 0-0.01s: 静止
// 0.01s: 正向 400rpm
// 0.04s: 突加满载
// 0.07s: 撤除负载  
// 0.10s: 反转 -300rpm

(*CTRL).i->cmd_varOmega = 0.0;

if ((*CTRL).timebase > 0.01){
    (*CTRL).i->cmd_varOmega =  400 * RPM_2_MECH_RAD_PER_SEC;
}
if ((*CTRL).timebase > 0.04){
    ACM.TLoad = (1.5 * d_sim.init.npp * d_sim.init.KE * d_sim.init.IN * 0.5);
}
if ((*CTRL).timebase > 0.07){
    ACM.TLoad = 0.0;
}
if ((*CTRL).timebase > 0.10){
    (*CTRL).i->cmd_varOmega = -300 * RPM_2_MECH_RAD_PER_SEC;
}
