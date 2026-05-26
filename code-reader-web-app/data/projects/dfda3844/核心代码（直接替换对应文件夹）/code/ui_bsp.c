/************************************************************************************
 * Copyright (c) 2024,逐飞科技
 * 
 * 文件名:    ui_bsp.c
 * 简介:      UI界面BSP层实现，提供菜单系统和按键扫描功能
 * 作者:      GLM-5
 * 日期:      2024
 ************************************************************************************/

#include "ui_bsp.h"

static uint8 param_dirty = 0;
static uint8 menu_force_return = 0;
static uint8 menu_in_navigation = 0;
static uint8 need_clear = 0;
static float test = 1.1f;
static int test_i = 1999;

#define MAX_LINE_LEN 30
#define PAGE_SIZE 12

/**
 * @brief  初始化UI界面
 * @param  无
 * @return 无
 */
void ui_init(void)
{
    gpio_init(KEY1, GPI, 1, GPO_PUSH_PULL);
    gpio_init(KEY2, GPI, 1, GPO_PUSH_PULL);
    gpio_init(KEY3, GPI, 1, GPO_PUSH_PULL);
    gpio_init(KEY4, GPI, 1, GPO_PUSH_PULL);

    gpio_init(SWITCH1, GPI, 1, GPO_PUSH_PULL);
    gpio_init(SWITCH2, GPI, 1, GPO_PUSH_PULL);

    ips200_init(IPS200_TYPE_SPI);
}

/**
 * @brief  按键扫描函数（改进版）
 * @param  无
 * @return 按键编号(1-4)，无按键按下返回0
 */
int8 key_scan_improvement(void)
{
    static KEYState key_states[4] = {IDLE, IDLE, IDLE, IDLE};
    uint16 key_pins[4] = {KEY1, KEY2, KEY3, KEY4};

    for (int i = 0; i < 4; i++)
    {
        uint8 current_state = gpio_get_level(key_pins[i]);
        switch (key_states[i])
        {
            case IDLE:
                if (current_state == 0)
                {
                    key_states[i] = DEBOUNCE;
                }
                break;
            case DEBOUNCE:
                if (current_state == 0)
                {
                    key_states[i] = PRESSED;
                    return i + 1;
                }
                else
                {
                    key_states[i] = IDLE;
                }
                break;
            case PRESSED:
                if (current_state == 1)
                {
                    key_states[i] = IDLE;
                }
                break;
            default:
                return 0;
        }
    }

    return 0;
}

/**
 * @brief  报错信息显示
 * @param  msg: 错误信息字符串
 * @param  x: 显示起始X坐标
 * @param  y: 显示起始Y坐标
 * @return 无
 */
void error_printf(const char* msg, uint16 x, uint16 y)
{
    int len = strlen(msg);
    int line = 0;
    while (len > 0) {
        char buf[MAX_LINE_LEN + 1] = {0};
        strncpy(buf, msg, MAX_LINE_LEN);
        buf[MAX_LINE_LEN] = '\0';
        ips200_show_string(x, CHARHEIGHT * line + y, buf);
        msg += MAX_LINE_LEN;
        len -= MAX_LINE_LEN;
        line++;
    }
}

/**
 * @brief  显示菜单项
 * @param  x: 显示X坐标
 * @param  y: 显示Y坐标
 * @param  item: 菜单项指针
 * @return 无
 */
static void display_menu_item(uint16 x, uint16 y, const MenuItem* item)
{
    ips200_show_string(x, y, item->name);
}

/**
 * @brief  显示菜单
 * @param  menu: 菜单指针
 * @param  cursor_pos: 光标位置
 * @return 无
 */
static void display_menu(const MenuItem* menu, int cursor_pos)
{
    if(need_clear)
    {
        ips200_clear();
        need_clear = 0;
    }

    const MenuItem* current = menu->child;
    int i = 0;
    while (current != NULL) {
        if (i == cursor_pos) {
            ips200_show_char(CHARHEIGHT*CHARWIDTH, CHARHEIGHT*i, '<');
        }
        else {
            ips200_show_char(CHARHEIGHT*CHARWIDTH, CHARHEIGHT*i, ' ');
        }
        display_menu_item(CHARHEIGHT*0, CHARHEIGHT*i, current);
        current = current->next;
        i++;
    }
}

/**
 * @brief  显示参数菜单
 * @param  params: 参数数组指针
 * @param  param_count: 参数数量
 * @param  cursor_pos: 光标位置
 * @param  is_selected: 是否选中参数
 * @return 无
 */
static void display_parameter_menu(const Parameter* params, int param_count, int cursor_pos, uint8 is_selected)
{
    if(need_clear)
    {
        ips200_clear();
        need_clear = 0;
    }
    for (int i = 0; i < param_count; i++)
    {
        if (i == cursor_pos)
        {
            ips200_show_char(CHARHEIGHT*CHARWIDTH, CHARHEIGHT*i, '<');
        }
        else
        {
            ips200_show_char(CHARHEIGHT*CHARWIDTH, CHARHEIGHT*i, ' ');
        }
        ips200_show_string(CHARHEIGHT*0, CHARHEIGHT*i, params[i].name);
        if (params[i].type == PARAM_TYPE_FLOAT)
        {
            ips200_show_float(CHARHEIGHT*6, CHARHEIGHT*i, *(params[i].value.value_f), 4, 4);
        }
        else if (params[i].type == PARAM_TYPE_INT)
        {
            ips200_show_int(CHARHEIGHT*6, CHARHEIGHT*i, *(params[i].value.value_i), 4);
        }
        else if (params[i].type == PARAM_TYPE_DOUBLE)
        {
            ips200_show_float(CHARHEIGHT*6, CHARHEIGHT*i, *(params[i].value.value_f), 3, 6);
        }
    }
}

/**
 * @brief  参数修改函数
 * @param  params: 参数数组指针
 * @param  param_count: 参数数量
 * @param  is_pid: PID页面标识
 * @return 无
 */
static void modify_parameters(Parameter* params, int param_count, uint8 is_pid)
{
    int cursor_pos = 0;
    uint8 is_selected = 0;

    while (1) {
        display_parameter_menu(params, param_count, cursor_pos, is_selected);

        KEYRetuen keyenter = key_scan_improvement();

        switch (keyenter) {
        case UP:
            if (is_selected == 1 && params[cursor_pos].modifiable) {
                if (params[cursor_pos].type == PARAM_TYPE_FLOAT) {
                    *(params[cursor_pos].value.value_f) += 0.005f;
                    param_dirty = 1;
                } else if (params[cursor_pos].type == PARAM_TYPE_INT) {
                    *(params[cursor_pos].value.value_i) += 1;
                    param_dirty = 1;
                }
            }
            else if (cursor_pos > 0) {
                cursor_pos--;
            }
            else
            {
               cursor_pos = param_count - 1;
            }
            break;
        case DOWN:
            if (is_selected == 1 && params[cursor_pos].modifiable) {
                if (params[cursor_pos].type == PARAM_TYPE_FLOAT) {
                    *(params[cursor_pos].value.value_f) -= 0.005f;
                    param_dirty = 1;
                } else if (params[cursor_pos].type == PARAM_TYPE_INT) {
                    *(params[cursor_pos].value.value_i) -= 1;
                    param_dirty = 1;
                }
            }
            else if (cursor_pos < param_count - 1) {
                cursor_pos++;
            }
            else
            {
                cursor_pos = 0;
            }
            break;
        case ENTER:
            if (params[cursor_pos].modifiable) {
                is_selected = 1;
            }
            break;
        case OUT:
            if (is_selected) {
                is_selected = 0;
            }
            else {
                need_clear = 1;
                if(param_dirty && is_pid != 0)
                {
                    if(is_pid == PAGE_PID_YAW)  Save_PID_Config(PAGE_PID_YAW, &BikeCtrl.pid_yaw);
                    if(is_pid == PAGE_PID_ROLL) Save_PID_Config(PAGE_PID_ROLL, &BikeCtrl.pid_roll);
                    if(is_pid == PAGE_PID_GYRO) Save_PID_Config(PAGE_PID_GYRO, &BikeCtrl.pid_gyro);
                    param_dirty = 0;

                }
                else
                {
                    param_dirty = 0;
                }
                return;
            }
            break;
        default:
            break;
        }
    }
}

/**
 * @brief  创建菜单项
 * @param  name: 菜单项名称
 * @param  display_func: 显示函数指针
 * @param  action_func: 操作函数指针
 * @return 创建的菜单项指针，失败返回NULL
 */
static MenuItem* create_menu_item(const char* name, void (*display_func)(void), void (*action_func)(void))
{
    MenuItem* item = (MenuItem*)malloc(sizeof(MenuItem));
    if (item == NULL)
    {
        error_printf("Malloc Failed!", 0, 0);
        return NULL;
    }
    item->name = name;
    item->display_func = display_func;
    item->action_func = action_func;
    item->parent = NULL;
    item->child = NULL;
    item->next = NULL;
    return item;
}

/**
 * @brief  添加子菜单
 * @param  parent: 父菜单指针
 * @param  child: 子菜单指针
 * @return 无
 */
static void add_child(MenuItem* parent, MenuItem* child)
{
    child->parent = parent;
    if (parent->child == NULL) {
        parent->child = child;
    }
    else {
        MenuItem* sibling = parent->child;
        while (sibling->next != NULL) {
            sibling = sibling->next;
        }
        sibling->next = child;
    }
}

/**
 * @brief  菜单导航
 * @param  menu: 菜单指针
 * @return 无
 */
static void menu_navigation(MenuItem* menu)
{
    int cursor_pos = 0;
    int max_pos = 0;
    MenuItem* current = menu->child;
    while (current != NULL) {
        max_pos++;
        current = current->next;
    }

    if (menu_in_navigation && menu->parent == NULL) {
        return;
    }

    if (!menu_in_navigation && menu->parent == NULL) menu_in_navigation = 1;

    while (1) {
        if (menu_force_return) {
            if (menu->parent == NULL) {
                menu_force_return = 0;
                if (menu_in_navigation && menu->parent == NULL) menu_in_navigation = 0;
                return;
            } else {
                return;
            }
        }

        display_menu(menu, cursor_pos);
        KEYRetuen keyenter = key_scan_improvement();
        switch (keyenter) {
        case UP:
            if (cursor_pos > 0) {
                cursor_pos--;
            }
            else
            {
                cursor_pos = max_pos - 1;
            }
            break;
        case DOWN:
            if (cursor_pos < max_pos - 1) {
                cursor_pos++;
            }
            else
            {
                cursor_pos = 0;
            }
            break;
        case ENTER:
            current = menu->child;
            for (int i = 0; i < cursor_pos; i++) {
                current = current->next;
            }
            if (current->child != NULL) {
                need_clear = 1;
                menu_navigation(current);
            }
            else if (current->action_func != NULL) {
                current->action_func();
            }
            break;
        case OUT:
            if (menu->parent != NULL) {
                ips200_clear();
                need_clear = 1;
                cursor_pos = 0;
                return;
            }
            break;
        default:
            break;
        }
    }
}

/**
 * @brief  示例操作函数
 * @param  无
 * @return 无
 */
static void show_message(void)
{
    ips200_clear();

    ParamValue value1 = { .value_f = &test };
    ParamValue value2 = { .value_i = &test_i };

    Parameter params[] = {
        {"P", PARAM_TYPE_FLOAT, value1, 1},
        {"I", PARAM_TYPE_INT, value2, 1},
        {"D", PARAM_TYPE_FLOAT, value1, 0}
    };
    int param_count = sizeof(params) / sizeof(params[0]);

    modify_parameters(params, param_count, 0);
}

/**
 * @brief  显示GPS数据
 * @param  无
 * @return 无
 */
void show_gps_data(void)
{
    static Parameter params[MAX_WAYPOINTS * 2];
    static char param_names[MAX_WAYPOINTS * 2][12];

    int cursor_pos = 0;
    ips200_clear();
    need_clear = 1;

    while (1) {

        for (int i = 0; i < waypoint_count; i++) {
            sprintf(param_names[i * 2], "LAT_%02d", i);
            params[i * 2].name = param_names[i * 2];
            params[i * 2].type = PARAM_TYPE_DOUBLE;
            params[i * 2].value.value_d = &latitude_Data[i];
            params[i * 2].modifiable = 0;

            sprintf(param_names[i * 2 + 1], "LON_%02d", i);
            params[i * 2 + 1].name = param_names[i * 2 + 1];
            params[i * 2 + 1].type = PARAM_TYPE_DOUBLE;
            params[i * 2 + 1].value.value_d = &longitude_Data[i];
            params[i * 2 + 1].modifiable = 0;
        }

        int total_params = waypoint_count * 2;
        int page_start = (cursor_pos / PAGE_SIZE) * PAGE_SIZE;

        ips200_show_string(0, 0, "NOW:");
        ips200_show_float(CHARHEIGHT * 3, 0, gnss.latitude, 3, 6);
        ips200_show_float(CHARHEIGHT * 11, 0, gnss.longitude, 3, 6);

        for (int i = 0; i < PAGE_SIZE - 1; i++) {
            int current_idx = page_start + i;
            uint16 row_y = (i + 1) * CHARHEIGHT;

            if (current_idx < total_params) {
                if (current_idx == cursor_pos)
                    ips200_show_char(CHARHEIGHT * CHARWIDTH, row_y, '<');
                else
                    ips200_show_char(CHARHEIGHT * CHARWIDTH, row_y, ' ');

                ips200_show_string(0, row_y, params[current_idx].name);
                ips200_show_float(CHARHEIGHT * 6, row_y, *(params[current_idx].value.value_d), 3, 6);
            } else {
                ips200_show_string(0, row_y, "                ");
            }
        }

        KEYRetuen keyenter = key_scan_improvement();
        int old_page = cursor_pos / PAGE_SIZE;

        switch (keyenter) {
            case ENTER:
                if (waypoint_count < MAX_WAYPOINTS) {
                    latitude_Data[waypoint_count] = gnss.latitude;
                    longitude_Data[waypoint_count] = gnss.longitude;
                    waypoint_count++;
                }
                break;

            case UP:
                if (total_params > 0)
                    cursor_pos = (cursor_pos > 0) ? cursor_pos - 1 : total_params - 1;
                break;

            case DOWN:
                if (total_params > 0)
                    cursor_pos = (cursor_pos < total_params - 1) ? cursor_pos + 1 : 0;
                break;

            case OUT:
                Write_GPS_To_Flash_Group(1);
                need_clear = 1;
                return;

            default:
                break;
        }

        if (cursor_pos / PAGE_SIZE != old_page) {
            ips200_clear();
        }
    }
}

/**
 * @brief  显示姿态数据
 * @param  无
 * @return 无
 */
void show_pose(void)
{
    ips200_clear();
    while (1) {
        KEYRetuen keyenter = key_scan_improvement();

        ips200_show_string(0, CHARHEIGHT*0, "acc_x:");               ips200_show_float(CHARHEIGHT*10, CHARHEIGHT*0, IMU.acc.x, 4, 4);
        ips200_show_string(0, CHARHEIGHT*1, "acc_y:");               ips200_show_float(CHARHEIGHT*10, CHARHEIGHT*1, IMU.acc.y, 4, 4);
        ips200_show_string(0, CHARHEIGHT*2, "acc_z:");               ips200_show_float(CHARHEIGHT*10, CHARHEIGHT*2, IMU.acc.z, 4, 4);
        ips200_show_string(0, CHARHEIGHT*3, "gyro_x:");              ips200_show_float(CHARHEIGHT*10, CHARHEIGHT*3, IMU.gyro.x, 4, 4);
        ips200_show_string(0, CHARHEIGHT*4, "gyro_y:");              ips200_show_float(CHARHEIGHT*10, CHARHEIGHT*4, IMU.gyro.y, 4, 4);
        ips200_show_string(0, CHARHEIGHT*5, "gyro_z:");              ips200_show_float(CHARHEIGHT*10, CHARHEIGHT*5, IMU.gyro.z, 4, 4);
        ips200_show_string(0, CHARHEIGHT*6, "gyro_x_offet:");        ips200_show_float(CHARHEIGHT*10, CHARHEIGHT*6, IMU.gyro_offset.x, 4, 4);
        ips200_show_string(0, CHARHEIGHT*7, "gyro_y_offet:");        ips200_show_float(CHARHEIGHT*10, CHARHEIGHT*7, IMU.gyro_offset.z, 4, 4);
        ips200_show_string(0, CHARHEIGHT*8, "gyro_z_offet:");        ips200_show_float(CHARHEIGHT*10, CHARHEIGHT*8, IMU.gyro_offset.z, 4, 4);
        ips200_show_string(0, CHARHEIGHT*9, "pitch:");               ips200_show_float(CHARHEIGHT*10, CHARHEIGHT*9, IMU.pitch, 3, 4);
        ips200_show_string(0, CHARHEIGHT*10, "roll:");               ips200_show_float(CHARHEIGHT*10, CHARHEIGHT*10, IMU.roll, 3, 4);
        ips200_show_string(0, CHARHEIGHT*11, "yaw:");                ips200_show_float(CHARHEIGHT*10, CHARHEIGHT*11, IMU.yaw, 3, 4);

        if (keyenter == OUT)
        {
            need_clear = 1;
            return;
        }
    }
}

static MenuItem* root_menu = NULL;

/**
 * @brief  强制返回根菜单
 * @param  无
 * @return 无
 */
void return_root(void)
{
    if (root_menu != NULL)
    {
        ips200_clear();
        need_clear = 1;
        menu_force_return = 1;
    }
}

/**
 * @brief  递归释放菜单内存
 * @param  menu: 菜单指针
 * @return 无
 */
static void free_menu(MenuItem* menu)
{
    if (menu == NULL) return;
    if (menu->child != NULL) free_menu(menu->child);
    if (menu->next != NULL) free_menu(menu->next);
    free(menu);
}

/**
 * @brief  菜单主函数
 * @param  无
 * @return 无
 */
void menu(void)
{
    root_menu = create_menu_item("Root", NULL, NULL);

    MenuItem* item1 = create_menu_item("Show_message", NULL, show_message);

    add_child(root_menu, item1);

    menu_in_navigation = 1;
    menu_navigation(root_menu);
    menu_in_navigation = 0;

    free_menu(root_menu);
}
