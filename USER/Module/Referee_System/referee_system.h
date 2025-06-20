//
// Created by 刘嘉俊 on 25-2-25.
//

#ifndef CTRBOARD_H7_ALL_REFEREE_SYSTEM_H
#define CTRBOARD_H7_ALL_REFEREE_SYSTEM_H

#include "main.h"

#define HEADER_SOF 0xA5
#define REFEREE_RX_BUF_SIZE (127+9+136+68+5)   // DMA一次性接收两帧半数据

#define CUSTOMER_CONTROLLER_BUF_SIZE (30+9+39+39+4) // DMA一次性接收三帧数据


/**
*   接收协议数据的帧头数据结构体
*/
typedef struct __attribute__((__packed__))
{
    uint8_t sof;                           /*! 数据帧起始字节，固定值为 0xA5 */
    uint16_t data_length;                  /*! 数据帧中 data 的长度 */
    uint8_t seq;                           /*! 包序号 */
    uint8_t crc8;                          /*! 帧头 CRC8校验 */
} referee_data_header_t;

#define REF_PROTOCOL_HEADER_SIZE            sizeof(referee_data_header_t)  // 协议帧头大小 5字节
#define REF_PROTOCOL_CMD_SIZE               2  // 协议命令字节大小
#define REF_PROTOCOL_CRC16_SIZE             2  // 协议 CRC16 校验大小
#define REF_HEADER_CRC_CMD_SIZE            (REF_PROTOCOL_HEADER_SIZE + REF_PROTOCOL_CRC16_SIZE + REF_PROTOCOL_CMD_SIZE)  // 帧头加命令码的 CRC 长度
#define REF_PROTOCOL_DATA_MAX_SIZE         127  //TODO：交互数据段大小不明意义，以127保险//(127 - REF_HEADER_CRC_CMD_SIZE)   // 最大可传输数据大小，命令码0x301机器人交互数据最大长度为127字节(已经包含帧头帧尾），还有子命令6字节，故数据包大小最终为112字节
#define REF_PROTOCOL_FRAME_MAX_SIZE         (REF_HEADER_CRC_CMD_SIZE+REF_PROTOCOL_DATA_MAX_SIZE)  // 协议帧最大大小

/**
*   接收的协议数据
*/
typedef struct __attribute__((__packed__))
{
    referee_data_header_t* frame_header;        /*! 帧头数据结构体 */
    uint16_t cmd_id;                          /*! 命令码 ID */
    uint8_t* data;                            /*! 接收的数据指针 */
    uint16_t frame_tail;                      /*! frame_tail */
} referee_data_t;

typedef enum
{
    STEP_HEADER_SOF  = 0,
    STEP_DATA_SIZE_LOW  = 1,
    STEP_DATA_SIZE_HIGH = 2,
    STEP_FRAME_SEQ   = 3,
    STEP_HEADER_CRC8 = 4,
    STEP_DATA_CRC16  = 5,
} unpack_step_e;

/**
*   解包函数
*/
typedef struct __attribute__((__packed__))
{
    referee_data_header_t *p_header;
    unpack_step_e  unpack_step;
    uint8_t        protocol_packet[REF_PROTOCOL_FRAME_MAX_SIZE];
    uint16_t       index;
} unpack_data_t;

typedef enum
{
    GAME_STATUS_CMD_ID                = 0x0001,  // 比赛状态数据 长度 11
    GAME_RESULT_CMD_ID                = 0x0002,  // 比赛结果数据 长度 1
    GAME_ROBOT_HP_CMD_ID              = 0x0003,  // 机器人血量数据 长度 32
    FIELD_EVENTS_CMD_ID               = 0x0101,  // 场地事件数据 长度 4
    REFEREE_WARNING_CMD_ID            = 0x0104,  // 裁判警告数据 长度 3
    DART_FIRE_CMD_ID                  = 0x0105,  // 飞镖发射数据 长度 3
    ROBOT_STATUS_CMD_ID               = 0x0201,  // 机器人状态与性能数据 长度 13
    POWER_HEAT_DATA_CMD_ID            = 0x0202,  // 实时底盘缓冲能量和射击热量数据 长度 16
    ROBOT_POS_CMD_ID                  = 0x0203,  // 机器人位置数据 长度 16
    BUFF_MUSK_CMD_ID                  = 0x0204,  // 机器人增益和底盘能量数据 长度 7
    ROBOT_HURT_CMD_ID                 = 0x0206,  // 伤害状态数据 长度 1
    SHOOT_DATA_CMD_ID                 = 0x0207,  // 实时射击数据 长度 7
    BULLET_REMAINING_CMD_ID           = 0x0208,  // 剩余子弹数据 长度 6
    ROBOT_RFID_CMD_ID                 = 0x0209,  // 机器人RFID模块状态数据 长度 4
    DART_DIRECTIONS_CMD_ID            = 0x020A,  // 飞镖选手端指令数据 长度 6
    ROBOT_LOCATION_CMD_ID             = 0x020B,  // 地面机器人位置数据 长度 40
    RADAR_PROGRESS_CMD_ID             = 0x020C,  // 雷达标记进度数据 长度 1
    SENTRY_AUTONOMY__CMD_ID           = 0x020D,  // 哨兵自主决策信息同步数据 长度 6
    RADAR_AUTONOMY_CMD_ID             = 0x020E,  // 雷达自主决策信息同步数据 长度 1
    STUDENT_INTERACTIVE_DATA_CMD_ID   = 0x0301,  // 机器人交互数据，发送方触发发送 长度 127
    ARM_DATA_FROM_CONTROLLER_CMD_ID_2  = 0x0302,  // 自定义控制器与机器人交互数据，发送方触发发送 长度 30
    PLAYER_MINIMAP_CMD_ID             = 0x0303,  // 选手端小地图交互数据 长度 15
    KEYBOARD_MOUSE_CMD_ID             = 0x0304,  // 键鼠遥控数据 长度 12
    RADAR_MINIMAP_CMD_ID              = 0x0305,  // 选手端小地图接收雷达数据 长度 24
    CUSTOMER_CONTROLLER_PLAYER_CMD_ID = 0x0306,  // 自定义控制器与选手端交互数据 长度 8
    PLAYER_MINIMAP_SENTRY_CMD_ID      = 0x0307,  // 选手端小地图接收哨兵数据 长度 103
    PLAYER_MINIMAP_ROBOT_CMD_ID       = 0x0308,  // 选手端小地图接收机器人数据 长度 34
    ARM_DATA_FROM_CONTROLLER_CMD_ID_9   = 0x0309,  // 自定义控制器接收机器人数据 长度 30
} referee_cmd_id_t;

/**
*   比赛状态数据         对应的命令码ID为：0x0001
*/
typedef struct __attribute__((__packed__))
{
    uint8_t game_type : 4;                      /*! 比赛的类型 */
    uint8_t game_progress : 4;                  /*! 当前比赛阶段 */
    uint16_t stage_remain_time;                 /*! 当前阶段剩余时间 */
    uint64_t SyncTimeStamp;                     /*! 机器人接收到该指令的精确 Unix 时间 */
} game_status_t;

/**
*   比赛结果数据         对应的命令码ID为：0x0002
*/
typedef struct __attribute__((__packed__))
{
    uint8_t winner;                             /*! 0 平局 1 红方胜利 2 蓝方胜利 */
} game_result_t;

/**
*   机器人血量数据         对应的命令码ID为：0x0003
*/
typedef struct __attribute__((__packed__))
{
    uint16_t red_1_robot_HP;            /*! 红 1 英雄机器人血量。若该机器人未上场或者被罚下，则血量为 0 */
    uint16_t red_2_robot_HP;            /*! 红 2 工程机器人血量 */
    uint16_t red_3_robot_HP;            /*! 红 3 步兵机器人血量 */
    uint16_t red_4_robot_HP;            /*! 红 4 步兵机器人血量 */
    uint16_t reserved_1;                  /*! 保留位 1 */
    uint16_t red_7_robot_HP;            /*! 红 7 哨兵机器人血量 */
    uint16_t red_outpost_HP;            /*! 红方前哨站血量 */
    uint16_t red_base_HP;               /*! 红方基地血量 */
    uint16_t blue_1_robot_HP;           /*! 蓝 1 英雄机器人血量 */
    uint16_t blue_2_robot_HP;           /*! 蓝 2 工程机器人血量 */
    uint16_t blue_3_robot_HP;           /*! 蓝 3 步兵机器人血量 */
    uint16_t blue_4_robot_HP;           /*! 蓝 4 步兵机器人血量 */
    uint16_t reserved_2;                  /*! 保留位 2 */
    uint16_t blue_7_robot_HP;           /*! 蓝 7 哨兵机器人血量 */
    uint16_t blue_outpost_HP;           /*! 蓝方前哨站血量 */
    uint16_t blue_base_HP;              /*! 蓝方基地血量 */
} game_robot_HP_t;

/**
    0：未占领/未激活         对应的命令码ID为：0x0101
    1：已占领/已激活
    bit 0-2：
    ? bit 0：己方与兑换区不重叠的补给区占领状态，1 为已占领
    ? bit 1：己方与兑换区重叠的补给区占领状态，1 为已占领
    ? bit 2：己方补给区的占领状态，1 为已占领（仅 RMUL 适用）
    bit 3-5：己方能量机关状态
    ? bit 3：己方小能量机关的激活状态，1 为已激活
    ? bit 4：己方大能量机关的激活状态，1 为已激活
    ? bit 5-6：己方中央高地的占领状态，1 为被己方占领，2 为被对方占领
    ? bit 7-8：己方梯形高地的占领状态，1 为已占领
    ? bit 9-17：对方飞镖最后一次击中己方前哨站或基地的时间（0-420，开
    局默认为 0）
    ? bit 18-20：对方飞镖最后一次击中己方前哨站或基地的具体目标，开局
    默认为 0，1 为击中前哨站，2 为击中基地固定目标，3 为击中基地随机
    固定目标，4 为击中基地随机移动目标
    ? bit 21-22：中心增益点的占领状态，0 为未被占领，1 为被己方占领，2
    为被对方占领，3 为被双方占领。（仅 RMUL 适用）
    ? bit 23-31：保留位
 */
typedef struct __attribute__((__packed__))
{
    uint32_t event_data;
} event_data_t;

/**
 *   裁判警告信息         对应的命令码ID为：0x0104
 *   发送频率：己方警告发生后发送
 *   己方最后一次受到判罚的等级：
 *          1：双方黄牌
 *          2：黄牌
 *          3：红牌
 *          4：判负
 *    犯规机器人 ID：
 *          己方最后一次受到判罚的违规机器人 ID。（如红 1 机器人 ID 为 1，蓝1 机器人 ID 为 101）
 *          判负和双方黄牌时，该值为 0
 *    违规次数：
 *          己方最后一次受到判罚的违规机器人对应判罚等级的违规次数。（开局默认为 0。）
*/
typedef struct __attribute__((__packed__))
{
    uint8_t level;                              /*! 警告等级 */
    uint8_t offending_robot_id;                      /*! 犯规机器人 ID */
    uint8_t count;                                /*! 违规次数 */
} referee_warning_t;

/**
 *   飞镖发射口倒计时         对应的命令码ID为：0x0105
 *   发送频率：1Hz 周期发送，发送范围：己方机器人
 *   剩余时间：
 *         己方飞镖发射剩余时间，单位：秒
 *   飞镖目标：
 *   bit 0-2：
 *      最近一次己方飞镖击中的目标，开局默认为 0，1 为击中前哨站，2 为击中
 *      基地固定目标，3 为击中基地随机固定目标，4 为击中基地随机移动目标
 *   bit 3-5：
 *      对方最近被击中的目标累计被击中计次数，开局默认为 0，至多为 4
 *   bit 6-7：
 *      飞镖此时选定的击打目标，开局默认或未选定/选定前哨站时为 0，选中基
 *      地固定目标为 1，选中基地随机固定目标为 2，选中基地随机移动目标为 3
 *   bit 8-15：保留
*/
typedef struct __attribute__((__packed__))
{
    uint8_t dart_remaining_time;            /*! 15s 倒计时 */
    uint16_t dart_info;
} dart_info_t;


/**
 *   比赛机器人状态         对应的命令码ID为：0x0201
 *   发送频率：10Hz
 *    本机器人 ID：
 *            1：红方英雄机器人；
 *            2：红方工程机器人；
 *            3/4/5：红方步兵机器人；
 *            6：红方空中机器人；
 *            7：红方哨兵机器人；
 *            8：红方飞镖机器人；
 *            9：红方雷达站；
 *            101：蓝方英雄机器人；
 *            102：蓝方工程机器人；
 *            103/104/105：蓝方步兵机器人；
 *            106：蓝方空中机器人；
 *            107：蓝方哨兵机器人；
 *            108：蓝方飞镖机器人；
 *            109：蓝方雷达站。
 *   主控电源输出情况：
 *            gimbal 口输出： 1 为有 24V 输出，0 为无 24v 输出；
 *            chassis 口输出：1 为有 24V 输出，0 为无 24v 输出；
 *            shooter 口输出：1 为有 24V 输出，0 为无 24v 输出；
*/
typedef struct __attribute__((__packed__))
{
    uint8_t robot_id;                               /*! 本机器人 ID */
    uint8_t robot_level;                            /*! 机器人等级 */
    uint16_t current_HP;                             /*! 机器人剩余血量 */
    uint16_t maximum_HP;                                /*! 机器人上限血量 */
    uint16_t shooter_barrel_cooling_value;             /*!机器人枪口热量每秒冷却值*/
    uint16_t shooter_barrel_heat_limit;                /*!机器人枪口热量上限*/
    uint16_t chassis_power_limit;                   /*! 机器人底盘功率限制上限 */
    uint8_t power_management_gimbal_output : 1;          /*! 主控电源输出情况：gimbal 口输出 */
    uint8_t power_management_chassis_output : 1;         /*! 主控电源输出情况：chassis 口输出 */
    uint8_t power_management_shooter_output : 1;        /*! 主控电源输出情况：shooter 口输出 */
} robot_status_t;

/**
 *   实时功率热量数据   对应命令码ID为：0x0202
 *   发送频率：50Hz
 *   保留位
 *   保留位
 *   保留位
 *   底盘功率缓冲 单位 J 焦耳 备注：飞坡根据规则增加至 250J
 *   1 号 17mm 枪口热量
 *   2 号 17mm 枪口热量
 *   42mm 枪口热量
 */
typedef struct __attribute__((__packed__))
{
    uint16_t reserved_1;                          /*!底盘输出电压*/
    uint16_t reserved_2;                          /*!底盘输出电流*/
    float reserved_3;;                               /*!底盘输出功率*/
    uint16_t buffer_energy;                            /*!底盘功率缓存*/
    uint16_t shooter_17mm_1_barrel_heat;               /*!机器人1 号 17mm 枪口热量*/
    uint16_t shooter_17mm_2_barrel_heat;               /*!机器人2 号 17mm 枪口热量*/
    uint16_t shooter_42mm_barrel_heat;                 /*!机器人42mm 枪口热量*/
} power_heat_data_t;

/**
 * 机器人位置    对应命令码ID为：0x0203
 * 发送频率：10Hz
 * 位置 x 坐标，单位 m
 * 位置 y 坐标，单位 m
 * 位置枪口，单位度
 */
typedef  struct __attribute__((__packed__))
{
    float x;                               /*!位置 x 坐标*/
    float y;                               /*!位置 y 坐标*/
    float angle;                           /*!本机器人测速模块的朝向，单位：度。正北为 0 度*/
} robot_pos_t;

/**
* 机器人增益  对应命令码ID为：0x0204
* 发送频率：1Hz
* 机器人血量补血状态
* 枪口热量冷却加速
* 机器人防御加成
* 机器人攻击加成
* 其他 bit 保留
*/
typedef  struct __attribute__((__packed__))
{
    uint8_t recovery_buff;               /*!机器人回血增益（百分比，值为 10 表示每秒恢复血量上限的 10%）*/
    uint8_t cooling_buff;                /*!机器人枪口冷却倍率（直接值，值为 5 表示 5 倍冷却）*/
    uint8_t defence_buff;                /*!机器人防御增益（百分比，值为 50 表示 50%防御增益）*/
    uint8_t vulnerability_buff;          /*!机器人负防御增益（百分比，值为 30 表示-30%防御增益）*/
    uint16_t attack_buff;                /*!机器人攻击增益（百分比，值为 50 表示 50%攻击增益）*/
    uint8_t remaining_energy;            /*!机器人剩余能量（百分比，值为 50 表示剩余能量为 50%）*/
/*
* bit 0-4：机器人剩余能量值反馈，以 16 进制标识机器人剩余能量值比例，仅在机器人剩余能量小于 50%时反馈，其余默认反馈 0x32。bit 0：在剩余能量≥50%时为 1，其余情况为 0
* ? bit 1：在剩余能量≥30%时为 1，其余情况为 0
* ? bit 2：在剩余能量≥15%时为 1，其余情况为 0
* ? bit 3：在剩余能量≥5%时为 1，其余情况为 0
* ? bit 4：在剩余能量≥1%时为 1，其余情况为 0
*/
} buff_t;

/**
*  伤害状态  对应命令码ID为：0x0206
*  发送频率：伤害发生后发送
*  bit 0-3：当扣血原因为装甲模块被弹丸攻击、受撞击、离线或测速模块离线时，
*  该 4 bit 组成的数值为装甲模块或测速模块的 ID 编号；
*  当其他原因导致扣血时，该数值为 0
*  bit 4-7：血量变化类型
*  0x0 装甲模块被弹丸攻击导致扣血
*  0x1 裁判系统重要模块离线导致扣血
*  0x2 射击初速度超限导致扣血
*  0x3 射击热量超限导致扣血
*  0x4 底盘功率超限导致扣血
*  0x5 装甲模块受到撞击导致扣血
*/
typedef struct __attribute__((__packed__))
{
    uint8_t armor_id : 4;                    /*!装甲ID */
    uint8_t HP_deduction_reason : 4;         /*!血量变化原因*/
} hurt_data_t;

/**
*  实时射击信息   对应命令码ID为：0x0207
*  发送频率：射击后发送
*  子弹类型: 1：17mm 弹丸 2：42mm 弹丸
*  发射机构 ID：
*  1：1 号 17mm 发射机构
*  2：2 号 17mm 发射机构
*  3：42mm 发射机构
* 子弹射频 单位 Hz
* 子弹射速 单位 m/s
*/
typedef struct __attribute__((__packed__))
{
    uint8_t bullet_type;                 /*!子弹类型*/
    uint8_t shooter_number;              /*!发射枪口ID*/
    uint8_t launching_frequency;         /*!子弹射频*/
    float initial_speed;                 /*!子弹射速*/
} shoot_data_t;

/**
*  子弹剩余发射数    对应命令码ID为：0x0208
*  发送频率：10Hz 周期发送，所有机器人发送
*   17mm 子弹剩余发射数量含义说明             联盟赛                              对抗赛
*    步兵机器人                   全队步兵与英雄剩余可发射 17mm 弹丸总量          全队 17mm 弹丸剩余可兑换数量
*    英雄机器人                   全队步兵与英雄剩余可发射 17mm 弹丸总量          全队 17mm 弹丸剩余可兑换数量
*    空中机器人、哨兵机器人         该机器人剩余可发射 17mm 弹丸总量               该机器人剩余可发射 17mm 弹丸总量
*
*   17mm 子弹剩余发射数目
*   42mm 子弹剩余发射数目
*   剩余金币数量
*/
typedef struct __attribute__((__packed__))
{
    uint16_t projectile_allowance_17mm;                 /*!17mm弹头剩余数量*/
    uint16_t projectile_allowance_42mm;                 /*!42mm弹头剩余数量*/
    uint16_t remaining_gold_coin;                       /*!金币剩余数量*/
} projectile_allowance_t;

/**
*  机器人 RFID 状态  对应命令码ID为：0x0209
*  发送频率：1Hz    发送范围：己方装有 RFID模块的机器人
*  bit 位值为 1/0 的含义：是否已检测到该增益点 RFID 卡
? bit 0：己方基地增益点
? bit 1：己方中央高地增益点
? bit 2：对方中央高地增益点
? bit 3：己方梯形高地增益点
? bit 4：对方梯形高地增益点
? bit 5：己方地形跨越增益点（飞坡）（靠近己方一侧飞坡前）
? bit 6：己方地形跨越增益点（飞坡）（靠近己方一侧飞坡后）
? bit 7：对方地形跨越增益点（飞坡）（靠近对方一侧飞坡前）
? bit 8：对方地形跨越增益点（飞坡）（靠近对方一侧飞坡后）
? bit 9：己方地形跨越增益点（中央高地下方）
? bit 10：己方地形跨越增益点（中央高地上方）
? bit 11：对方地形跨越增益点（中央高地下方）
? bit 12：对方地形跨越增益点（中央高地上方）
? bit 13：己方地形跨越增益点（公路下方）
? bit 14：己方地形跨越增益点（公路上方）
? bit 15：对方地形跨越增益点（公路下方）
? bit 16：对方地形跨越增益点（公路上方）
? bit 17：己方堡垒增益点
? bit 18：己方前哨站增益点
? bit 19：己方与兑换区不重叠的补给区/RMUL 补给区
? bit 20：己方与兑换区重叠的补给区
? bit 21：己方大资源岛增益点
? bit 22：对方大资源岛增益点
? bit 23：中心增益点（仅 RMUL 适用）
? bit 24-31：保留
注：所有 RFID 卡仅在赛内生效。在赛外，即使检测到对应的 RFID 卡，对
应值也为 0。
* 注：基地增益点、高地增益点、飞坡增益点、前哨站增益点、资源岛增益点、
* 补血点、兑换区、中心增益点（仅适用于 RMUL）和哨兵巡逻区的 RFID 卡
* 仅在赛内生效。在赛外，即使检测到对应的 RFID 卡，对应值也为 0。
*/
typedef struct __attribute__((__packed__))
{
    uint32_t rfid_status;                        /*!机器人RFID状态 */
} rfid_status_t;

/**
* 飞镖机器人客户端指令数据   对应命令码ID为：0x020A
* 发送频率：10Hz 发送范围：己方飞镖机器人
*  当前飞镖发射站的状态
*  1：关闭；
*  2：正在开启或者关闭中
*  0：已经开启
*  保留
*  切换击打目标时的比赛剩余时间，单位：秒，无/未切换动作，默认为 0。
*  最后一次操作手确定发射指令时的比赛剩余时间，单位秒, 初始值为 0。
*/
typedef struct __attribute__((__packed__))
{
    uint8_t dart_launch_opening_status;             /*!飞镖发射站状态*/
    uint8_t reserved;                               /*!保留*/
    uint16_t target_change_time;                    /*!切换打击目标时的比赛剩余时间*/
    uint16_t latest_launch_cmd_time;               /*!最后一次操作手确定发射指令时的比赛剩余时间*/
} dart_client_cmd_t;

/**
*  地面机器人位置数据   对应命令码ID为：0x020B
*  发送频率：1Hz 发送范围：己方哨兵机器人
*  单位：m
*
*/
typedef struct __attribute__((__packed__))
{
    float hero_x;       /*!己方英雄机器人位置 x 轴坐标*/
    float hero_y;       /*!己方英雄机器人位置 y 轴坐标*/
    float engineer_x;   /*!己方工程机器人位置 x 轴坐标*/
    float engineer_y;   /*!己方工程机器人位置 y 轴坐标*/
    float standard_3_x; /*!己方 3 号步兵机器人位置 x 轴坐标*/
    float standard_3_y; /*!己方 3 号步兵机器人位置 y 轴坐标*/
    float standard_4_x; /*!己方 4 号步兵机器人位置 x 轴坐标*/
    float standard_4_y; /*!己方 4 号步兵机器人位置 y 轴坐标*/
    float reserved_1; /*!己方 5 号步兵机器人位置 x 轴坐标*/
    float reserved_2; /*!己方 5 号步兵机器人位置 y 轴坐标*/
} ground_robot_position_t;

/**
*  雷达标记进度数据  对应命令码ID为：0x020C
*  发送频率：1Hz 发送范围：己方雷达机器人
*  在对应机器人被标记进度≥100 时发送 1，被标记进度<100 时发送 0。
? bit 0：对方 1 号英雄机器人易伤情况
? bit 1：对方 2 号工程机器人易伤情况
? bit 2：对方 3 号步兵机器人易伤情况
? bit 3：对方 4 号步兵机器人易伤情况
? bit 4：对方哨兵机器人易伤情况
*/

typedef struct __attribute__((__packed__))
{
    uint8_t mark_progress;
} radar_mark_data_t;

/**
*  哨兵自主决策信息同步  对应命令码ID为：0x020D
*  发送频率：1Hz 发送范围：己方哨兵机器人
* bit 0-10：
*     除远程兑换外，哨兵成功兑换的发弹量，开局为 0，在哨兵成功兑
*     换一定发弹量后，该值将变为哨兵成功兑换的发弹量值。
* bit 11-14：
*     哨兵成功远程兑换发弹量的次数，开局为 0，在哨兵成功远程兑
*     换发弹量后，该值将变为哨兵成功远程兑换发弹量的次数。
* bit 15-18：
*     哨兵成功远程兑换血量的次数，开局为 0，在哨兵成功远程兑换
*     血量后，该值将变为哨兵成功远程兑换血量的次数。
* bit 19：哨兵机器人当前是否可以确认免费复活，可以确认免费复活时值为1，否则为 0。
* bit 20：哨兵机器人当前是否可以兑换立即复活，可以兑换立即复活时值为1，否则为 0。
* bit 21-30：哨兵机器人当前若兑换立即复活需要花费的金币数。
* bit 31：保留。

bit 0：哨兵当前是否处于脱战状态，处于脱战状态时为 1，否则为 0。
bit 1-11：队伍 17mm 允许发弹量的剩余可兑换数。
bit 12-15：保留。
*/

typedef struct __attribute__((__packed__))
{
    uint32_t sentry_info;
    uint16_t sentry_info_2;
} sentry_info_t;

/**
*  雷达自主决策信息同步  对应命令码ID为：0x020E
*  发送频率：1Hz 发送范围：己方雷达机器人
* bit 0-1：
*     雷达是否拥有触发双倍易伤的机会，开局为 0，
*     数值为雷达拥有触发双倍易伤的机会，至多为 2
* bit 2：
*     对方是否正在被触发双倍易伤
*      0：对方未被触发双倍易伤
*      1：对方正在被触发双倍易伤
* bit 3-7：保留
*/
typedef struct __attribute__((__packed__))
{
    uint8_t radar_info;
} radar_info_t;

/**
 *  交互数据接收信息   对应命令码ID为：0x0301
 * 子内容 ID：
 *      需为开放的子内容 ID
 * 发送者的 ID:
 *      需要校验发送者的 ID 正确性，例如红 1 发送给红 5，此项需要校验红 1
 * 接收者的 ID:
 *      仅限己方通信
 *      需为规则允许的多机通讯接收者
 *      若接收者为选手端，则仅可发送至发送者对应的选手端
 *      ID 编号详见附录
 * 内容数据段:
 *  最大为 113
*/
/*
机器人交互数据通过常规链路发送，其数据段包含一个统一的数据段头结构。数据段头结构包括内容 ID、
发送者和接收者的 ID、内容数据段。机器人交互数据包的总长不超过 127 个字节，减去 frame_header、
cmd_id 和 frame_tail 的 9 个字节以及数据段头结构的 6 个字节，故机器人交互数据的内容数据段最大
为 112 个字节。
每 1000 毫秒，英雄、工程、步兵、空中机器人、飞镖能够接收数据的上限为 3720 字节，雷达和哨兵机器
人能够接收数据的上限为 5120 字节。
由于存在多个内容 ID，但整个 cmd_id 上行频率最大为 30Hz，请合理安排带宽。
*/
typedef struct __attribute__((__packed__))
{
    uint16_t data_cmd_id;                           /*! 子内容 ID*/
    uint16_t sender_id;                             /*! 发送者 ID */
    uint16_t receiver_id;                           /*! 接收者 ID */
    uint8_t user_data[112];                              /*! 内容数据段 内容最大为112字节*/
} robot_interaction_data_t;
//TODO:机器人交互数据子内容的完善
//TODO:未添加自主决策指令
/** 交互数据包绘图子命令 内容ID */
typedef enum
{
    //0x200-0x02ff 	队伍自定义命令 格式  INTERACT_ID_XXXX
    DELETE_GRAPHIC_ID = 0x0100,  //客户端删除图形
    DRAW_ONE_GRAPHIC_ID = 0x0101,  //客户端绘制一个图形
    DRAW_TWO_GRAPHIC_ID = 0x0102,  //客户端绘制二个图形
    DRAW_FIVE_GRAPHIC_ID = 0x0103,  //客户端绘制五个图形
    DRAW_SEVEN_GRAPHIC_ID = 0x0104,  //客户端绘制七个图形
    DRAW_CHAR_GRAPHIC_ID = 0x0110,  //客户端绘制字符图形

    ID_delete_graphic 			= 0x0100,  //客户端删除图形
    ID_draw_one_graphic 		= 0x0101,  //客户端绘制一个图形
    ID_draw_two_graphic 		= 0x0102,  //客户端绘制二个图形
    ID_draw_five_graphic 	  = 0x0103,  //客户端绘制五个图形
    ID_draw_seven_graphic 	= 0x0104,  //客户端绘制七个图形
    ID_draw_char_graphic 	  = 0x0110,  //客户端绘制字符图形
} ui_cmd_id_e;

/* 数据段长度 */
enum
{
    DELETE_GRAPHIC_LEN = 8,
    DRAW_ONE_GRAPHIC_LEN = 21,
    DRAW_TWO_GRAPHIC_LEN = 36,
    DRAW_FIVE_GRAPHIC_LEN = 81,
    DRAW_SEVEN_GRAPHIC_LEN = 111,
    DRAW_CHAR_GRAPHIC_LEN = 51,

    LEN_ID_delete_graphic     = 8,  //6+2
    LEN_ID_draw_one_graphic   = 21, //6+15
    LEN_ID_draw_two_graphic   = 36, //6+15*2
    LEN_ID_draw_five_graphic  = 81, //6+15*5
    LEN_ID_draw_seven_graphic = 111,//6+15*7
    LEN_ID_draw_char_graphic  = 51, //6+15+30（字符串内容）
};

/* 操作类型 */
typedef enum
{
    NONE   = 0,/*空操作*/
    ADD    = 1,/*增加图层*/
    MODIFY = 2,/*修改图层*/
    DELETE = 3,/*删if除图层*/
} operate_tpye_e;

/* 颜色 */
typedef enum
{
    RED_BLUE  = 0,  //红蓝主色
    YELLOW    = 1,  //黄
    GREEN     = 2,  //绿
    ORANGE    = 3,  //橙
    FUCHSIA   = 4,	//紫红色
    PINK      = 5,   //粉
    CYAN_BLUE = 6,	//青色
    BLACK     = 7,
    WHITE     = 8
}graphic_color_e;

typedef struct __attribute__((__packed__))
{
    uint8_t operate_tpye;
    uint8_t layer;
} ext_client_custom_graphic_delete_t;

typedef struct __attribute__((__packed__))
{
    uint8_t delete_type;
    uint8_t layer;
} interaction_layer_delete_t;

/* 图形数据 */
typedef struct __attribute__((__packed__))
{
    uint8_t graphic_name[3];
    uint32_t operate_tpye:3;       /* 0:空操作;1:增加;2:修改;3:删除	*/
    uint32_t graphic_tpye:3;        /*	0:直线;1:矩形;2:正圆;3:椭圆;4:圆弧;5:浮点数;6:整形;7:字符 */
    uint32_t layer:4;
    uint32_t color:4;
    uint32_t start_angle:9;
    uint32_t end_angle:9;
    uint32_t width:10;
    uint32_t start_x:11;
    uint32_t start_y:11;
    uint32_t radius:10;
    uint32_t end_x:11;
    uint32_t end_y:11;
} graphic_data_struct_t;

typedef struct __attribute__((__packed__))
{
    uint8_t figure_name[3];
    uint32_t operate_tpye:3;
    uint32_t figure_tpye:3;
    uint32_t layer:4;
    uint32_t color:4;
    uint32_t details_a:9;
    uint32_t details_b:9;
    uint32_t width:10;
    uint32_t start_x:11;
    uint32_t start_y:11;
    uint32_t details_c:10;
    uint32_t details_d:11;
    uint32_t details_e:11;
} interaction_figure_t;

/* 客户端绘制一个图形数据段 */
typedef struct __attribute__((__packed__))
{
    graphic_data_struct_t grapic_data_struct;
} ext_client_custom_graphic_single_t;

/* 客户端绘制二个图形数据段 */
typedef struct __attribute__((__packed__))
{
    graphic_data_struct_t grapic_data_struct[2];
} ext_client_custom_graphic_double_t;

typedef struct __attribute__((__packed__))
{
    interaction_figure_t interaction_figure[2];
} interaction_figure_2_t;

/* 客户端绘制五个图形数据段 */
typedef struct __attribute__((__packed__))
{
    graphic_data_struct_t grapic_data_struct[5];
} ext_client_custom_graphic_five_t;

typedef struct __attribute__((__packed__))
{
    interaction_figure_t interaction_figure[5];
} interaction_figure_3_t;

/* 客户端绘制七个图形数据段 */
typedef struct __attribute__((__packed__))
{
    graphic_data_struct_t grapic_data_struct[7];
} ext_client_custom_graphic_seven_t;

typedef struct __attribute__((__packed__))
{
    interaction_figure_t interaction_figure[7];
}interaction_figure_4_t;

/* 客户端绘制字符数据段 */
typedef struct __attribute__((__packed__))
{
    graphic_data_struct_t grapic_data_struct;
    char data[30];
} ext_client_custom_character_t;

//typedef struct __attribute__((__packed__))
//{
//    graphic_data_struct_t grapic_data_struct;
//    uint8_t data[30];
//} ext_client_custom_character_t;

///* 机器人间交互数据专用帧结构 */
//typedef struct __attribute__((__packed__))
//{
//    referee_data_header_t frame_header;  //帧头
//    uint16_t cmd_id;  //命令码 ID
//    ext_student_interactive_header_data_t data_header;  //数据段头结构
//    uint16_t frame_tail;  //帧尾
//} frame_t;
//
//typedef struct __attribute__((__packed__))
//{
//    referee_data_header_t frame_header;  //帧头
//    uint16_t cmd_id;  //命令码 ID
//    ext_student_interactive_header_data_t data_header;  //数据段头结构
//    uint16_t frame_tail;  //帧尾
//} interaction_data_frame_t;

/* 客户端信息 */
typedef struct __attribute__((__packed__))
{
    uint8_t robot_id;
    uint16_t client_id;
} client_info_t;




/**
*  云台手可通过选手端大地图向机器人发送固定数据。对应命令码为 0x0303
*  触发时发送，两次发送间隔不得低于 0.5 秒。
* Byte 0-3：
*     目标位置 x 轴坐标，单位 m，当发送目标机器人 ID 时，该值为 0
* Byte 4-7：
*     目标位置 y 轴坐标，单位 m，当发送目标机器人 ID 时，该值为 0
* Byte 8：
*     云台手按下的键盘按键通用键值，无按键按下，则为 0
* Byte 9：
*     对方机器人 ID，当发送坐标数据时，该值为 0
* Byte 10-11：
*     信息来源 ID，信息来源的 ID，ID 对应关系详见附录
*/
typedef struct __attribute__((__packed__))
{
    float target_position_x;
    float target_position_y;
    uint8_t cmd_keyboard;
    uint8_t target_robot_id;
    uint8_t cmd_source;
} map_command_t;

/**
*  选手端小地图接收的雷达发送的对方机器人的坐标数据。 对应命令码ID为：0x0305
 *  当 x、y 超出边界时显示在对应边缘处，当 x、y 均为 0 时，视为未发送此机器人坐标。
 *  英雄机器人 x 位置坐标，单位：cm
 *  英雄机器人 y 位置坐标，单位：cm
 *  工程机器人 x 位置坐标，单位：cm
 *  工程机器人 y 位置坐标，单位：cm
 *  步兵 3 号机器人 x 位置坐标，单位：cm
 *  步兵 3 号机器人 y 位置坐标，单位：cm
 *  步兵 4 号机器人 x 位置坐标，单位：cm
 *  步兵 4 号机器人 y 位置坐标，单位：cm
 *  步兵 5 号机器人 x 位置坐标，单位：cm
 *  步兵 5 号机器人 y 位置坐标，单位：cm
 *  哨兵机器人 x 位置坐标，单位：cm
 *  哨兵机器人 y 位置坐标，单位：cm
*/
typedef struct __attribute__((__packed__))
{
    uint16_t hero_position_x;
    uint16_t hero_position_y;
    uint16_t engineer_position_x;
    uint16_t engineer_position_y;
    uint16_t infantry_3_position_x;
    uint16_t infantry_3_position_y;
    uint16_t infantry_4_position_x;
    uint16_t infantry_4_position_y;
    uint16_t infantry_5_position_x;
    uint16_t infantry_5_position_y;
    uint16_t sentry_position_x;
    uint16_t sentry_position_y;
} map_robot_data_t;

/**
*  选手端小地图接收的哨兵机器人或选择半自动控制方式的机器人通过常规链路发送的对方机器人的坐标数据。 对应命令码ID为：0x0307
*     小地图左下角为坐标原点，水平向右为 X 轴正
*     方向，竖直向上为 Y 轴正方向。显示位置将按
*     照场地尺寸与小地图尺寸等比缩放，超出边界
*     的位置将在边界处显示
* Byte 0：
*     1：到目标点攻击
*     2：到目标点防守
*     3：移动到目标点
* Byte 1-2：
*     路径起点 x 轴坐标，单位：dm
* Byte 3-4：
*     路径起点 y 轴坐标，单位：dm
* Byte 5-53：
*     路径点 x 轴增量数组，单位：dm
* Byte 54-102：
*     路径点 y 轴增量数组，单位：dm
* Byte 103-104：
*     发送者 ID
*/
typedef struct __attribute__((__packed__))
{
    uint8_t intention;
    uint16_t start_position_x;
    uint16_t start_position_y;
    int8_t delta_x[49];
    int8_t delta_y[49];
    uint16_t sender_id;
} map_data_t;

/**
*  己方机器人可通过常规链路向己方任意选手端发送自定义的消息，该消息会在己方选手端特定位置显示。 对应命令码ID为：0x0308
* Byte 0-1：
*     发送者 ID
* Byte 2-3：
*     接收者 ID
* Byte 4-33：
*     字符
*/
typedef struct __attribute__((__packed__))
{
    uint16_t sender_id;
    uint16_t receiver_id;
    uint8_t user_data[30];
} custom_info_t;

/**
*  操作手可使用自定义控制器通过图传链路向对应的机器人发送数据  对应命令码ID为：0x0302
*/
typedef struct __attribute__((__packed__))
{
    uint8_t data[30];
} custom_robot_data_t;

/**
*  操作手可使用自定义控制器通过图传链路向对应的机器人发送数据  对应命令码ID为：0x0309
*/
typedef struct __attribute__((__packed__))
{
    uint8_t data[30];
} robot_custom_data_t;

/**
*  通过遥控器发送的键鼠遥控数据将同步通过图传链路发送给对应机器人。  对应命令码ID为：0x0304
* Byte 0-1：
*     鼠标 x 轴移动速度，负值标识向左移动
* Byte 2-3：
*     鼠标 y 轴移动速度，负值标识向下移动
* Byte 4-5：
*     鼠标滚轮移动速度，负值标识向后滚动
* Byte 6：
*     鼠标左键是否按下：0 为未按下；1 为按下
* Byte 7：
*     鼠标右键是否按下：0 为未按下，1 为按下
* Byte 8-9：
*     键盘按键信息，每个 bit 对应一个按键，0 为未按下，1 为按下：
*     ? bit 0：W 键
*     ? bit 1：S 键
*     ? bit 2：A 键
*     ? bit 3：D 键
*     ? bit 4：Shift 键
*     ? bit 5：Ctrl 键
*     ? bit 6：Q 键
*     ? bit 7：E 键
*     ? bit 8：R 键
*     ? bit 9：F 键
*     ? bit 10：G 键
*     ? bit 11：Z 键
*     ? bit 12：X 键
*     ? bit 13：C 键
*     ? bit 14：V 键
*     ? bit 15：B 键
* Byte 10-11：
*     保留位
*/
typedef struct __attribute__((__packed__))
{
    int16_t mouse_x;
    int16_t mouse_y;
    int16_t mouse_z;
    int8_t left_button_down;
    int8_t right_button_down;
    uint16_t keyboard_value;
    uint16_t reserved;
} remote_control_t;

/**
*  非链路数据，操作手可使用自定义控制器模拟键鼠操作选手端。 对应命令码ID为：0x0306
* Byte 0-1：
*     键盘键值：
*     ? bit 0-7：按键 1 键值
*     ? bit 8-15：按键 2 键值
* Byte 2-3：
*     ? bit 0-11：鼠标 X 轴像素位置
*     ? bit 12-15：鼠标左键状态
* Byte 4-5：
*     ? bit 0-11：鼠标 Y 轴像素位置
*     ? bit 12-15：鼠标右键状态
* Byte 6-7：
*     保留位
*/
typedef struct __attribute__((__packed__))
{
    uint16_t key_value;
    uint16_t x_position:12;
    uint16_t mouse_left:4;
    uint16_t y_position:12;
    uint16_t mouse_right:4;
    uint16_t reserved;
} custom_client_data_t;

/**
 * @brief 裁判系统接收初始化
 */
void referee_system_init();
/**
 * @brief 裁判系统接收数据帧解包
 */
void referee_data_unpack(uint8_t *data, uint16_t len);
/**
 * @brief 裁判系统数据更新并保存
 */
void referee_data_save(uint8_t* referee_data_frame);

/* ------------------------------ referee反馈状态数据 ------------------------------ */
/**
 * @brief 上位机反馈状态数据,由referee发布
 */
struct referee_fdb_msg
{
    robot_status_t robot_status;
    power_heat_data_t power_heat_data;
    remote_control_t remote_control;
    custom_robot_data_t custom_robot_data;
};

#endif //CTRBOARD_H7_ALL_REFEREE_SYSTEM_H
