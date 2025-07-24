#pragma once

#include <stdint.h>
#include <math.h>
#include <stdbool.h>
#include <string.h>

/* 消息缓冲区大小 */
#define PD_MSG_BUFFER_SIZE 16 // 消息缓冲区大小
#define PD_MSG_MAX_LEN     34 // 单条消息最大长度

/* PD 消息类型描述结构体 */
typedef struct {
    uint8_t type;     // 消息类型编号
    const char *name; // 消息类型名称
} pd_msg_type_desc_t;

/* PD 消息头部结构体 */
typedef struct {
    uint16_t MessageType : 5;           // 消息类型
    uint16_t PortDataRole : 1;          // 数据角色
    uint16_t SpecificationRevision : 2; // 规范版本
    uint16_t PortPowerRole : 1;         // 电源角色
    uint16_t MessageID : 3;             // 消息 ID
    uint16_t NumberOfDataObjects : 3;   // 数据对象数量
    uint16_t Extended : 1;              // 扩展消息标志
} pd_msg_header_t;

/* PD 消息结构体 */
typedef struct {
    volatile uint32_t status;       // 消息状态
    volatile uint32_t msg_id;       // 消息序号
    volatile uint16_t vbus_raw;     // VBUS 电压 (raw)
    volatile uint32_t timestamp_ms; // 运行时间 (ms)
    volatile uint8_t len;           // 消息长度
    uint8_t data[PD_MSG_MAX_LEN];   // 消息数据
} pd_msg_t;

/* 消息缓冲区结构体 */
typedef struct {
    pd_msg_t msgs[PD_MSG_BUFFER_SIZE]; // 消息数组
    volatile uint16_t read_idx;        // 读指针
    volatile uint16_t write_idx;       // 写指针
} pd_msg_buffer_t;

/* 控制消息类型 */
#define CTRL_GOODCRC 0x01

/* 函数声明 */
void print_message(pd_msg_t *msg);
void save_message(uint32_t status, uint8_t *data, uint8_t len);
void reset_message_counter(void);
pd_msg_buffer_t *get_message_buffer(void);
