//
// Created by ASUS on 2026/7/23.
//

#ifndef JIUXIAN_SP_CHASSIS_XPP_VOFA_ASCII_H
#define JIUXIAN_SP_CHASSIS_XPP_VOFA_ASCII_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stdint.h"
#include "stdbool.h"

/**
 * @file    vofa_ascii.h
 * @brief   STM32 优化的 ASCII 码表与字符处理工具
 * @author  Yuanbao
 * @date    2026-07-23
 * @version 2.0.0 (STM32 精简版)
 *
 * @details
 *  本文件为原 ascii_table.h 的 STM32 嵌入式精简版，去掉了：
 *    - printf / 打印函数（无 stdio 依赖）
 *    - 控制字符名称字符串表（节省 ~600B Flash）
 *    - 完整 128 项码表结构体数组（节省 ~2KB Flash）
 *    - ascii_to_repr() 转义函数
 *
 *  保留并增强：
 *    1. 全部 ASCII 码值宏定义
 *    2. 分类判断宏（纯运算，零内存）
 *    3. 大小写转换 / 数值转换宏
 *    4. 内联查找、比较、长度工具函数（协议解析用）
 *
 *  依赖：仅 stdint.h + stdbool.h，可 -ffreestanding 编译
 */

/* --- 控制字符 (0x00 ~ 0x1F) & DEL (0x7F) --- */

#define ASCII_NUL    0x00   /**< Null 空字符                */
#define ASCII_SOH    0x01   /**< Start of Heading           */
#define ASCII_STX    0x02   /**< Start of Text              */
#define ASCII_ETX    0x03   /**< End of Text                */
#define ASCII_EOT    0x04   /**< End of Transmission         */
#define ASCII_ENQ    0x05   /**< Enquiry                    */
#define ASCII_ACK    0x06   /**< Acknowledge                */
#define ASCII_BEL    0x07   /**< Bell 响铃                  */
#define ASCII_BS     0x08   /**< Backspace 退格             */
#define ASCII_HT     0x09   /**< Horizontal Tab 水平制表     */
#define ASCII_LF     0x0A   /**< Line Feed 换行 (\n)         */
#define ASCII_VT     0x0B   /**< Vertical Tab 垂直制表        */
#define ASCII_FF     0x0C   /**< Form Feed 换页             */
#define ASCII_CR     0x0D   /**< Carriage Return 回车 (\r)   */
#define ASCII_SO     0x0E   /**< Shift Out                  */
#define ASCII_SI     0x0F   /**< Shift In                   */
#define ASCII_DLE    0x10   /**< Data Link Escape           */
#define ASCII_DC1    0x11   /**< Device Control 1 (XON)     */
#define ASCII_DC2    0x12   /**< Device Control 2           */
#define ASCII_DC3    0x13   /**< Device Control 3 (XOFF)    */
#define ASCII_DC4    0x14   /**< Device Control 4           */
#define ASCII_NAK    0x15   /**< Negative Acknowledge       */
#define ASCII_SYN    0x16   /**< Synchronous Idle           */
#define ASCII_ETB    0x17   /**< End of Transmission Block   */
#define ASCII_CAN    0x18   /**< Cancel                     */
#define ASCII_EM     0x19   /**< End of Medium              */
#define ASCII_SUB    0x1A   /**< Substitute                 */
#define ASCII_ESC    0x1B   /**< Escape                     */
#define ASCII_FS     0x1C   /**< File Separator             */
#define ASCII_GS     0x1D   /**< Group Separator            */
#define ASCII_RS     0x1E   /**< Record Separator           */
#define ASCII_US     0x1F   /**< Unit Separator             */
#define ASCII_DEL    0x7F   /**< Delete 删除                */

/* --- 可打印字符 (0x20 ~ 0x7E) --- */

#define ASCII_SPACE  0x20   /**< 空格 ' '                   */
#define ASCII_EXCL   0x21   /**< 感叹号 '!'                 */
#define ASCII_DQUO   0x22   /**< 双引号 '"'                 */
#define ASCII_HASH   0x23   /**< 井号 '#'                   */
#define ASCII_DOLL   0x24   /**< 美元符 '$'                 */
#define ASCII_PERC   0x25   /**< 百分号 '%'                 */
#define ASCII_AMP    0x26   /**< 和号 '&'                   */
#define ASCII_SQUO   0x27   /**< 单引号 '\''                */
#define ASCII_LPAR   0x28   /**< 左括号 '('                 */
#define ASCII_RPAR   0x29   /**< 右括号 ')'                 */
#define ASCII_AST    0x2A   /**< 星号 '*'                   */
#define ASCII_PLUS   0x2B   /**< 加号 '+'                   */
#define ASCII_COMMA  0x2C   /**< 逗号 ','                   */
#define ASCII_MINUS  0x2D   /**< 减号 '-'                   */
#define ASCII_DOT    0x2E   /**< 句号 '.'                   */
#define ASCII_SLASH  0x2F   /**< 斜杠 '/'                   */

/* 数字 '0' ~ '9' : 0x30 ~ 0x39 */
#define ASCII_0      0x30
#define ASCII_1      0x31
#define ASCII_2      0x32
#define ASCII_3      0x33
#define ASCII_4      0x34
#define ASCII_5      0x35
#define ASCII_6      0x36
#define ASCII_7      0x37
#define ASCII_8      0x38
#define ASCII_9      0x39

#define ASCII_COLON  0x3A   /**< 冒号 ':'                   */
#define ASCII_SEMI   0x3B   /**< 分号 ';'                   */
#define ASCII_LT     0x3C   /**< 小于号 '<'                 */
#define ASCII_EQ     0x3D   /**< 等号 '='                   */
#define ASCII_GT     0x3E   /**< 大于号 '>'                 */
#define ASCII_QSTN   0x3F   /**< 问号 '?'                   */

/* 大写字母 'A' ~ 'Z' : 0x41 ~ 0x5A */
#define ASCII_A      0x41
#define ASCII_B      0x42
#define ASCII_C      0x43
#define ASCII_D      0x44
#define ASCII_E      0x45
#define ASCII_F      0x46
#define ASCII_G      0x47
#define ASCII_H      0x48
#define ASCII_I      0x49
#define ASCII_J      0x4A
#define ASCII_K      0x4B
#define ASCII_L      0x4C
#define ASCII_M      0x4D
#define ASCII_N      0x4E
#define ASCII_O      0x4F
#define ASCII_P      0x50
#define ASCII_Q      0x51
#define ASCII_R      0x52
#define ASCII_S      0x53
#define ASCII_T      0x54
#define ASCII_U      0x55
#define ASCII_V      0x56
#define ASCII_W      0x57
#define ASCII_X      0x58
#define ASCII_Y      0x59
#define ASCII_Z      0x5A

#define ASCII_LBRK   0x5B   /**< 左方括号 '['               */
#define ASCII_BSLH   0x5C   /**< 反斜杠 '\\'                */
#define ASCII_RBRK   0x5D   /**< 右方括号 ']'               */
#define ASCII_CARET  0x5E   /**< 脱字符 '^'                 */
#define ASCII_UNDER  0x5F   /**< 下划线 '_'                 */

/* 小写字母 'a' ~ 'z' : 0x61 ~ 0x7A */
#define ASCII_a      0x61
#define ASCII_b      0x62
#define ASCII_c      0x63
#define ASCII_d      0x64
#define ASCII_e      0x65
#define ASCII_f      0x66
#define ASCII_g      0x67
#define ASCII_h      0x68
#define ASCII_i      0x69
#define ASCII_j      0x6A
#define ASCII_k      0x6B
#define ASCII_l      0x6C
#define ASCII_m      0x6D
#define ASCII_n      0x6E
#define ASCII_o      0x6F
#define ASCII_p      0x70
#define ASCII_q      0x71
#define ASCII_r      0x72
#define ASCII_s      0x73
#define ASCII_t      0x74
#define ASCII_u      0x75
#define ASCII_v      0x76
#define ASCII_w      0x77
#define ASCII_x      0x78
#define ASCII_y      0x79
#define ASCII_z      0x7A

#define ASCII_LCBR   0x7B   /**< 左花括号 '{'               */
#define ASCII_PIPE   0x7C   /**< 竖线 '|'                   */
#define ASCII_RCBR   0x7D   /**< 右花括号 '}'               */
#define ASCII_TILD   0x7E   /**< 波浪号 '~'                 */


/* =========================================================================
 * 2. 分类判断宏（纯运算，零内存占用，适合 STM32）
 * ========================================================================= */

/** @brief 判断是否为 ASCII 控制字符 (0x00~0x1F, 0x7F) */
#define ASCII_IS_CTRL(c)   (((c) >= 0x00 && (c) <= 0x1F) || ((c) == 0x7F))

/** @brief 判断是否为可打印 ASCII 字符 (0x20~0x7E) */
#define ASCII_IS_PRINT(c)  (((c) >= 0x20 && (c) <= 0x7E))

/** @brief 判断是否为空白字符 (空格/制表/换行/回车/换页/垂直制表) */
#define ASCII_IS_SPACE(c)  (((c) == ' ') || ((c) == '\t') || ((c) == '\n') || \
                            ((c) == '\r') || ((c) == '\f') || ((c) == '\v'))

/** @brief 判断是否为数字 '0'~'9' */
#define ASCII_IS_DIGIT(c)  (((c) >= '0') && ((c) <= '9'))

/** @brief 判断是否为大写字母 'A'~'Z' */
#define ASCII_IS_UPPER(c)  (((c) >= 'A') && ((c) <= 'Z'))

/** @brief 判断是否为小写字母 'a'~'z' */
#define ASCII_IS_LOWER(c)  (((c) >= 'a') && ((c) <= 'z'))

/** @brief 判断是否为字母 */
#define ASCII_IS_ALPHA(c)  (ASCII_IS_UPPER(c) || ASCII_IS_LOWER(c))

/** @brief 判断是否为字母或数字 */
#define ASCII_IS_ALNUM(c)  (ASCII_IS_ALPHA(c) || ASCII_IS_DIGIT(c))

/** @brief 判断是否为十六进制数字字符 */
#define ASCII_IS_XDIGIT(c) (ASCII_IS_DIGIT(c) || \
                            (((c) >= 'A') && ((c) <= 'F')) || \
                            (((c) >= 'a') && ((c) <= 'f')))

/** @brief 判断是否为标点符号（除空格外的可打印非字母数字） */
#define ASCII_IS_PUNCT(c)  (((c) >= 0x21 && (c) <= 0x2F) || \
                            ((c) >= 0x3A && (c) <= 0x40) || \
                            ((c) >= 0x5B && (c) <= 0x60) || \
                            ((c) >= 0x7B && (c) <= 0x7E))

/** @brief 判断是否为 ASCII 范围内的合法值 (0~127) */
#define ASCII_IS_VALID(c)  (((c) & 0x80) == 0x00)

/**
 * @brief 判断是否为行结束符（LF 或 CR）
 */
#define ASCII_IS_NL(c)     (((c) == '\n') || ((c) == '\r'))

/**
 * @brief 判断是否为 VOFA 协议帧尾标志（'\n'）
 */
#define ASCII_IS_VOFA_END(c)  ((c) == '\n')

/* =========================================================================
 * 3. 大小写转换宏
 * ========================================================================= */

/** @brief 大写转小写（仅对字母有效） */
#define ASCII_TO_LOWER(c)  (((c) >= 'A' && (c) <= 'Z') ? ((c) + 0x20) : (c))

/** @brief 小写转大写（仅对字母有效） */
#define ASCII_TO_UPPER(c)  (((c) >= 'a' && (c) <= 'z') ? ((c) - 0x20) : (c))

/** @brief 字符转对应的数值（'0'~'9' -> 0~9，非法返回 -1） */
#define ASCII_DIGIT_VAL(c) \
    ((int8_t)(((c) >= '0' && (c) <= '9') ? ((c) - '0') : -1))

/** @brief 十六进制字符转数值（'0'~'9','A'~'F','a'~'f' -> 0~15，非法返回 -1） */
#define ASCII_XDIGIT_VAL(c) \
    ((int8_t)( \
        ((c) >= '0' && (c) <= '9') ? ((c) - '0') : \
        ((c) >= 'A' && (c) <= 'F') ? ((c) - 'A' + 10) : \
        ((c) >= 'a' && (c) <= 'f') ? ((c) - 'a' + 10) : -1 \
    ))


/* =========================================================================
 * 4. 内联查找函数（无字符串表，零 Flash 开销）
 * ========================================================================= */

/**
 * @brief   大写转小写
 * @return  转换后的字符（非大写字母原样返回）
 */
static inline uint8_t ascii_to_lower(uint8_t c)
{
    if (c >= 'A' && c <= 'Z') {
        return (uint8_t)(c + 0x20);
    }
    return c;
}

/**
 * @brief   小写转大写
 * @return  转换后的字符（非小写字母原样返回）
 */
static inline uint8_t ascii_to_upper(uint8_t c)
{
    if (c >= 'a' && c <= 'z') {
        return (uint8_t)(c - 0x20);
    }
    return c;
}

/**
 * @brief   数字字符转数值
 * @return  0~9；若非数字字符返回 -1
 */
static inline int8_t ascii_digit_value(uint8_t c)
{
    if (c >= '0' && c <= '9') {
        return (int8_t)(c - '0');
    }
    return -1;
}

/**
 * @brief   十六进制字符转数值
 * @return  0~15；若非十六进制字符返回 -1
 */
static inline int8_t ascii_xdigit_value(uint8_t c)
{
    if (c >= '0' && c <= '9') {
        return (int8_t)(c - '0');
    } else if (c >= 'A' && c <= 'F') {
        return (int8_t)(c - 'A' + 10);
    } else if (c >= 'a' && c <= 'f') {
        return (int8_t)(c - 'a' + 10);
    }
    return -1;
}

/**
 * @brief   计算字符串长度（最大限制）
 * @param   s       字符串指针
 * @param   max_len 最大长度限制（防止越界）
 * @return  字符串实际长度
 */
static inline uint16_t ascii_strlen(const char *s, uint16_t max_len)
{
    uint16_t len = 0;
    while (len < max_len && s[len] != '\0') {
        len++;
    }
    return len;
}

/**
 * @brief   比较两段缓冲区（忽略大小写）
 * @note    常用于不区分大小写的命令解析
 */
static inline bool ascii_memcmp_nocase(const char *p1, const char *p2, uint16_t len)
{
    for (uint16_t i = 0; i < len; i++) {
        uint8_t a = ascii_to_lower((uint8_t)p1[i]);
        uint8_t b = ascii_to_lower((uint8_t)p2[i]);
        if (a != b) {
            return false;
        }
    }
    return true;
}

#ifdef __cplusplus
}  /* extern "C" */
#endif

#endif  /* JIUXIAN_SP_CHASSIS_XPP_VOFA_ASCII_H */
