/**
  ******************************************************************************
  * @file    nrf24l01.c
  * @brief   NRF24L01+ 接收模式驱动实现
  ******************************************************************************
  */
#include "nrf24l01.h"

/* 射频参数(必须与手持端完全一致) --------------------------------------------*/
#define NRF24_RF_CHANNEL      40      /* 载波频率 2400+40 = 2440MHz */
#define NRF24_RF_SETUP_VAL    0x06    /* 1Mbps, 0dBm */

/* 5 字节接收地址(必须与手持端发送地址一致) */
static const uint8_t s_rxAddress[5] = {0x1A, 0x2B, 0x3C, 0x4D, 0x5E};

/* 微秒级近似延时(72MHz 主频) */
static void NRF24_DelayUs(uint32_t us)
{
    volatile uint32_t cnt;
    for (cnt = 0; cnt < us * 6u; cnt++)
    {
        __NOP();
    }
}

/* 引脚控制 */
static void NRF24_CSN_Low(void)
{
    HAL_GPIO_WritePin(NRF24_CSN_PORT, NRF24_CSN_PIN, GPIO_PIN_RESET);
}

static void NRF24_CSN_High(void)
{
    HAL_GPIO_WritePin(NRF24_CSN_PORT, NRF24_CSN_PIN, GPIO_PIN_SET);
}

static void NRF24_CE_Low(void)
{
    HAL_GPIO_WritePin(NRF24_CE_PORT, NRF24_CE_PIN, GPIO_PIN_RESET);
}

static void NRF24_CE_High(void)
{
    HAL_GPIO_WritePin(NRF24_CE_PORT, NRF24_CE_PIN, GPIO_PIN_SET);
}

/* SPI 单字节收发 */
static uint8_t NRF24_SPI_RW(uint8_t data)
{
    uint8_t rx = 0x00;
    HAL_SPI_TransmitReceive(&hspi1, &data, &rx, 1, 100);
    return rx;
}

/* 写单个寄存器 */
static void NRF24_WriteReg(uint8_t reg, uint8_t value)
{
    NRF24_CSN_Low();
    NRF24_SPI_RW(NRF24_CMD_W_REGISTER | reg);
    NRF24_SPI_RW(value);
    NRF24_CSN_High();
    NRF24_DelayUs(1);
}

/* 读单个寄存器 */
static uint8_t NRF24_ReadReg(uint8_t reg)
{
    uint8_t value;
    NRF24_CSN_Low();
    NRF24_SPI_RW(NRF24_CMD_R_REGISTER | reg);
    value = NRF24_SPI_RW(NRF24_CMD_NOP);
    NRF24_CSN_High();
    NRF24_DelayUs(1);
    return value;
}

/* 连续写多个寄存器 */
static void NRF24_WriteBuf(uint8_t reg, const uint8_t *pBuf, uint8_t len)
{
    uint8_t i;
    NRF24_CSN_Low();
    NRF24_SPI_RW(NRF24_CMD_W_REGISTER | reg);
    for (i = 0; i < len; i++)
    {
        NRF24_SPI_RW(pBuf[i]);
    }
    NRF24_CSN_High();
    NRF24_DelayUs(1);
}

/* 连续读多个寄存器 */
static void NRF24_ReadBuf(uint8_t reg, uint8_t *pBuf, uint8_t len)
{
    uint8_t i;
    NRF24_CSN_Low();
    NRF24_SPI_RW(NRF24_CMD_R_REGISTER | reg);
    for (i = 0; i < len; i++)
    {
        pBuf[i] = NRF24_SPI_RW(NRF24_CMD_NOP);
    }
    NRF24_CSN_High();
    NRF24_DelayUs(1);
}

/* 初始化 NRF24L01 为接收模式 */
void NRF24_Init(void)
{
    GPIO_InitTypeDef gpio = {0};

    /* 将 CSN 引脚(PA4, CubeMX 中为 SPI1_NSS 软件模式)复用为普通推挽输出 */
    gpio.Pin   = NRF24_CSN_PIN;
    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(NRF24_CSN_PORT, &gpio);

    /* 初始引脚状态: CE=0, CSN=1 */
    NRF24_CE_Low();
    NRF24_CSN_High();
    HAL_Delay(20);                           /* 等待模块上电稳定 */

    /* 配置寄存器(先保持 PWR_UP=0 以便写入) */
    NRF24_WriteReg(NRF24_REG_CONFIG,     0x0C);
    NRF24_WriteReg(NRF24_REG_EN_AA,      0x01);   /* 使能管道0自动应答 */
    NRF24_WriteReg(NRF24_REG_EN_RXADDR,  0x01);   /* 使能接收管道0 */
    NRF24_WriteReg(NRF24_REG_SETUP_AW,   0x03);   /* 地址宽度 5 字节 */
    NRF24_WriteReg(NRF24_REG_SETUP_RETR, 0x00);   /* 接收模式无需自动重发 */
    NRF24_WriteReg(NRF24_REG_RF_CH,      NRF24_RF_CHANNEL);
    NRF24_WriteReg(NRF24_REG_RF_SETUP,   NRF24_RF_SETUP_VAL);
    NRF24_WriteBuf(NRF24_REG_RX_ADDR_P0, s_rxAddress, 5);   /* 管道0接收地址 */
    NRF24_WriteBuf(NRF24_REG_TX_ADDR,   s_rxAddress, 5);   /* ACK回发地址(必须与手持端 RX_ADDR_P0 一致) */
    NRF24_WriteReg(NRF24_REG_RX_PW_P0,   NRF24_PAYLOAD_WIDTH);

    /* 清除中断标志并清空收发 FIFO */
    NRF24_WriteReg(NRF24_REG_STATUS, 0x70);
    NRF24_CSN_Low();  NRF24_SPI_RW(NRF24_CMD_FLUSH_RX); NRF24_CSN_High();
    NRF24_CSN_Low();  NRF24_SPI_RW(NRF24_CMD_FLUSH_TX); NRF24_CSN_High();

    /* 进入接收模式, CONFIG = 0x7B:
     *   bit0 PRIM_RX=1 接收模式
     *   bit1 PWR_UP=1  上电
     *   bit2 CRCO=0    1 字节 CRC
     *   bit3 EN_CRC=1  使能 CRC
     *   bit4~6 = 1     屏蔽全部中断, 由软件轮询 STATUS */
    NRF24_WriteReg(NRF24_REG_CONFIG, 0x7B);

    NRF24_CE_High();                         /* CE 拉高, 开始监听 */
    HAL_Delay(1);
}

/* 检测模块通信是否正常(写读回 TX_ADDR 校验) */
uint8_t NRF24_Check(void)
{
    uint8_t wr[5] = {0xA5, 0xA5, 0xA5, 0xA5, 0xA5};
    uint8_t rd[5] = {0x00, 0x00, 0x00, 0x00, 0x00};
    uint8_t i;

    NRF24_WriteBuf(NRF24_REG_TX_ADDR, wr, 5);
    NRF24_ReadBuf(NRF24_REG_TX_ADDR, rd, 5);

    for (i = 0; i < 5; i++)
    {
        if (rd[i] != 0xA5)
        {
            return 0;                        /* 通信异常 */
        }
    }

    /* 自检后必须恢复真实地址(此处 TX_ADDR 同时是 ACK 回发地址), 否则对端收不到 ACK */
    NRF24_WriteBuf(NRF24_REG_TX_ADDR, s_rxAddress, 5);
    return 1;                                /* 通信正常 */
}

/* 接收一个数据包, 成功返回 1, 否则返回 0 */
uint8_t NRF24_RxPacket(uint8_t *pBuf, uint8_t len)
{
    uint8_t status;
    uint8_t i;

    status = NRF24_ReadReg(NRF24_REG_STATUS);

    /* 检查是否收到数据(RX_DR 置位) */
    if ((status & NRF24_STATUS_RX_DR) == 0)
    {
        return 0;
    }

    /* 读取负载 */
    NRF24_CE_Low();
    NRF24_CSN_Low();
    NRF24_SPI_RW(NRF24_CMD_R_RX_PAYLOAD);
    for (i = 0; i < len; i++)
    {
        pBuf[i] = NRF24_SPI_RW(NRF24_CMD_NOP);
    }
    NRF24_CSN_High();
    NRF24_DelayUs(1);

    /* 清除 RX_DR 标志(写 1 清除) */
    NRF24_WriteReg(NRF24_REG_STATUS, status | NRF24_STATUS_RX_DR);

    /* 若接收 FIFO 已满, 清空防止溢出 */
    if (NRF24_ReadReg(NRF24_REG_FIFO_STATUS) & 0x01)
    {
        NRF24_CSN_Low();
        NRF24_SPI_RW(NRF24_CMD_FLUSH_RX);
        NRF24_CSN_High();
    }

    NRF24_CE_High();                         /* 恢复监听 */
    return 1;
}
