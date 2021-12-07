#ifndef _SD_H
#define _SD_H

#include <stdbool.h>
#include <stdint.h>

/**
  * 错误码定义
  */
typedef enum
{
    SD_OK = 0,

    /* R1 */
    SD_E_ILLEGAL_COMMAND,		    // 卡的当前状态不接受该命令
    SD_E_COM_CRC_ERROR,			    // 前一个命令CRC检查错误
    SD_E_ERASE_SEQUENCE_ERROR,	    // 擦除命令序列错误
    SD_E_ADDRESS_MISALIGNED,	    // 没对齐的地址，同命令中使用的块长度不匹配
    SD_E_CMD_ARG_ERROR,			    // 命令的参数超出卡片的接受范围

    /* R2 */
    SD_E_CARD_LOCKED,			    // 卡已被加锁
    SD_E_LOCKUNLOCK_ERROR,		    // 加锁解锁命令发生错误
    SD_E_UNKNOWN_ERROR,             // 未知错误
    SD_E_CC_ERROR,                  // 内部的卡控制器错误
    SD_E_CARD_ECC_FAILED,           // 内部ECC收到但是数据不对
    SD_E_WP_VIOLATION,              // 试图对写保护卡进行操作
    SD_E_BAD_ERASE_PARAM,           // 擦除参数错误
    SD_E_OUT_OF_RANGE,              // 块地址越界

    /* Data Response */
    SD_E_DATA_CRC_ERROR,            // 数据CRC检查错误
    SD_E_DATA_WRITE_ERROR,          // 写数据时发生错误

    /* 通用错误 */
    SD_E_CMD_RESP_TIMEOUT,		// 命令应答超时
    SD_E_DATA_RESP_TIMEOUT,     // 读数据块超时
    SD_E_UNUSABLE_CARD          // 不可用的卡
} SD_Error;


/*
* SD版本1.0x相关数据
*/
typedef struct
{
    uint8_t  Reserved1:2; 
    uint16_t DeviceSize:12;             // 设备尺寸。用于计算卡容量
    uint8_t  MaxRdCurrentVDDMin:3; 
    uint8_t  MaxRdCurrentVDDMax:3; 
    uint8_t  MaxWrCurrentVDDMin:3;
    uint8_t  MaxWrCurrentVDDMax:3; 
    uint8_t  DeviceSizeMul:3;           // 设备尺寸乘数。用于计算卡容量
} struct_v1;

/*
* SD版本2.0相关数据
*/
typedef struct
{
    uint8_t  Reserved1:6;
    uint32_t DeviceSize:22;             // 设备尺寸。用于计算卡容量
    uint8_t  Reserved2:1;
} struct_v2;


#define SD_CSD_STRUCT_V1	0x0
#define SD_CSD_STRUCT_V2 	0x1


/**
  * SD卡CSD寄存器
  */
typedef struct
{
    uint8_t  CSDStruct:2;               // CSD 结构体
    uint8_t  Reserved1:6;
    uint8_t  TAAC:8;                    // 数据读访问时间。固定为 0Eh，代表1ms
    uint8_t  NSAC:8;                    // 以时钟周期为单位的数据读访问时间。固定为00h
    uint8_t  MaxBusClkFrec:8;           // 最大数据传输速率
    uint16_t CardComdClasses:12;        // 卡命令类。位0=1，表示支持CLASS0命令，位1=1，表示支持CLASS1命令, ...
    uint8_t  RdBlockLen:4;              // 最大读数据块最大长度。这个值固定为 09h，代表512字节
    uint8_t  PartBlockRead:1;           // 是否允许部分块读操作。这个值固定为0，表示不允许，只能按块进行访问
    uint8_t  WrBlockMisalign:1;         // 是否允许写块时地址不对齐。对于高容量卡，这个值固定为0，表示写操作不允许越过物理块边界
    uint8_t  RdBlockMisalign:1;         // 是否允许读块时地址不对齐。对于高容量卡，这个值固定为0，表示读操作不允许越过物理块边界
    uint8_t  DSRImpl:1;                 // 指示Configurable Driver Stage 是否已集成在卡上

    union csd_version {
        struct_v1 v1;
        struct_v2 v2;
    } version;

    uint8_t  EraseSingleBlockEnable:1;  // 是否允许块擦除操作。这个值固定为1，表示允许
    uint8_t  EraseSectorSize:7;         // 扇区大小。这个值固定为 7Fh
    uint8_t  WrProtectGrSize:7;         // 写保护组大小。对于SDHC/SDXC，这个值固定为0，表示不支持写保护组
    uint8_t  WrProtectGrEnable:1;       // 是否允许写保护组。对于SDHC/SDXC，这个值固定为0，表示不允许
    uint8_t  Reserved2:2;
    uint8_t  WrSpeedFact:3;             // 块编程时间因数
    uint8_t  MaxWrBlockLen:4;           // 最大写数据块长度
    uint8_t  WriteBlockPartial:1;       // 是否允许部分块写操作
    uint8_t  Reserved3:5;
    uint8_t  FileFormatGroup:1;        	// 文件格式组
    uint8_t  CopyFlag:1;                // 指标卡内容是原始的(0)还是拷贝的(1)
    uint8_t  PermWrProtect:1;           // 指示卡内容是否永久写保护。默认值为0，非永久写保护
    uint8_t  TempWrProtect:1;           // 指示卡内容是否临时写保护。这个值可以设置和复位。默认值为0，非写保护
    uint8_t  FileFormat:2;              // 文件格式
    uint8_t  Reserved4:2;
    uint8_t  crc:7;
    uint8_t  Reserved5:1;               // 总是 1
} SD_CSD;

/*
 * SD卡CID寄存器
*/
typedef struct
{
    uint8_t  ManufacturerID;            // 制造商ID
    uint16_t OEM_AppliID;               // OEM/应用ID
    char 	 ProdName[6];               // 产品名称
    uint8_t  ProdRev;                   // 产品版本
    uint32_t ProdSN;                    // 产品序列号
    uint8_t  Reserved1;
    uint16_t ManufactDate;              // 生产日期
    uint8_t  CID_CRC;                   // CRC7
    uint8_t  Reserved2;                 // 总是 1
} SD_CID;


#define SD_BLOCK_SIZE		512	// 逻辑块大小

typedef struct {
    SD_CSD		CSD;
    SD_CID		CID;
    uint32_t	Capacity;		// 卡容量大小，MB单位
    uint32_t 	BlocksNum;		// 以SD_BLOCK_SIZE为单位计算的块数
} SD_CardInfo;


SD_Error SD_Init(void);
void SD_DeInit(void);

SD_Error SD_ResetAllCards(void);
SD_Error SD_SetCrcOn(bool enable);

SD_Error SD_GetCardInfo(SD_CardInfo *cardInfo);
SD_Error SD_ReadCSD(SD_CSD *CSD);
SD_Error SD_ReadCID(SD_CID *CID);

SD_Error SD_ReadSingleBlock(uint8_t *buf, uint32_t blkNo);
SD_Error SD_ReadBlocks(uint8_t *buf, uint32_t blkNo, uint32_t blksToRead);
SD_Error SD_WriteSingleBlock(const uint8_t *buf, uint32_t blkNo);
SD_Error SD_WriteBlocks(const uint8_t *buf, uint32_t blkNo, uint32_t blksToWrite);
SD_Error SD_Erase(uint32_t startBlkNo, uint32_t blksToErase);

#endif
