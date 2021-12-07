#include "stm32f10x.h"
#include "SD.h"
#include "SD_SPI.h"
#include "SD_Util.h"


/**
  *  命令索引
  */
#define CMD0_GO_IDLE_STATE				0
#define CMD8_SEND_IF_COND				8
#define CMD9_SEND_CSD             		9
#define CMD10_SEND_CID             		10
#define CMD12_STOP_TRANSMISSION			12
#define CMD13_SEND_STATUS				13
#define CMD16_SET_BLOCK_LEN        		16
#define CMD17_READ_SINGLE_BLOCK    		17
#define CMD18_READ_MULTIPLE_BLOCK  		18
#define CMD24_WRITE_BLOCK          		24
#define CMD25_WRITE_MULTIPLE_BLOCK  	25
#define CMD27_PROGRAM_CSD          		27
#define CMD28_SET_WRITE_PROT        	28
#define CMD29_CLR_WRITE_PROT       		29
#define CMD30_SEND_WRITE_PROT      		30
#define CMD32_ERASE_WR_BLK_START    	32
#define CMD33_ERASE_WR_BLK_END       	33
#define CMD38_ERASE                		38
#define CMD42_LOCK_UNLOCK            	42
#define CMD55_APP_CMD                  	55
#define CMD56_GEN_CMD                  	56
#define CMD58_READ_OCR					58
#define CMD59_CRC_ON_OFF				59

#define ACMD22_SEND_NUM_WR_BLOCKS		22
#define ACMD41_SEND_OP_COND            	41

/*
 * R1 标志位
 */
#define R1_NO_ERROR					0x00
#define R1_IN_IDLE_STATE			0x01
#define R1_ERASE_RESET				0x02
#define R1_ILLEGAL_COMMAND			0x04
#define R1_COM_CRC_ERROR			0x08
#define R1_ERASE_SEQUENCE_ERROR		0x10
#define R1_ADDRESS_ERROR			0x20
#define R1_PARAMETER_ERROR			0x40

/*
 * R2 标志位
 */
#define R2_NO_ERROR 				0x00
#define R2_CARD_LOCKED				0x01
#define R2_LOCKUNLOCK_ERROR 		0x02
#define R2_ERROR               		0x04
#define R2_CC_ERROR            		0x08
#define R2_CARD_ECC_FAILED    		0x10
#define R2_WP_VIOLATION        		0x20
#define R2_BAD_ERASE_PARAM         	0x40
#define R2_OUT_OF_RANGE          	0x80

/*
 * Data Error Token 标志位
 */
#define DET_ERROR					0x01
#define DET_CC_ERROR				0x02
#define DET_CARD_ECC_FAILED			0x04
#define DET_OUT_OF_RANGE			0x08

/*
 * Data Response Token 标志位
 */
#define DRT_DATA_OK                	0x05
#define DRT_DATA_CRC_ERROR         	0x0B
#define DRT_DATA_WRITE_ERROR       	0x0D

/*
 * 控制令牌
 */
#define TOKEN_START_SINGLE_BLOCK_READ		0xFE
#define TOKEN_START_SINGLE_BLOCK_WRITE		0xFE
#define TOKEN_START_MULT_BLOCK_READ			0xFE
#define TOKEN_START_MULT_BLOCK_WRITE		0xFC
#define TOKEN_STOP_MULT_BLOCK_WRTIE			0xFD

/*
 * 其它宏定义
 */
#define CHECK_PATTERN           	0x000001AA
#define DUMMY_BYTE					0xFF

#define TIMEOUT_NCR					30	// 标准应答超时值
#define TIMEOUT_NAC					2000	// 读数据块时的超时值

/*
 * SD卡命令应答类型
 */
typedef enum {
    RESPTYPE_R1,
    RESPTYPE_R1b,
    RESPTYPE_R2,
    RESPTYPE_R3,
    RESPTYPE_R7
} RespType;

/*
 * SD卡命令应答信息结构
*/
typedef struct {
    uint8_t R1;
    uint8_t R2;
    uint32_t Data;		// 命令应答数据
} RespInfo;

/*
 * 块读写参数
 */
typedef struct {
    uint8_t *Buf;		// 指向要读/写的数据缓冲区
    uint16_t BlkSize;	// 块大小
    uint32_t BlksToTransfer; // 要读/写的块数
} DataParam;


/*
* 私有变量定义
*/
static bool g_flagSDHC = false;	// 指示当前卡是否是高容量卡
static bool g_flagBlkLenSetted = false; // 指示块长度是否已设置
static bool g_flagCrcOn = false;    // 指示CRC是否开启


/*
 * 私有函数原型声明
*/
static SD_Error SetBlockLength(uint32_t blkSize);
static SD_Error _CmdToResp(uint8_t cmd, uint32_t arg, RespType rt, RespInfo *ri);
static SD_Error CmdToResp(uint8_t cmd, uint32_t arg, RespType rt, RespInfo *ri);
static SD_Error CmdRead(uint8_t cmd, uint32_t arg, RespType rt, DataParam *dp, uint32_t Nac);
static SD_Error CmdWrite(uint8_t cmd, uint32_t arg, RespType rt, DataParam *dp);
static SD_Error DataWrite(uint8_t st, const uint8_t *blkBuf, uint16_t blkSize);
static SD_Error DataRead(uint8_t st, uint8_t *blkBuf, uint16_t blkSize, uint32_t Nac);
static SD_Error CheckR1(uint8_t r1);
static SD_Error CheckR2(uint8_t r2);
static SD_Error CheckProgrammingStatus(void);


SD_Error SD_Init(void)
{
    SD_SPI_Init();
    return SD_ResetAllCards();
}


void SD_DeInit(void)
{
    SD_SPI_DeInit();
}


/*
* 复位所有卡片。
* 返回值: SD_Error
*/
SD_Error SD_ResetAllCards(void)
{
    const uint16_t MaxRetry = 200;

    RespInfo ri;
    uint16_t count;
    SD_Error ret = SD_OK;

    g_flagSDHC = false;
    g_flagBlkLenSetted = false;
    g_flagCrcOn = false;

    SD_SPI_SetSpeedLow();

    /* 初始化延时至少74个时钟周期 */
    for (count = 0; count < 10; count++)
        SD_SPI_Exchange(DUMMY_BYTE);

    /* 等待卡片进入IDLE状态 */
    count = MaxRetry;
    do {
        ret = CmdToResp(CMD0_GO_IDLE_STATE, 0, RESPTYPE_R1, &ri);
        if (ret != SD_OK)
            return ret;

        if (ri.R1 == R1_IN_IDLE_STATE)
            break;
        count--;
    } while (count);

    if (count == 0)
        return CheckR1(ri.R1);

    /* 检查卡片操作条件 */
    ret = CmdToResp(CMD8_SEND_IF_COND, CHECK_PATTERN, RESPTYPE_R7, &ri);
    if (ret != SD_OK)
        return ret;

    if (ri.R1 & R1_ILLEGAL_COMMAND) {   // 1.X 卡或不是SD卡
        /* 初始化 1.X SD卡 */
        count = MaxRetry;
        do {
            /* 在执行ACMD命令之前必须先发送 CMD55 */
            ret = CmdToResp(CMD55_APP_CMD, 0, RESPTYPE_R1, &ri);
            if (ret != SD_OK)
                return ret;

            ret = CmdToResp(ACMD41_SEND_OP_COND, 0, RESPTYPE_R1, &ri);
            if (ret != SD_OK)
                return ret;

            if (ri.R1 == R1_NO_ERROR)
                break;
            count--;
        } while (count);

        if (count == 0) {
            return CheckR1(ri.R1);
        }
    }
    else {
        if (!(ri.Data & CHECK_PATTERN)) {
            return SD_E_UNUSABLE_CARD;
        }

        /* 初始化 2.0 以后的SD卡 */
        count = MaxRetry;
        do {
            ret = CmdToResp(CMD55_APP_CMD, 0, RESPTYPE_R1, &ri);
            if (ret != SD_OK)
                return ret;

            ret = CmdToResp(ACMD41_SEND_OP_COND, 0x40000000, RESPTYPE_R1, &ri);
            if (ret != SD_OK)
                return ret;

            if (ri.R1 == R1_NO_ERROR)
                break;
            count--;
        } while (count);

        if (count == 0) {
            return CheckR1(ri.R1);
        }

        /* 读OCR寄存器 */
        ret = CmdToResp(CMD58_READ_OCR, 0, RESPTYPE_R3, &ri);
        if (ret != SD_OK)
            return ret;

        ret = CheckR1(ri.R1);
        if (ret != SD_OK)
            return ret;

        /* 检查OCR寄存器的CCS位是否置1，置1则为高容量SD卡 (SDHC) */
        if (ri.Data & 0x40000000)
            g_flagSDHC = true;
    }

    SD_SPI_SetSpeedHigh();

    return SD_OK;
}


/*
* 读CSD寄存器。
*   参数:
*   [out] CSD   指向SD_CSD的指针
* 返回值:
*   SD_Error
*/
SD_Error SD_ReadCSD(SD_CSD *CSD)
{
    SD_Error ret;
    DataParam dp;
    static uint8_t CSD_Tab[16];

    /* 设置读参数 */
    dp.Buf = CSD_Tab;
    dp.BlkSize = sizeof(CSD_Tab);
    dp.BlksToTransfer = 1;

    /* 读数据 */
    ret = CmdRead(CMD9_SEND_CSD, 0, RESPTYPE_R1, &dp, TIMEOUT_NCR);
    if (ret != SD_OK)
        return ret;

    CSD->CSDStruct = (CSD_Tab[0] & 0xC0) >> 6;
    CSD->Reserved1 =  CSD_Tab[0] & 0x3F;

    CSD->TAAC = CSD_Tab[1];

    CSD->NSAC = CSD_Tab[2];

    CSD->MaxBusClkFrec = CSD_Tab[3];

    CSD->CardComdClasses = (CSD_Tab[4] << 4) | ((CSD_Tab[5] & 0xF0) >> 4);
    CSD->RdBlockLen = CSD_Tab[5] & 0x0F;

    CSD->PartBlockRead   = (CSD_Tab[6] & 0x80) >> 7;
    CSD->WrBlockMisalign = (CSD_Tab[6] & 0x40) >> 6;
    CSD->RdBlockMisalign = (CSD_Tab[6] & 0x20) >> 5;
    CSD->DSRImpl         = (CSD_Tab[6] & 0x10) >> 4;

    if(CSD->CSDStruct == SD_CSD_STRUCT_V1)
    {
        CSD->version.v1.Reserved1 = ((CSD_Tab[6] & 0x0C) >> 2);

        CSD->version.v1.DeviceSize =  ((CSD_Tab[6] & 0x03) << 10)
                                      |  (CSD_Tab[7] << 2)
                                      | ((CSD_Tab[8] & 0xC0) >> 6);
        CSD->version.v1.MaxRdCurrentVDDMin = (CSD_Tab[8] & 0x38) >> 3;
        CSD->version.v1.MaxRdCurrentVDDMax = (CSD_Tab[8] & 0x07);
        CSD->version.v1.MaxWrCurrentVDDMin = (CSD_Tab[9] & 0xE0) >> 5;
        CSD->version.v1.MaxWrCurrentVDDMax = (CSD_Tab[9] & 0x1C) >> 2;
        CSD->version.v1.DeviceSizeMul = ((CSD_Tab[9] & 0x03) << 1)
                                        |((CSD_Tab[10] & 0x80) >> 7);
    }
    else
    {
        CSD->version.v2.Reserved1 = ((CSD_Tab[6] & 0x0F) << 2) | ((CSD_Tab[7] & 0xC0) >> 6);
        CSD->version.v2.DeviceSize= ((CSD_Tab[7] & 0x3F) << 16) | (CSD_Tab[8] << 8) | CSD_Tab[9];
        CSD->version.v2.Reserved2 = ((CSD_Tab[10] & 0x80) >> 8);
    }

    CSD->EraseSingleBlockEnable = (CSD_Tab[10] & 0x40) >> 6;
    CSD->EraseSectorSize   = ((CSD_Tab[10] & 0x3F) << 1)
                             |((CSD_Tab[11] & 0x80) >> 7);
    CSD->WrProtectGrSize   = (CSD_Tab[11] & 0x7F);
    CSD->WrProtectGrEnable = (CSD_Tab[12] & 0x80) >> 7;
    CSD->Reserved2         = (CSD_Tab[12] & 0x60) >> 5;
    CSD->WrSpeedFact       = (CSD_Tab[12] & 0x1C) >> 2;
    CSD->MaxWrBlockLen     = ((CSD_Tab[12] & 0x03) << 2)
                             |((CSD_Tab[13] & 0xC0) >> 6);
    CSD->WriteBlockPartial = (CSD_Tab[13] & 0x20) >> 5;
    CSD->Reserved3         = (CSD_Tab[13] & 0x1F);
    CSD->FileFormatGroup  = (CSD_Tab[14] & 0x80) >> 7;
    CSD->CopyFlag          = (CSD_Tab[14] & 0x40) >> 6;
    CSD->PermWrProtect     = (CSD_Tab[14] & 0x20) >> 5;
    CSD->TempWrProtect     = (CSD_Tab[14] & 0x10) >> 4;
    CSD->FileFormat        = (CSD_Tab[14] & 0x0C) >> 2;
    CSD->Reserved4         = (CSD_Tab[14] & 0x03);
    CSD->crc               = (CSD_Tab[15] & 0xFE) >> 1;
    CSD->Reserved5         = (CSD_Tab[15] & 0x01);

    return SD_OK;
}


/*
* 读CID寄存器。
* 参数:
*   [out] CID   指向SD_CID的指针
* 返回值:
*   SD_Error
*/
SD_Error SD_ReadCID(SD_CID *CID)
{
    SD_Error ret;
    DataParam dp;
    static uint8_t CID_Tab[16];

    /* 设置读参数 */
    dp.Buf = CID_Tab;
    dp.BlkSize = sizeof(CID_Tab);
    dp.BlksToTransfer = 1;

    /* 读数据 */
    ret = CmdRead(CMD10_SEND_CID, 0, RESPTYPE_R1, &dp, TIMEOUT_NCR);
    if (ret != SD_OK)
        return ret;

    CID->ManufacturerID = CID_Tab[0];

    CID->OEM_AppliID = CID_Tab[1] << 8;
    CID->OEM_AppliID |= CID_Tab[2];

    CID->ProdName[0] = CID_Tab[3];
    CID->ProdName[1] = CID_Tab[4];
    CID->ProdName[2] = CID_Tab[5];
    CID->ProdName[3] = CID_Tab[6];
    CID->ProdName[4] = CID_Tab[7];
    CID->ProdName[5] = 0;

    CID->ProdRev = CID_Tab[8];

    CID->ProdSN = CID_Tab[9] << 24;
    CID->ProdSN |= CID_Tab[10] << 16;
    CID->ProdSN |= CID_Tab[11] << 8;
    CID->ProdSN |= CID_Tab[12];

    CID->Reserved1 |= (CID_Tab[13] & 0xF0) >> 4;

    CID->ManufactDate = (CID_Tab[13] & 0x0F) << 8;
    CID->ManufactDate |= CID_Tab[14];

    CID->CID_CRC = (CID_Tab[15] & 0xFE) >> 1;
    CID->Reserved2 = 1;

    return SD_OK;
}



/*
* 获取卡片信息。
* 参数:
* 	[out] cardIfo	 指向 SD_CardInfo 的指针
* 返回值:
*   SD_Error
*/
SD_Error SD_GetCardInfo(SD_CardInfo *cardInfo)
{
    uint32_t mult;
    uint32_t blkNr;
    uint32_t blkLen;
    SD_Error ret;

    /* 读CSD寄存器 */
    ret = SD_ReadCSD(&cardInfo->CSD);
    if (ret != SD_OK)
        return ret;

    /* 读CID寄存器 */
    ret = SD_ReadCID(&cardInfo->CID);
    if (ret != SD_OK)
        return ret;

    if (cardInfo->CSD.CSDStruct == SD_CSD_STRUCT_V1) {
        /*
        * capacity = BLOCKNR * BLOCK_LEN
        * BLOCKNR = (C_SIZE + 1) * MULT
        * MULT = 2^(C_SIZE_MULT + 2)
        * BLOCK_LEN = 2^(READ_BL_LEN)
        */
        blkLen = 1 << cardInfo->CSD.RdBlockLen;
        mult = 1 << (cardInfo->CSD.version.v1.DeviceSizeMul + 2);
        blkNr = (cardInfo->CSD.version.v1.DeviceSize+1) * mult;
        cardInfo->Capacity = (blkLen * blkNr) >> 20;	// 转换为MB
    }
    else {
        /* memory capacity = (C_SIZE + 1) * 512 KByte */
        cardInfo->Capacity = (cardInfo->CSD.version.v2.DeviceSize + 1) >> 1;
    }

    cardInfo->BlocksNum = (cardInfo->Capacity / SD_BLOCK_SIZE) << 20;

    return SD_OK;
}


/*
* 使能或禁用CRC。
* 参数:
*   [in] enable   true=使能, false=禁用
* 返回值:
*   SD_Error
*/
SD_Error SD_SetCrcOn(bool enable)
{
    SD_Error ret;
    RespInfo ri;

    ret = CmdToResp(CMD59_CRC_ON_OFF, enable? 1:0, RESPTYPE_R1, &ri);
    if (ret != SD_OK)
        return ret;

    ret = CheckR1(ri.R1);
    if (ret != SD_OK)
        return ret;

    g_flagCrcOn = enable;

    return SD_OK;
}


/*
* 读单个数据块。
*
* 参数:
*   [out] buf   用于接收数据的缓冲区
*   [in] blkNo  块号
* 返回值:
*   SD_Error
*/
SD_Error SD_ReadSingleBlock(uint8_t *buf, uint32_t blkNo)
{
    SD_Error ret;
    DataParam dp;
    uint32_t blkAddr = blkNo; 

    if (!g_flagSDHC) { // 标准容量卡
        /* 对于标准容量卡，块大小可以是任意字节。
        * 对于高容量SD卡，块大小固定为512字节。
        * 为了保持统一，这里把标准容量卡的块大小
        * 也设置为 SD_BLOCK_SIZE (512) 字节.
        */
        ret = SetBlockLength(SD_BLOCK_SIZE);
        if (ret != SD_OK)
            return ret;

        blkAddr = blkNo * SD_BLOCK_SIZE; // 设置标准容量卡的块地址
    }

    dp.Buf = buf;
    dp.BlkSize = SD_BLOCK_SIZE;
    dp.BlksToTransfer = 1;

    return CmdRead(CMD17_READ_SINGLE_BLOCK, blkAddr, RESPTYPE_R1, &dp, TIMEOUT_NAC);
}


/*
* 读多个数据块。
*
* 参数:
*   [out] buf   接收数据的缓冲区.
*   [in] starBlkNo  开始块号
*   [in] blksToRead 要读取的块数
* 返回值:
*   SD_Error
*/
SD_Error SD_ReadBlocks(uint8_t *buf, uint32_t startBlkNo, uint32_t blksToRead)
{
    SD_Error ret;
    RespInfo ri;
    DataParam dp;
    uint32_t blkAddr = startBlkNo;

    if (!g_flagSDHC) { // 标准容量卡
        ret = SetBlockLength(SD_BLOCK_SIZE);
        if (ret != SD_OK)
            return ret;

        blkAddr = startBlkNo * SD_BLOCK_SIZE;
    }

    dp.Buf = buf;
    dp.BlkSize = SD_BLOCK_SIZE;
    dp.BlksToTransfer = blksToRead;

    ret = CmdRead(CMD18_READ_MULTIPLE_BLOCK, blkAddr, RESPTYPE_R1, &dp, TIMEOUT_NAC);
    if (ret != SD_OK)
        return ret;

    /* 强止停止读操作 */
    CmdToResp(CMD12_STOP_TRANSMISSION, 0, RESPTYPE_R1b, &ri);

    return SD_OK;
}


/*
* 写单个数据块。
*
* 参数:
*   [in] buf    要写入的数据
*   [in] blkNo  块号
* 返回值:
*   SD_Error
*/
SD_Error SD_WriteSingleBlock(const uint8_t *buf, uint32_t blkNo)
{
    SD_Error ret;
    DataParam dp;;
    uint32_t blkAddr = blkNo;

    if (!g_flagSDHC) {  // 标准容量卡
        ret = SetBlockLength(SD_BLOCK_SIZE);
        if (ret != SD_OK)
            return ret;

        blkAddr = blkNo * SD_BLOCK_SIZE;
    }

    dp.Buf = (uint8_t*)buf;
    dp.BlkSize = SD_BLOCK_SIZE;
    dp.BlksToTransfer = 1;

    ret = CmdWrite(CMD24_WRITE_BLOCK, blkAddr, RESPTYPE_R1, &dp);
    if (ret != SD_OK)
        return ret;

    return CheckProgrammingStatus();
}


/*
* 写多个数据块。
* 参数:
*   [in] buf        要写入的数据
*   [in] startBlkNo  起始块号
*   [in] blksToWrite    要写的块数
* 返回值:
*   SD_Error
*/
SD_Error SD_WriteBlocks(const uint8_t *buf, uint32_t startBlkNo, uint32_t blksToWrite)
{
    SD_Error ret;
    DataParam dp;;
    uint32_t blkAddr = startBlkNo;

    if (!g_flagSDHC) { // 标准容量卡
        ret = SetBlockLength(SD_BLOCK_SIZE);
        if (ret != SD_OK)
            return ret;

        blkAddr = startBlkNo * SD_BLOCK_SIZE;
    }

    dp.Buf = (uint8_t*)buf;
    dp.BlkSize = SD_BLOCK_SIZE;
    dp.BlksToTransfer = blksToWrite;

    ret = CmdWrite(CMD25_WRITE_MULTIPLE_BLOCK, blkAddr, RESPTYPE_R1, &dp);
    if (ret != SD_OK)
        return ret;

    return CheckProgrammingStatus();
}


/*
* 擦除数据块。
* 参数:
*   [in] startBlkNo     起始块号
*   [in] blksToErase    要擦除的块数
* 返回值:
*   SD_Error
*/
SD_Error SD_Erase(uint32_t startBlkNo, uint32_t blksToErase)
{
    SD_Error ret;
    RespInfo ri;
    uint32_t startBlkAddr;
    uint32_t endBlkAddr;

    if (blksToErase == 0)
        return SD_E_CMD_ARG_ERROR;

    if (!g_flagSDHC) { // 标准容量卡
        startBlkAddr *= SD_BLOCK_SIZE;
        endBlkAddr = startBlkAddr + blksToErase * SD_BLOCK_SIZE;
    }
    else { // 高容量卡
        startBlkAddr = startBlkNo;
        endBlkAddr = startBlkNo + (blksToErase - 1);
    }

    /* 设置要擦除的起始块地址 */
    ret = CmdToResp(CMD32_ERASE_WR_BLK_START, startBlkAddr, RESPTYPE_R1, &ri);
    if (ret != SD_OK)
        return ret;

    ret = CheckR1(ri.R1);
    if (ret != SD_OK)
        return ret;

    /* 设置要擦除的结束块地址 */
    ret = CmdToResp(CMD33_ERASE_WR_BLK_END, endBlkAddr, RESPTYPE_R1, &ri);
    if (ret != SD_OK)
        return ret;

    ret = CheckR1(ri.R1);
    if (ret != SD_OK)
        return ret;

    /* 开始擦除 */
    ret = CmdToResp(CMD38_ERASE, 0, RESPTYPE_R1b, &ri);
    if (ret != SD_OK)
        return ret;

    ret = CheckR1(ri.R1);
    if (ret != SD_OK)
        return ret;

    return SD_OK;
}


/*
* 设置数据块大小。
* 返回值: SD_Error
*/
static SD_Error SetBlockLength(uint32_t blkSize)
{
    RespInfo ri;
    SD_Error ret;

    if (!g_flagBlkLenSetted) {
        ret = CmdToResp(CMD16_SET_BLOCK_LEN, blkSize, RESPTYPE_R1, &ri);
        if (ret != SD_OK)
            return ret;

        ret = CheckR1(ri.R1);
        if (ret != SD_OK)
            return ret;

        g_flagBlkLenSetted = true;
    }

    return SD_OK;
}


/*
* 发送命令并接收命令应答。
* 参数:
*   [in] cmd    命令索引
*   [in] arg    命令参数
*   [in] rt     命令应答类型
*   [out] ri    命令应答信息
* 返回值:
*   SD_Error
* 备注:
*   该函数由 CmdResp, CmdRead, CmdWrite 内部使用。
*/
static SD_Error _CmdToResp(uint8_t cmd, uint32_t arg, RespType rt, RespInfo *ri)
{
    int i;
    uint8_t buf[6];
    uint32_t counter = TIMEOUT_NCR;

    buf[0] = cmd | 0x40;
    buf[1] = (uint8_t)(arg >> 24);
    buf[2] = (uint8_t)(arg >> 16);
    buf[3] = (uint8_t)(arg >> 8);
    buf[4] = (uint8_t)(arg);
    buf[5] = (uint8_t)((SDU_CalcCRC7(buf, 5) << 1) + 1);

    for(i = 0; i < sizeof(buf); i++)
        SD_SPI_Exchange(buf[i]);

    /* 循环读取R1直到读取到的字节不是 DUMMY_BYTE, 或者超时 */
    do {
        ri->R1 = SD_SPI_Exchange(DUMMY_BYTE);
        if (ri->R1 != DUMMY_BYTE)
            break;
        counter--;
    } while (counter);

    if (counter == 0)
        return SD_E_CMD_RESP_TIMEOUT;

    /*
     *如果 R1 的 Illegal Command 和 Com CRC Error 位为1，
     * 主机就不需要再接受剩余的字节
     */
    if (!(ri->R1 & (R1_ILLEGAL_COMMAND | R1_COM_CRC_ERROR))) {
        /* 根据应答类型读取数据 */
        switch (rt) {
        case RESPTYPE_R1:
        case RESPTYPE_R1b:
            break;
        case RESPTYPE_R2:
            ri->R2 = SD_SPI_Exchange(DUMMY_BYTE);
            break;
        case RESPTYPE_R3:
        case RESPTYPE_R7:
            ri->Data = SD_SPI_Exchange(DUMMY_BYTE) << 24;
            ri->Data += SD_SPI_Exchange(DUMMY_BYTE) << 16;
            ri->Data += SD_SPI_Exchange(DUMMY_BYTE) << 8;
            ri->Data += SD_SPI_Exchange(DUMMY_BYTE);
            break;
        }
    }

    return SD_OK;
}


/*
* 发送命令并接收命令应答。
* 参数:
*   [in] cmd    命令索引
*   [in] arg    命令参数
*   [in] rt     命令应答类型
*   [out] ri    命令应答信息
* 返回值:
*   SD_Error
*/
static SD_Error CmdToResp(uint8_t cmd, uint32_t arg, RespType rt, RespInfo *ri)
{
    SD_Error ret;

    SD_SPI_SetCSLow();
    SD_SPI_Exchange(DUMMY_BYTE);

    ret = _CmdToResp(cmd, arg, rt, ri);

    SD_SPI_SetCSHigh();
    SD_SPI_Exchange(DUMMY_BYTE);

    return ret;
}


/*
* 读单个数据块或多个数据块。
* 参数:
*   [in] cmd    命令索引
*   [in] arg    命令参数
*   [in] rt     命令应答类型
*   [in] dp     读/写参数
*   [out] Nac   最大读超时值
* 返回值:
*   SD_Error
*/
static SD_Error CmdRead(uint8_t cmd, uint32_t arg, RespType rt, DataParam *dp, uint32_t Nac)
{
    uint8_t st; // 开始令牌
    RespInfo ri;
    SD_Error ret = SD_OK;

    SD_SPI_SetCSLow();
    SD_SPI_Exchange(DUMMY_BYTE);

    ret = _CmdToResp(cmd, arg, rt, &ri);
    if (ret != SD_OK)
        goto L_END;

    ret = CheckR1(ri.R1);
    if (ret != SD_OK)
        goto L_END;

    if (cmd == CMD18_READ_MULTIPLE_BLOCK)
        st = TOKEN_START_MULT_BLOCK_READ;
    else
        st = TOKEN_START_SINGLE_BLOCK_READ;

    do {
        ret = DataRead(st, dp->Buf, dp->BlkSize, Nac);
        if (ret != SD_OK)
            goto L_END;

        dp->Buf += dp->BlkSize;
        dp->BlksToTransfer--;
    } while (dp->BlksToTransfer);

L_END:
    SD_SPI_SetCSHigh();
    SD_SPI_Exchange(DUMMY_BYTE);

    return ret;
}

/*
* 写单个数据块或多个数据块。
* 参数:
*   [in] cmd    命令索引
*   [in] arg    命令参数
*   [in] rt     命令应答类型
*   [in] dp     写参数
* 返回值:
*   SD_Error
*/
static SD_Error CmdWrite(uint8_t cmd, uint32_t arg, RespType rt, DataParam *dp)
{
    uint8_t st;
    RespInfo ri;
    SD_Error ret = SD_OK;

    SD_SPI_SetCSLow();
    SD_SPI_Exchange(DUMMY_BYTE);

    ret = _CmdToResp(cmd, arg, rt, &ri);
    if (ret != SD_OK)
        goto L_END;

    ret = CheckR1(ri.R1);
    if (ret != SD_OK)
        goto L_END;

    /* 根据命令索引设置开始令牌 */
    if (cmd == CMD25_WRITE_MULTIPLE_BLOCK)
        st = TOKEN_START_MULT_BLOCK_WRITE;
    else
        st = TOKEN_START_SINGLE_BLOCK_WRITE;

    do {
        ret = DataWrite(st, dp->Buf, dp->BlkSize);
        if (ret != SD_OK)
            goto L_END;

        dp->Buf += dp->BlkSize;
        dp->BlksToTransfer--;
    } while (dp->BlksToTransfer);

    /* 如果是多块写，写完数据后要发送停止令牌 */
    if (cmd == CMD25_WRITE_MULTIPLE_BLOCK)
        SD_SPI_Exchange(TOKEN_STOP_MULT_BLOCK_WRTIE);

L_END:
    SD_SPI_SetCSHigh();
    SD_SPI_Exchange(DUMMY_BYTE);

    return ret;
}


static SD_Error DataRead(uint8_t st, uint8_t *buf, uint16_t blkSize, uint32_t Nac)
{
    uint8_t b;
    uint16_t i, crc1, crc2;

    do {
        b = SD_SPI_Exchange(DUMMY_BYTE);
        if (b == st)
            break;

        if ((b >> 4) == 0 && b) {
            if (b & DET_ERROR)
                return SD_E_UNKNOWN_ERROR;

            if (b & DET_CC_ERROR)
                return SD_E_CC_ERROR;

            if (b & DET_CARD_ECC_FAILED)
                return SD_E_CARD_ECC_FAILED;

            if (b & DET_OUT_OF_RANGE)
                return SD_E_OUT_OF_RANGE;
        }
        Nac--;
    } while (Nac);

    if (Nac == 0)
        return SD_E_DATA_RESP_TIMEOUT;

    for (i = 0; i < blkSize; i++)
        buf[i] = SD_SPI_Exchange(DUMMY_BYTE);

    /* 读 CRC16  */
    crc1 = SD_SPI_Exchange(DUMMY_BYTE) << 8;
    crc1 += SD_SPI_Exchange(DUMMY_BYTE);

    if (g_flagCrcOn) {
        crc2 = SDU_CalcCRC16(buf, blkSize);
        if (crc1 != crc2)
            return SD_E_DATA_CRC_ERROR;
    }

    return SD_OK;
}


static SD_Error DataWrite(uint8_t st, const uint8_t *buf, uint16_t blkSize)
{
    uint8_t drt;	// 数据应答令牌
    uint16_t i, crc;

    SD_SPI_Exchange(st);

    for (i = 0; i < blkSize; i++)
        SD_SPI_Exchange(buf[i]);

    /* 写 CRC16  */
    if (g_flagCrcOn) {
        crc = SDU_CalcCRC16(buf, blkSize);
        SD_SPI_Exchange(crc >> 8);
        SD_SPI_Exchange((uint8_t)crc);
    }
    else {
        SD_SPI_Exchange(DUMMY_BYTE);
        SD_SPI_Exchange(DUMMY_BYTE);
    }

    drt = SD_SPI_Exchange(DUMMY_BYTE);  // 读数据应答令牌
    SD_SPI_Exchange(DUMMY_BYTE);	// 读 busy 字

    switch (drt & 0x1F) {
    case DRT_DATA_OK:
        /*
        * SD卡写数据时为低电平，此时读取的数据为0。
        * 如果读取的数据是 DUMMY_BYTE (0xFF), 表示写完成。
        */
        while (SD_SPI_Exchange(DUMMY_BYTE) != DUMMY_BYTE);

        return SD_OK;

    case DRT_DATA_CRC_ERROR:
        return SD_E_DATA_CRC_ERROR;

    default:
        return SD_E_DATA_WRITE_ERROR;
    }
}


static SD_Error CheckR1(uint8_t r1)
{
    if (r1 != R1_NO_ERROR) {
        if (r1 & R1_ILLEGAL_COMMAND)
            return SD_E_ILLEGAL_COMMAND;

        if (r1 & R1_COM_CRC_ERROR)
            return SD_E_COM_CRC_ERROR;

        if (r1 & R1_ERASE_SEQUENCE_ERROR)
            return SD_E_ERASE_SEQUENCE_ERROR;

        if (r1 & R1_ADDRESS_ERROR)
            return SD_E_ADDRESS_MISALIGNED;

        if (r1 & R1_PARAMETER_ERROR)
            return SD_E_CMD_ARG_ERROR;
    }

    return SD_OK;
}


static SD_Error CheckR2(uint8_t r2)
{
    if (r2 != R2_NO_ERROR) {
        if (r2 & R2_CARD_LOCKED)
            return SD_E_CARD_LOCKED;

        if (r2 & R2_LOCKUNLOCK_ERROR)
            return SD_E_LOCKUNLOCK_ERROR;

        if (r2 & R2_ERROR)
            return SD_E_UNKNOWN_ERROR;

        if (r2 & R2_CC_ERROR)
            return SD_E_CC_ERROR;

        if (r2 & R2_CARD_ECC_FAILED)
            return SD_E_CARD_ECC_FAILED;

        if (r2 & R2_WP_VIOLATION)
            return SD_E_WP_VIOLATION;

        if (r2 & R2_BAD_ERASE_PARAM)
            return SD_E_BAD_ERASE_PARAM;

        if (r2 & R2_OUT_OF_RANGE)
            return SD_E_OUT_OF_RANGE;
    }

    return SD_OK;
}


static SD_Error CheckProgrammingStatus(void)
{
    SD_Error ret;
    RespInfo ri;

    ret = CmdToResp(CMD13_SEND_STATUS, 0, RESPTYPE_R2, &ri);
    if (ret != SD_OK)
        return ret;

    ret = CheckR1(ri.R1);
    if (ret != SD_OK)
        return ret;

    ret = CheckR2(ri.R2);
    if (ret != SD_OK)
        return ret;

    return SD_OK;
}
