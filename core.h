#ifndef RISCV_CORE_H
#define RISCV_CORE_H

#define BYTE                   1
#define HALF_WORD              2
#define WORD                   4

#define ACCESS_GRANTED         0
#define ACCESS_REVOKED         1

#define ACCESS_INSTRUCTION     0
#define ACCESS_DATA            1

#define CORE_MCAUSE_INTERRUPT_BITMASK   0x80000000
#define CORE_MSTATUS_SIE_BIT            1
#define CORE_MSTATUS_MIE_BIT            3
#define CORE_MSTATUS_SPIE_BIT           5
#define CORE_MSTATUS_UBE_BIT            6
#define CORE_MSTATUS_MPIE_BIT           7
#define CORE_MSTATUS_SPP_BIT            8
#define CORE_MSTATUS_VS_BITS            9
#define CORE_MSTATUS_MPP_BITS           11
#define CORE_MSTATUS_FS_BITS            13
#define CORE_MSTATUS_XS_BITS            15
#define CORE_MSTATUS_MPRV_BIT           17

#define CORE_MSTATUS_SIE_BITMASK        (1 << 1)
#define CORE_MSTATUS_MIE_BITMASK        (1 << 3)
#define CORE_MSTATUS_SPIE_BITMASK       (1 << 5)
#define CORE_MSTATUS_UBE_BITMASK        (1 << 6)
#define CORE_MSTATUS_MPIE_BITMASK       (1 << 7)
#define CORE_MSTATUS_SPP_BITMASK        (1 << 8)
#define CORE_MSTATUS_VS_BITMASK         (0b11 << 9)
#define CORE_MSTATUS_MPP_BITMASK        (0b11 << 11)
#define CORE_MSTATUS_FS_BITMASK         (0b11 << 13)
#define CORE_MSTATUS_XS_BITMASK         (0b11 << 15)
#define CORE_MSTATUS_MPRV_BITMASK       (1 << 17)


typedef struct riscv_core_struct
{
    
    uint64_t cycle;
    uint64_t time;
    uint64_t timercmp;

    /* Registers */
    uint32_t x[32];
    uint32_t pc;

    uint32_t mvendorid;
    uint32_t mepc;
    uint32_t mcause;
    uint32_t mtval;
    uint32_t mie;
    uint32_t mip;
    uint32_t mtvec;
    uint32_t mscratch;
    uint32_t mstatus;
    uint32_t misa;

    struct operations_struct
    {
        uint8_t (*mem_read)(uint32_t address, uint8_t type, uint32_t * dest, uint8_t bytes_num);
        uint8_t (*mem_write)(uint32_t address, uint32_t data, uint8_t type, uint8_t bytes_num);

    } operations;

    uint32_t trap: 1;
    uint32_t privilege:2;
    uint32_t load_reserved:1;
    uint32_t wait_int:1;

} riscv_core;



void Core_Init(riscv_core *core);
void Core_SingleCycle(riscv_core *core);

#endif /* RISCV_CORE_H */

