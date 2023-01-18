#ifndef RISCV_INSTRUCTIONS_H
#define RISCV_INSTRUCTIONS_H

#define INST_U_TYPE_OPCODE_LUI          0b0110111
#define INST_U_TYPE_OPCODE_AUIPC        0b0010111
#define INST_J_TYPE_OPCODE_JAL          0b1101111

#define INST_I_TYPE_OPCODE_JALR         0b1100111
#define INST_I_TYPE_OPCODE_LOAD         0b0000011
#define INST_I_TYPE_FUNCT3_LB           0b000
#define INST_I_TYPE_FUNCT3_LH           0b001
#define INST_I_TYPE_FUNCT3_LW           0b010
#define INST_I_TYPE_FUNCT3_LBU          0b100
#define INST_I_TYPE_FUNCT3_LHU          0b101
#define INST_I_TYPE_OPCODE_ALSH         0b0010011
#define INST_I_TYPE_FUNCT3_ADDI         0b000
#define INST_I_TYPE_FUNCT3_SLTI         0b010
#define INST_I_TYPE_FUNCT3_SLTIU        0b011
#define INST_I_TYPE_FUNCT3_XORI         0b100
#define INST_I_TYPE_FUNCT3_ORI          0b110
#define INST_I_TYPE_FUNCT3_ANDI         0b111
#define INST_I_TYPE_FUNCT3_SLLI         0b001
#define INST_I_TYPE_FUNCT3_SRLAI        0b101
#define INST_I_TYPE_IMM_SHAMT           0b11111
#define INST_I_TYPE_IMM_EXT_25_SLLI     0b0000000
#define INST_I_TYPE_IMM_EXT_25_SRLI     0b0000000
#define INST_I_TYPE_IMM_EXT_25_SRAI     0b0100000

#define INST_B_TYPE_OPCODE              0b1100011
#define INST_B_TYPE_FUNCT3_BEQ          0b000
#define INST_B_TYPE_FUNCT3_BNE          0b001
#define INST_B_TYPE_FUNCT3_BLT          0b100
#define INST_B_TYPE_FUNCT3_BGE          0b101
#define INST_B_TYPE_FUNCT3_BLTU         0b110
#define INST_B_TYPE_FUNCT3_BGEU         0b111

#define INST_R_TYPE_OPCODE              0b0110011
#define INST_R_TYPE_FUNCT3_ADD_SUB      0b000
#define INST_R_TYPE_FUNCT7_ADD          0b0000000
#define INST_R_TYPE_FUNCT7_SUB          0b0100000
#define INST_R_TYPE_FUNCT3_SLL          0b001
#define INST_R_TYPE_FUNCT3_SLT          0b010
#define INST_R_TYPE_FUNCT3_SLTU         0b011
#define INST_R_TYPE_FUNCT3_XOR          0b100
#define INST_R_TYPE_FUNCT3_SRL_SRA      0b101
#define INST_R_TYPE_FUNCT7_SRL          0b0000000
#define INST_R_TYPE_FUNCT7_SRA          0b0100000
#define INST_R_TYPE_FUNCT3_OR           0b110
#define INST_R_TYPE_FUNCT3_AND          0b111
#define INST_R_TYPE_FUNCT7_M_EXT        0b0000001
#define INST_R_TYPE_FUNCT3_MUL          0b000
#define INST_R_TYPE_FUNCT3_MULH         0b001
#define INST_R_TYPE_FUNCT3_MULHSU       0b010
#define INST_R_TYPE_FUNCT3_MULHU        0b011
#define INST_R_TYPE_FUNCT3_DIV          0b100
#define INST_R_TYPE_FUNCT3_DIVU         0b101
#define INST_R_TYPE_FUNCT3_REM          0b110
#define INST_R_TYPE_FUNCT3_REMU         0b111

#define INST_S_TYPE_OPCODE              0b0100011
#define INST_S_TYPE_FUNCT3_SB           0b000
#define INST_S_TYPE_FUNCT3_SH           0b001
#define INST_S_TYPE_FUNCT3_SW           0b010

#define INST_CSR_INT_OPCODE             0b1110011
#define INST_CSR_FUNCT3_INT             0b000
#define INST_CSR_FUNCT3_CSRRW           0b001
#define INST_CSR_FUNCT3_CSRRS           0b010
#define INST_CSR_FUNCT3_CSRRC           0b011
#define INST_CSR_FUNCT3_CSRRWI          0b101
#define INST_CSR_FUNCT3_CSRRSI          0b110
#define INST_CSR_FUNCT3_CSRRCI          0b111

#define INST_INT_IMM_ECALL              0b000000000000
#define INST_INT_IMM_EBREAK             0b000000000001
#define INST_INT_IMM_WFI                0b000100000101
#define INST_INT_IMM_MRET               0b001100000010 


#define INST_FENCE_OPCODE               0b0001111

#define INST_ATOMIC_OPCODE              0b0101111
#define INST_ATOMIC_FUNCT3_DEFAULT      0b010
#define INST_ATOMIC_FUNCT5_LR_W         0b00010
#define INST_ATOMIC_FUNCT5_SC_W         0b00011
#define INST_ATOMIC_FUNCT5_AMOSWAP_W    0b00001
#define INST_ATOMIC_FUNCT5_AMOADD_W     0b00000
#define INST_ATOMIC_FUNCT5_AMOXOR_W     0b00100
#define INST_ATOMIC_FUNCT5_AMOAND_W     0b01100
#define INST_ATOMIC_FUNCT5_AMOOR_W      0b01000
#define INST_ATOMIC_FUNCT5_AMOMIN_W     0b10000
#define INST_ATOMIC_FUNCT5_AMOMAX_W     0b10100
#define INST_ATOMIC_FUNCT5_AMOMINU_W    0b11000
#define INST_ATOMIC_FUNCT5_AMOMAXU_W    0b11100


#define CSR_ADDRESS_MSTATUS             0x300
#define CSR_ADDRESS_MISA                0x301
#define CSR_ADDRESS_MEDELEG             0x302
#define CSR_ADDRESS_MIDELEG             0x303
#define CSR_ADDRESS_MIE                 0x304
#define CSR_ADDRESS_MTVEC               0x305
#define CSR_ADDRESS_MCOUNTEREN          0x305
#define CSR_ADDRESS_MSCRATCH            0x340
#define CSR_ADDRESS_MEPC                0x341
#define CSR_ADDRESS_MCAUSE              0x342
#define CSR_ADDRESS_MTVAL               0x343
#define CSR_ADDRESS_MIP                 0x344
#define CSR_ADDRESS_CYCLE               0xC00
#define CSR_ADDRESS_TIME                0xC01
#define CSR_ADDRESS_CYCLEH              0xC80
#define CSR_ADDRESS_TIMEH               0xC81
#define CSR_ADDRESS_MVENDORID           0xF11



typedef struct R_type_struct
{
    uint32_t opcode : 7;
    uint32_t rd : 5;
    uint32_t funct3 : 3;
    uint32_t rs1 : 5;
    uint32_t rs2 : 5;
    uint32_t funct7 : 7;
} R_type_t;

typedef struct I_type_struct
{
    uint32_t opcode : 7;
    uint32_t rd : 5;
    uint32_t funct3 : 3;
    uint32_t rs1 : 5;
    uint32_t imm : 12;
} I_type_t;

typedef struct S_type_struct
{
    uint32_t opcode : 7;
    uint32_t imm_0_4 : 5;
    uint32_t funct3 : 3;
    uint32_t rs1 : 5;
    uint32_t rs2 : 5;
    uint32_t imm_5_11 : 7;
} S_type_t;

typedef struct U_type_struct
{
    uint32_t opcode : 7;
    uint32_t rd : 5;
    uint32_t imm : 20;
} U_type_t;

typedef struct B_type_struct
{
    uint32_t opcode : 7;
    uint32_t imm_11 : 1;
    uint32_t imm_1_4 : 4;
    uint32_t funct3 : 3;
    uint32_t rs1 : 5;
    uint32_t rs2 : 5;
    uint32_t imm_5_10 : 6;
    uint32_t imm_12 : 1;
} B_type_t;

typedef struct J_type_struct
{
    uint32_t opcode : 7;
    uint32_t rd : 5;
    uint32_t imm_12_19 : 8;
    uint32_t imm_11 : 1;
    uint32_t imm_1_10 : 10;
    uint32_t imm_20 : 1;
} J_type_t;

typedef struct Atomic_type_struct
{
    uint32_t opcode : 7;
    uint32_t rd : 5;
    uint32_t funct3 : 3;
    uint32_t rs1 : 5;
    uint32_t rs2 : 5;
    uint32_t rl : 1;
    uint32_t aq : 1;
    uint32_t funct5 : 5;
} Atomic_type_t;

typedef struct Uninitialized_type_struct
{
    uint32_t opcode : 7;
    uint32_t reserved : 25;
} Uninitialized_type_t;

#endif /* RISCV_INSTRUCTIONS_H */