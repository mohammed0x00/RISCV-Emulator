#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "instructions.h"
#include "core.h"

#define EXEC_NORET              0x00
#define EXEC_RET_PC_NOINC       0x01

#define PRIVILEGE_USER          0x00
#define PRIVILEGE_MACHINE       0x03

#define EXCEPTION_INST_ADDR_MISALIGNED 0x0
#define EXCEPTION_INST_ACCESS_FAULT 0x1
#define EXCEPTION_ILLEGAL_INST 0x2
#define EXCEPTION_BREAKPOINT 0x3
#define EXCEPTION_LOAD_ACCESS_FAULT 0x5
#define EXCEPTION_STORE_ACCESS_FAULT 0x7
#define EXCEPTION_MEI 0xB
#define EXCEPTION_ECALL_U 0x8
#define EXCEPTION_ECALL_S 0x9
#define EXCEPTION_ECALL_RSVD 0xA
#define EXCEPTION_ECALL_M 0xB

#define SIGN_EXT(var, bit) (var | ((var & (1 << bit)) ? ~((1 << bit)-1) : 0))
#define TRAP(no) core->trap=1;core->mcause=no;

static uint8_t Execute_U_Type(riscv_core *core, U_type_t *inst);
static uint8_t Execute_J_Type(riscv_core *core, J_type_t *inst);
static uint8_t Execute_S_Type(riscv_core *core, S_type_t *inst);
static uint8_t Execute_I_Type(riscv_core *core, I_type_t *inst);
static uint8_t Execute_B_Type(riscv_core *core, B_type_t *inst);
static uint8_t Execute_R_Type(riscv_core *core, R_type_t *inst);
static uint8_t Execute_INT_Type(riscv_core *core, I_type_t *inst);
static uint8_t Execute_CSR(riscv_core *core, I_type_t *inst);
static uint32_t * GetCSR(riscv_core *core, uint16_t address);
static uint8_t Execute_Atomic(riscv_core *core, Atomic_type_t *inst);


void Core_Init(riscv_core *core)
{
    core->misa = 0x40401101;
    core->x[0] = 0;
    core->trap = 0;
    core->privilege = PRIVILEGE_MACHINE;
    core->wait_int =0;
}



void Core_SingleCycle(riscv_core *core)
{
    uint8_t ret;
    uint32_t word;

    
    Uninitialized_type_t * instruction = (Uninitialized_type_t *)(&word);
    core->cycle++;
    //core->time++;
    uint32_t oldpc = core->pc;

    if(core->time >= core->timercmp)
    {
        core->mip |= (1<<7);
        core->wait_int = 0;
    }
    else
    {
        core->mip &= ~(1<<7);
    }

    if(core->wait_int) return;

    static unsigned int counter = 0;
    counter++;
    if(counter == 6000001)
    {
       //printf("stop");
    }

    if(core->pc & 0b11)
    {
        TRAP(EXCEPTION_INST_ADDR_MISALIGNED);
    }
    else if(core->operations.mem_read(core->pc, ACCESS_INSTRUCTION, &word, WORD) == ACCESS_REVOKED)
    {
        TRAP(EXCEPTION_INST_ACCESS_FAULT);
    }
    else
    {
        switch(instruction->opcode)
        {
        case INST_U_TYPE_OPCODE_AUIPC:
        case INST_U_TYPE_OPCODE_LUI:
            ret = Execute_U_Type(core, (U_type_t *)instruction);
            break;
        case INST_J_TYPE_OPCODE_JAL:
            ret = Execute_J_Type(core, (J_type_t *)instruction);
            break;
        case INST_S_TYPE_OPCODE:
            ret = Execute_S_Type(core, (S_type_t *)instruction);
            break;
        case INST_I_TYPE_OPCODE_JALR:
        case INST_I_TYPE_OPCODE_LOAD:
        case INST_I_TYPE_OPCODE_ALSH:
            ret = Execute_I_Type(core, (I_type_t *)instruction);
            break;
        case INST_B_TYPE_OPCODE:
            ret = Execute_B_Type(core, (B_type_t *)instruction);
            break;
        case INST_R_TYPE_OPCODE:
            ret = Execute_R_Type(core, (R_type_t *)instruction);
            break;
        case INST_CSR_INT_OPCODE:
            switch(((I_type_t *)instruction)->funct3)
            {
            case INST_CSR_FUNCT3_INT:
                ret = Execute_INT_Type(core, (I_type_t *)instruction);
                break;
            default:
                ret = Execute_CSR(core, (I_type_t *)instruction);
                break;
            }
            break;
        case INST_ATOMIC_OPCODE:
            ret = Execute_Atomic(core, (Atomic_type_t *)instruction);
            break;
        case INST_FENCE_OPCODE:
            ret = EXEC_NORET;
            break;
        default: TRAP(EXCEPTION_ILLEGAL_INST); break;
        }
    }
    core->x[0] = 0;



    uint32_t myrd = (word >> 7) & 0x1f;
/*

    if(counter == 10000000) {exit(0);}
    if(counter >= 1)
    {
        if(myrd <= 31)
        {
            if(instruction->opcode == INST_B_TYPE_OPCODE || instruction->opcode == INST_J_TYPE_OPCODE_JAL || instruction->opcode == INST_I_TYPE_OPCODE_JALR || (instruction->opcode == INST_CSR_INT_OPCODE && ((I_type_t *)instruction)->funct3 == INST_CSR_FUNCT3_INT && ((I_type_t *)instruction)->imm ==INST_INT_IMM_MRET ))
                printf("%u 0x%08x -> 0x%08x  word=0x%08x\n",counter, oldpc, core->x[myrd], word);
            else
                printf("%u 0x%08x -> 0x%08x  word=0x%08x\n",counter, core-> pc, core->x[myrd], word);
        }
        else
            printf("%u 0x%08x\n",counter, core-> pc);
    }
*/

    if(core->mip & (1<<7))
    {
        if(!core->trap)
        {
            if(core->mie & (1<<7))
            {
                if(core->mstatus & CORE_MSTATUS_MIE_BITMASK)
                {
                    TRAP(0x80000007);
                }
            }
        }
    }
    if(core->trap)
    {
        if(core->mcause & CORE_MCAUSE_INTERRUPT_BITMASK)
        {
            core->mtval = 0;
            core->load_reserved = 0;
            core->pc += ((ret & EXEC_RET_PC_NOINC) == EXEC_RET_PC_NOINC)? 0:4;
        }
        else
        {
            uint8_t rd = ((I_type_t *)instruction)->rd;
            core->mtval = ((core->mcause+1 > 5) && (core->mcause+1 <=8))? core->x[rd]:core->pc;
        }
        
        core->mstatus &= ~CORE_MSTATUS_MPIE_BITMASK;
        core->mstatus |= ((CORE_MSTATUS_MIE_BITMASK & core->mstatus) << (CORE_MSTATUS_MPIE_BIT - CORE_MSTATUS_MIE_BIT));
        core->mstatus &= ~(CORE_MSTATUS_MPP_BITMASK | CORE_MSTATUS_MIE_BITMASK);
        core->mstatus |= core->privilege << CORE_MSTATUS_MPP_BITS;
        
        core->mepc = core->pc;
        core->pc = core->mtvec;
        
        core->trap=0;

    }
    else
    {
        core->pc += ((ret & EXEC_RET_PC_NOINC) == EXEC_RET_PC_NOINC)? 0:4;
    }

}


static uint8_t Execute_U_Type(riscv_core *core, U_type_t *inst)
{
    uint32_t imm = inst->imm << 12;
    uint8_t rd = inst->rd;
    switch(inst->opcode)
    {
    case INST_U_TYPE_OPCODE_LUI:
        core->x[rd] = imm;
        break;
    case INST_U_TYPE_OPCODE_AUIPC:
        core->x[rd] = (core->pc) + imm;
        break;
    default: TRAP(EXCEPTION_ILLEGAL_INST); break;
    }
    return EXEC_NORET;
}

static uint8_t Execute_J_Type(riscv_core *core, J_type_t *inst)
{
    uint32_t imm = (inst->imm_20 << 20) | (inst->imm_12_19 << 12) | (inst->imm_11 << 11) | (inst->imm_1_10 << 1);
    uint8_t rd = inst->rd;

    imm = SIGN_EXT(imm, 20);

    core->x[rd] = (core->pc) + 4;
    core->pc += imm;


    return EXEC_RET_PC_NOINC;
}


static uint8_t Execute_S_Type(riscv_core *core, S_type_t *inst)
{
    uint32_t imm = inst->imm_0_4 | (inst->imm_5_11 << 5);
    uint8_t rs1 = inst->rs1;
    uint8_t rs2 = inst->rs2;
    uint8_t access = ACCESS_GRANTED;

    imm = SIGN_EXT(imm, 11);

    switch(inst->funct3)
    {
    case INST_S_TYPE_FUNCT3_SB:
        access = core->operations.mem_write(core->x[rs1] + imm, (uint8_t)core->x[rs2], ACCESS_DATA, BYTE);
        break;
    case INST_S_TYPE_FUNCT3_SH:
        access = core->operations.mem_write(core->x[rs1] + imm, (uint16_t)core->x[rs2], ACCESS_DATA, HALF_WORD);
        break;
    case INST_S_TYPE_FUNCT3_SW:
        access = core->operations.mem_write(core->x[rs1] + imm, (uint32_t)core->x[rs2], ACCESS_DATA, WORD);
        break;
    default: TRAP(EXCEPTION_ILLEGAL_INST); break;
    }

    if(access == ACCESS_REVOKED)
    {
        TRAP(EXCEPTION_STORE_ACCESS_FAULT);
    }
    
    return EXEC_NORET;
}

static uint8_t Execute_I_Type(riscv_core *core, I_type_t *inst)
{
    uint32_t s_imm = SIGN_EXT(inst->imm, 11);
    uint8_t rd = inst->rd;
    uint8_t rs1 = inst->rs1;
    uint32_t t ;

    switch(inst->opcode)
    {
    case INST_I_TYPE_OPCODE_JALR:
        t = core->pc + 4;
        core->pc = (core->x[rs1] + s_imm) & (~1);
        core->x[rd] = t;
        return EXEC_RET_PC_NOINC;
        break;
    case INST_I_TYPE_OPCODE_LOAD:
        if(core->operations.mem_read(core->x[rs1] + s_imm, ACCESS_DATA, &t, WORD) == ACCESS_GRANTED)
        {
            switch(inst->funct3)
            {
            case INST_I_TYPE_FUNCT3_LB:
                core->x[rd] = SIGN_EXT((uint8_t)t, 7);
                break;
            case INST_I_TYPE_FUNCT3_LH:
                core->x[rd] = SIGN_EXT((uint16_t)t, 15);
                break;
            case INST_I_TYPE_FUNCT3_LW:
                core->x[rd] = t;
                break;
            case INST_I_TYPE_FUNCT3_LBU:
                core->x[rd] = (uint32_t)((uint8_t)t);
                break;
            case INST_I_TYPE_FUNCT3_LHU:
                core->x[rd] = (uint32_t)((uint16_t)t);
                break;

            default: TRAP(EXCEPTION_ILLEGAL_INST); break;
            }
        }
        else 
        {
            TRAP(EXCEPTION_LOAD_ACCESS_FAULT);
        }
        break;
    case INST_I_TYPE_OPCODE_ALSH:
        switch(inst->funct3)
        {
        case INST_I_TYPE_FUNCT3_ADDI:
            core->x[rd] = core->x[rs1] + s_imm;
            break;
        case INST_I_TYPE_FUNCT3_SLTI:
            core->x[rd] = ((int32_t)(core->x[rs1]) < (int32_t)(s_imm))? 1:0;
            break;
        case INST_I_TYPE_FUNCT3_SLTIU:
            core->x[rd] = (core->x[rs1] < s_imm)? 1:0;
            break;
        case INST_I_TYPE_FUNCT3_XORI:
            core->x[rd] = core->x[rs1] ^ s_imm;
            break;
        case INST_I_TYPE_FUNCT3_ORI:
            core->x[rd] = core->x[rs1] | s_imm;
            break;
        case INST_I_TYPE_FUNCT3_ANDI:
            core->x[rd] = core->x[rs1] & s_imm;
            break;
        case INST_I_TYPE_FUNCT3_SLLI:
            core->x[rd] = core->x[rs1] << (inst->imm & INST_I_TYPE_IMM_SHAMT);
            break;
        case INST_I_TYPE_FUNCT3_SRLAI:
            switch(inst->imm >> 5)
            {
            case INST_I_TYPE_IMM_EXT_25_SRLI:
                core->x[rd] = core->x[rs1] >> (inst->imm & INST_I_TYPE_IMM_SHAMT);
                break;
            case INST_I_TYPE_IMM_EXT_25_SRAI:
                core->x[rd] = ((int32_t)core->x[rs1]) >> (inst->imm & INST_I_TYPE_IMM_SHAMT);
                break;

            default: TRAP(EXCEPTION_ILLEGAL_INST); break;
            }
            break;

        default: TRAP(EXCEPTION_ILLEGAL_INST); break;
        }
        break;

    default: TRAP(EXCEPTION_ILLEGAL_INST); break;
    }
    return EXEC_NORET;
}

static uint8_t Execute_B_Type(riscv_core *core, B_type_t *inst)
{
    uint32_t imm = (inst->imm_12 << 12) | (inst->imm_11 << 11) | (inst->imm_5_10 << 5) | (inst->imm_1_4 << 1);
    uint8_t rs1 = inst->rs1;
    uint8_t rs2 = inst->rs2;

    imm = SIGN_EXT(imm, 12);

    switch(inst->funct3)
    {
    case INST_B_TYPE_FUNCT3_BEQ:
        if(core->x[rs1] == core->x[rs2])
        {
            core->pc += imm;
            return EXEC_RET_PC_NOINC;
        }
        break;
    case INST_B_TYPE_FUNCT3_BNE:
        if(core->x[rs1] != core->x[rs2])
        {
            core->pc += imm;
            return EXEC_RET_PC_NOINC;
        }
        break;
    case INST_B_TYPE_FUNCT3_BLT:
        if((int32_t)(core->x[rs1]) < (int32_t)(core->x[rs2]))
        {
            core->pc += imm;
            return EXEC_RET_PC_NOINC;
        }
        break;
    case INST_B_TYPE_FUNCT3_BGE:
        if((int32_t)(core->x[rs1]) >= (int32_t)(core->x[rs2]))
        {
            core->pc += imm;
            return EXEC_RET_PC_NOINC;
        }
        break;
    case INST_B_TYPE_FUNCT3_BLTU:
        if(core->x[rs1] < core->x[rs2])
        {
            core->pc += imm;
            return EXEC_RET_PC_NOINC;
        }
        break;
    case INST_B_TYPE_FUNCT3_BGEU:
        if(core->x[rs1] >= core->x[rs2])
        {
            core->pc += imm;
            return EXEC_RET_PC_NOINC;
        }
        break;

    default: TRAP(EXCEPTION_ILLEGAL_INST); break;
    }
    return EXEC_NORET;
}

static uint8_t Execute_R_Type(riscv_core *core, R_type_t *inst)
{
    uint8_t rs1 = inst->rs1;
    uint8_t rs2 = inst->rs2;
    uint8_t rd = inst->rd;

    switch(inst->funct7)
    {
    case INST_R_TYPE_FUNCT7_ADD:
    case INST_R_TYPE_FUNCT7_SUB:
        switch(inst->funct3)
        {
        case INST_R_TYPE_FUNCT3_ADD_SUB:
            switch (inst->funct7)
            {
            case INST_R_TYPE_FUNCT7_ADD:
                core->x[rd] = core->x[rs1] + core->x[rs2];
                break;
            case INST_R_TYPE_FUNCT7_SUB:
                core->x[rd] = core->x[rs1] - core->x[rs2];
                break;

            default: TRAP(EXCEPTION_ILLEGAL_INST); break;
            }
            break;

        case INST_R_TYPE_FUNCT3_SLL:
            core->x[rd] = core->x[rs1] << (core->x[rs2] & 0x1F);
            break;
        case INST_R_TYPE_FUNCT3_SLT:
            core->x[rd] = ((int32_t)(core->x[rs1]) < (int32_t)(core->x[rs2]))? 1:0;
            break;
        case INST_R_TYPE_FUNCT3_SLTU:
            core->x[rd] = (core->x[rs1] < core->x[rs2])? 1:0;
            break;
        case INST_R_TYPE_FUNCT3_XOR:
            core->x[rd] = core->x[rs1] ^ core->x[rs2];
            break;
        case INST_R_TYPE_FUNCT3_SRL_SRA:
            switch (inst->funct7)
            {
            case INST_R_TYPE_FUNCT7_SRL:
                core->x[rd] = core->x[rs1] >> (core->x[rs2] & 0x1F);
                break;
            case INST_R_TYPE_FUNCT7_SRA:
                core->x[rd] = ((int32_t)core->x[rs1]) >> (core->x[rs2] & 0x1F);
                break;

            default: TRAP(EXCEPTION_ILLEGAL_INST); break;
            }
            break;
            
        case INST_R_TYPE_FUNCT3_OR:
            core->x[rd] = core->x[rs1] | core->x[rs2];
            break;
        case INST_R_TYPE_FUNCT3_AND:
            core->x[rd] = core->x[rs1] & core->x[rs2];
            break;

        default: TRAP(EXCEPTION_ILLEGAL_INST); break;
        }
        break;

    case INST_R_TYPE_FUNCT7_M_EXT:
        switch(inst->funct3)
        {
        case INST_R_TYPE_FUNCT3_MUL:
            core->x[rd] = core->x[rs1] * core->x[rs2];
            break;
        case INST_R_TYPE_FUNCT3_MULH:
            core->x[rd] = (uint32_t)(((int64_t)(int32_t)core->x[rs1] * (int64_t)(int32_t)core->x[rs2]) >> 32);
            break;
        case INST_R_TYPE_FUNCT3_MULHSU:
            core->x[rd] = (uint32_t)(((int64_t)(int32_t)core->x[rs1] * (uint64_t)core->x[rs2]) >> 32);
            break;
        case INST_R_TYPE_FUNCT3_MULHU:
            core->x[rd] = ((uint64_t)core->x[rs1] * (uint64_t)core->x[rs2]) >> 32;
            break;
        case INST_R_TYPE_FUNCT3_DIV:
            core->x[rd] = ((int32_t)core->x[rs1] / (int32_t)core->x[rs2]);
            break;
        case INST_R_TYPE_FUNCT3_DIVU:
            core->x[rd] = core->x[rs1] / core->x[rs2];
            break;
        case INST_R_TYPE_FUNCT3_REM:
            core->x[rd] = ((int32_t)core->x[rs1] % (int32_t)core->x[rs2]);
            break;
        case INST_R_TYPE_FUNCT3_REMU:
            core->x[rd] = core->x[rs1] % core->x[rs2];
            break;

        default: TRAP(EXCEPTION_ILLEGAL_INST); break;
        }
        break;

    default: TRAP(EXCEPTION_ILLEGAL_INST); break;
    }
    return EXEC_NORET;
}

static uint8_t Execute_INT_Type(riscv_core *core, I_type_t *inst)
{
    uint32_t t;
    uint8_t ret = EXEC_NORET;
    if(inst->rd != 0 || inst->rs1 != 0)
    {
        TRAP(EXCEPTION_ILLEGAL_INST);
        return EXEC_NORET;
    }

    switch(inst->imm)
    {
    case INST_INT_IMM_ECALL:
        if(core->privilege & PRIVILEGE_MACHINE)
        {
            TRAP(EXCEPTION_ECALL_M);
        }
        else
        {
            TRAP(EXCEPTION_ECALL_U);
        }
        break;
    case INST_INT_IMM_EBREAK:
        TRAP(EXCEPTION_BREAKPOINT);
        break;
    case INST_INT_IMM_WFI:
        core->wait_int=1;
        core->mstatus |= CORE_MSTATUS_MIE_BITMASK;
        break;
    case INST_INT_IMM_MRET:
        t = core->mstatus;
        core->mstatus = ((t & CORE_MSTATUS_MPIE_BITMASK) >> (CORE_MSTATUS_MPIE_BIT - CORE_MSTATUS_MIE_BIT)) | (core->privilege << CORE_MSTATUS_MPP_BITS) | CORE_MSTATUS_MPIE_BITMASK;
        core->privilege = (t & CORE_MSTATUS_MPP_BITMASK) >> CORE_MSTATUS_MPP_BITS;
        core->pc = core->mepc;
        ret = EXEC_RET_PC_NOINC;
        break;
    default: TRAP(EXCEPTION_ILLEGAL_INST); break;
    }
    return ret;
}

static uint8_t Execute_CSR(riscv_core *core, I_type_t *inst)
{
    uint8_t rs1 = inst->rs1;
    uint32_t uimm = (uint32_t)inst->rs1;
    uint8_t rd = inst->rd;
    uint32_t * csr = GetCSR(core, inst->imm);
    uint32_t t;

    if(csr == NULL)
    {
        core->x[rd] = 0x00000000;
        return EXEC_NORET;
    }

    switch (inst->funct3)
    {
    case INST_CSR_FUNCT3_CSRRW:
        t = *csr;
        *csr = core->x[rs1];
        core->x[rd] = t;
        break;
    case INST_CSR_FUNCT3_CSRRS:
        t = *csr;
        *csr = t | core->x[rs1];
        core->x[rd] = t;
        break;
    case INST_CSR_FUNCT3_CSRRC:
        t = *csr;
        *csr = t & ~core->x[rs1];
        core->x[rd] = t;
        break;
    case INST_CSR_FUNCT3_CSRRWI:
        core->x[rd] = *csr;
        *csr = uimm;
        break;
    case INST_CSR_FUNCT3_CSRRSI:
        core->x[rd] = *csr;
        *csr |= uimm;
        break;
    case INST_CSR_FUNCT3_CSRRCI:
        core->x[rd] = *csr;
        *csr &= ~uimm;
        break;

    default: TRAP(EXCEPTION_ILLEGAL_INST); break;
    }
    return EXEC_NORET;
}

static uint8_t Execute_Atomic(riscv_core *core, Atomic_type_t *inst)
{
    uint32_t rs1_val = core->x[inst->rs1];
    uint32_t rs2_val = core->x[inst->rs2];
    uint32_t rd_val=0;

    if(core->operations.mem_read(rs1_val, ACCESS_DATA, &rd_val, WORD) == ACCESS_REVOKED)
    {
        TRAP(EXCEPTION_LOAD_ACCESS_FAULT);
        return EXEC_NORET;
    }
    
    switch (inst->funct5)
    {
    case INST_ATOMIC_FUNCT5_LR_W:
        core->load_reserved = 1;
        break;
    case INST_ATOMIC_FUNCT5_SC_W:
        rd_val = (core->load_reserved)? 0:1;
        break;
    case INST_ATOMIC_FUNCT5_AMOSWAP_W:
        break;
    case INST_ATOMIC_FUNCT5_AMOADD_W:
        rs2_val += rd_val;
        break;
    case INST_ATOMIC_FUNCT5_AMOXOR_W:
        rs2_val ^= rd_val;
        break;
    case INST_ATOMIC_FUNCT5_AMOAND_W:
        rs2_val &= rd_val;
        break;
    case INST_ATOMIC_FUNCT5_AMOOR_W:
        rs2_val |= rd_val;
        break;
    case INST_ATOMIC_FUNCT5_AMOMIN_W:
        rs2_val = ((int32_t)rs2_val < (int32_t)rd_val)?rs2_val:rd_val;
        break;
    case INST_ATOMIC_FUNCT5_AMOMAX_W:
        rs2_val = ((int32_t)rs2_val > (int32_t)rd_val)?rs2_val:rd_val;
        break;
    case INST_ATOMIC_FUNCT5_AMOMINU_W:
        rs2_val = (rs2_val < rd_val)?rs2_val:rd_val;
        break;
    case INST_ATOMIC_FUNCT5_AMOMAXU_W:
        rs2_val = (rs2_val > rd_val)?rs2_val:rd_val;
        break;    
    default: TRAP(EXCEPTION_ILLEGAL_INST); break;
    }

    if((!core->trap) && (inst->funct5 != INST_ATOMIC_FUNCT5_LR_W))
    {
        if(core->operations.mem_write(rs1_val, rs2_val, ACCESS_DATA, WORD) == ACCESS_REVOKED)
        {
            TRAP(EXCEPTION_STORE_ACCESS_FAULT);
        }
    }

    if(!core->trap)
    {
        core->x[inst->rd] = rd_val;
    }

    return EXEC_NORET;
}

static uint32_t * GetCSR(riscv_core *core, uint16_t address)
{
    uint32_t * ret = NULL;
    switch(address)
    {
    case CSR_ADDRESS_MSTATUS: ret = (uint32_t *)(&core->mstatus); break;
    case CSR_ADDRESS_MISA: ret = (uint32_t *)(&core->misa); break;
    case CSR_ADDRESS_MIE: ret = (uint32_t *)(&core->mie); break;
    case CSR_ADDRESS_MTVEC: ret = (uint32_t *)(&core->mtvec); break;
    case CSR_ADDRESS_MSCRATCH: ret = (uint32_t *)(&core->mscratch); break;
    case CSR_ADDRESS_MEPC: ret = (uint32_t *)(&core->mepc); break;
    case CSR_ADDRESS_MCAUSE: ret = (uint32_t *)(&core->mcause); break;
    case CSR_ADDRESS_MTVAL: ret = (uint32_t *)(&core->mtval); break;
    case CSR_ADDRESS_MIP: ret = (uint32_t *)(&core->mip); break;
    case CSR_ADDRESS_CYCLE: ret = (uint32_t *)(&core->cycle); break;
    case CSR_ADDRESS_CYCLEH: ret = (uint32_t *)((&core->cycle) + 4); break;
    case CSR_ADDRESS_TIME: ret = (uint32_t *)(&core->time); break;
    case CSR_ADDRESS_TIMEH: ret = (uint32_t *)((&core->time) + 4); break;
    case CSR_ADDRESS_MVENDORID: ret = (uint32_t *)(&core->mvendorid); break;
    }

    return ret;
}