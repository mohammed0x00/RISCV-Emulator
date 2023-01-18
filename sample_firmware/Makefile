all: out.program

CC = riscv64-unknown-elf-
CC_FLAGS = -march=rv32im -mabi=ilp32 -Wl,-Bstatic,-T,$(LD_FILE) -nostdlib -ffreestanding
LD_FILE = linker.ld

out.program: out.elf
	@$(CC)objcopy -O binary $^ $@
	@$(CC)objdump -d $^ > out.lst

out.elf: start.s main.c
	@$(CC)gcc $(CC_FLAGS) $^ -o $@

.PHONY: clean
clean:
	@rm -rf out.program
	@rm -rf out.elf
	@rm -rf out.lst
