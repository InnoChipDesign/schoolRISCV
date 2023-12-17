# RISC-V simple counter program
#
# Stanislav Zhelnio, 2020
#

            .text

start:      mv a0, zero              #  a0 = 0
                                     # do {
counter:    addi a0, a0, 1           #  a0 = a0 + 1
            beq zero, zero, counter  # } while (true);
