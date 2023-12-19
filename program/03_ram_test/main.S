        .text

# Initialize program
init:   mv      t0, zero
        mv      t1, zero
        li      t2, 0x14

# Clear memory
clear:  sw      zero, 1024 (t0)
        addi    t0, t0, 4
        bne     t0, t2, clear

# Loop memory updates
loop:
        lw      a0  1040 (zero)
        lw      a1  1028 (zero)
        mv      t0, zero

# Loop through memory range
update: lw      t1, 1024 (t0)
        add     t1, t1, t0
        sw      t1, 1024 (t0)
        addi    t0, t0, 4
        bne     t0, t2, update
        
        beqz    zero, loop